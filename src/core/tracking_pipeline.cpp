#include "radar/tracking_pipeline.hpp"

#include <chrono>

#include "radar/protocol.hpp"

namespace radar {

using namespace std::chrono_literals;

TrackingPipeline::TrackingPipeline(PipelineConfig config)
    : config_(std::move(config)), queue_(config_.queue_capacity), tracker_(config_.tracker) {}

TrackingPipeline::~TrackingPipeline() {
    stop();
}

void TrackingPipeline::start() {
    if (running_.exchange(true)) {
        return;
    }
    try {
        socket_ = UdpSocket::bind(config_.port, config_.bind_address);
    } catch (...) {
        running_ = false;
        throw;
    }
    bound_port_ = socket_.localPort();
    tracker_thread_ = std::thread(&TrackingPipeline::trackerLoop, this);
    receiver_thread_ = std::thread(&TrackingPipeline::receiverLoop, this);
}

void TrackingPipeline::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    // The receiver notices running_ == false within one poll timeout; closing
    // the queue lets the tracker drain what is left and exit.
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    queue_.close();
    if (tracker_thread_.joinable()) {
        tracker_thread_.join();
    }
}

void TrackingPipeline::receiverLoop() {
    bool have_sequence = false;
    std::uint32_t last_sequence = 0;

    while (running_.load(std::memory_order_relaxed)) {
        auto datagram = socket_.receive(50ms);
        if (!datagram) {
            continue;
        }
        ++datagrams_received_;

        auto batch = protocol::decode(*datagram);
        if (!batch) {
            ++datagrams_rejected_;
            continue;
        }

        // Large sectors are split into several datagrams with the same
        // sequence number, so only a jump of more than one counts as loss.
        if (have_sequence && batch->sequence > last_sequence + 1) {
            sequence_gaps_ += batch->sequence - last_sequence - 1;
        }
        have_sequence = true;
        last_sequence = batch->sequence;

        queue_.push(std::move(*batch));
    }
}

void TrackingPipeline::trackerLoop() {
    std::uint64_t batches = 0;
    std::uint64_t plots = 0;
    const auto persistence_us = static_cast<std::uint64_t>(config_.plot_persistence_s * 1e6);

    while (auto batch = queue_.pop()) {
        const auto t0 = std::chrono::steady_clock::now();
        tracker_.process(*batch);
        const auto t1 = std::chrono::steady_clock::now();

        ++batches;
        plots += batch->plots.size();
        for (const PolarMeasurement& p : batch->plots) {
            recent_plots_.push_back({p, batch->timestamp_us});
        }
        while (!recent_plots_.empty() &&
               recent_plots_.front().timestamp_us + persistence_us < batch->timestamp_us) {
            recent_plots_.pop_front();
        }

        PipelineSnapshot next;
        next.sensor_time_us = batch->timestamp_us;
        next.antenna_azimuth_rad = batch->sector_end_rad;
        next.tracks = tracker_.tracks();
        next.plots.assign(recent_plots_.begin(), recent_plots_.end());
        next.stats.batches_processed = batches;
        next.stats.plots_received = plots;
        next.stats.last_processing_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_ = std::move(next);
    }
}

PipelineSnapshot TrackingPipeline::snapshot() const {
    PipelineSnapshot copy;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        copy = snapshot_;
    }
    copy.stats.datagrams_received = datagrams_received_.load();
    copy.stats.datagrams_rejected = datagrams_rejected_.load();
    copy.stats.sequence_gaps = sequence_gaps_.load();
    copy.stats.batches_dropped = queue_.dropped();
    copy.stats.queue_depth = queue_.size();
    return copy;
}

}  // namespace radar

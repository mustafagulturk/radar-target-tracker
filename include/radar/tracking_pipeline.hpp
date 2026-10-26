#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "radar/thread_safe_queue.hpp"
#include "radar/track_manager.hpp"
#include "radar/types.hpp"
#include "radar/udp_socket.hpp"

namespace radar {

struct PipelineConfig {
    std::uint16_t port = 5005;
    std::string bind_address = "0.0.0.0";
    std::size_t queue_capacity = 256;
    double plot_persistence_s = 4.0;  ///< How long raw plots stay in snapshots.
    TrackerConfig tracker;
};

struct PipelineStats {
    std::uint64_t datagrams_received = 0;
    std::uint64_t datagrams_rejected = 0;  ///< Failed to decode.
    std::uint64_t batches_processed = 0;
    std::uint64_t batches_dropped = 0;  ///< Discarded because the queue was full.
    std::uint64_t plots_received = 0;
    std::uint64_t sequence_gaps = 0;  ///< Lost datagrams detected by sequence number.
    std::size_t queue_depth = 0;
    double last_processing_us = 0.0;  ///< Tracker time spent on the last batch.
};

struct TimedPlot {
    PolarMeasurement plot;
    std::uint64_t timestamp_us = 0;
};

/// Everything a display needs to draw one frame. Produced by the tracker
/// thread and copied out under a mutex, so consumers never touch live state.
struct PipelineSnapshot {
    std::uint64_t sensor_time_us = 0;
    double antenna_azimuth_rad = 0.0;
    std::vector<TrackState> tracks;
    std::vector<TimedPlot> plots;
    PipelineStats stats;
};

/// Two-stage, multithreaded processing chain:
///
///   [UDP socket] --receiver thread--> ThreadSafeQueue<ScanBatch>
///                --tracker thread---> TrackManager --> PipelineSnapshot
///
/// The receiver only decodes and enqueues, so network I/O is never stalled by
/// tracking work. The tracker thread exclusively owns the TrackManager; the
/// only shared state is the queue and the published snapshot.
class TrackingPipeline {
public:
    explicit TrackingPipeline(PipelineConfig config);
    ~TrackingPipeline();

    TrackingPipeline(const TrackingPipeline&) = delete;
    TrackingPipeline& operator=(const TrackingPipeline&) = delete;

    /// Binds the socket and starts both worker threads. Throws on bind failure.
    void start();

    /// Signals both threads to finish and joins them. Safe to call twice.
    void stop();

    bool running() const { return running_.load(); }

    /// Port actually bound (differs from config when config.port == 0).
    std::uint16_t port() const { return bound_port_.load(); }

    PipelineSnapshot snapshot() const;

private:
    void receiverLoop();
    void trackerLoop();

    PipelineConfig config_;
    UdpSocket socket_;
    ThreadSafeQueue<ScanBatch> queue_;
    TrackManager tracker_;

    std::thread receiver_thread_;
    std::thread tracker_thread_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint16_t> bound_port_{0};

    std::atomic<std::uint64_t> datagrams_received_{0};
    std::atomic<std::uint64_t> datagrams_rejected_{0};
    std::atomic<std::uint64_t> sequence_gaps_{0};

    mutable std::mutex snapshot_mutex_;
    PipelineSnapshot snapshot_;
    std::deque<TimedPlot> recent_plots_;  // tracker thread only
};

}  // namespace radar

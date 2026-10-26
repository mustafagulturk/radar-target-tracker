#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <system_error>
#include <thread>

#include "radar/protocol.hpp"
#include "radar/tracking_pipeline.hpp"
#include "radar/udp_socket.hpp"
#include "sim_harness.hpp"

namespace radar {
namespace {

using namespace std::chrono_literals;

/// Polls `condition` until it holds or `timeout` expires.
bool waitFor(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(2ms);
    }
    return condition();
}

TEST(UdpSocket, BindToPortZeroPicksFreePort) {
    UdpSocket sock = UdpSocket::bind(0, "127.0.0.1");
    EXPECT_NE(sock.localPort(), 0);
    EXPECT_GE(sock.nativeHandle(), 0);
}

TEST(UdpSocket, SendAndReceiveOnLoopback) {
    UdpSocket receiver = UdpSocket::bind(0, "127.0.0.1");
    UdpSocket sender;
    sender.connect("127.0.0.1", receiver.localPort());

    const std::vector<std::uint8_t> payload{0x00, 0x01, 0xFE, 0xFF, 0x42};
    ASSERT_TRUE(sender.send(payload));
    const auto received = receiver.receive(2s);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(*received, payload);

    // A full-size protocol datagram arrives intact as well.
    ScanBatch batch;
    batch.sequence = 9;
    batch.plots.assign(protocol::kMaxPlotsPerDatagram, PolarMeasurement{1234.5, 1.0, 3.0});
    const auto bytes = protocol::encode(batch);
    ASSERT_TRUE(sender.send(bytes));
    const auto big = receiver.receive(2s);
    ASSERT_TRUE(big.has_value());
    EXPECT_EQ(*big, bytes);
}

TEST(UdpSocket, ReceiveTimesOut) {
    UdpSocket receiver = UdpSocket::bind(0, "127.0.0.1");
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(receiver.receive(30ms).has_value());
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_GE(elapsed, 25ms);
    EXPECT_LT(elapsed, 2s);
}

TEST(UdpSocket, MoveTransfersOwnership) {
    UdpSocket a = UdpSocket::bind(0, "127.0.0.1");
    const int fd = a.nativeHandle();
    const std::uint16_t port = a.localPort();
    UdpSocket b(std::move(a));
    EXPECT_EQ(b.nativeHandle(), fd);
    EXPECT_EQ(b.localPort(), port);
    EXPECT_EQ(a.nativeHandle(), -1);  // NOLINT(bugprone-use-after-move)

    UdpSocket c;
    c = std::move(b);
    EXPECT_EQ(c.nativeHandle(), fd);
}

TEST(UdpSocket, BindToForeignAddressThrows) {
    // 203.0.113.0/24 is TEST-NET-3: never assigned to a local interface.
    EXPECT_THROW(UdpSocket::bind(0, "203.0.113.7"), std::system_error);
}

PipelineConfig loopbackConfig() {
    PipelineConfig cfg;
    cfg.port = 0;
    cfg.bind_address = "127.0.0.1";
    cfg.queue_capacity = 4096;
    return cfg;
}

TEST(TrackingPipeline, StartAndStopAreIdempotent) {
    TrackingPipeline pipeline(loopbackConfig());
    EXPECT_FALSE(pipeline.running());
    pipeline.stop();  // stop before start is harmless
    pipeline.start();
    pipeline.start();  // second start is a no-op
    EXPECT_TRUE(pipeline.running());
    EXPECT_NE(pipeline.port(), 0);
    pipeline.stop();
    pipeline.stop();
    EXPECT_FALSE(pipeline.running());
}

TEST(TrackingPipeline, DestructorStopsRunningPipeline) {
    auto pipeline = std::make_unique<TrackingPipeline>(loopbackConfig());
    pipeline->start();
    pipeline.reset();  // must join both threads without hanging
    SUCCEED();
}

TEST(TrackingPipeline, StartFailureLeavesPipelineStopped) {
    PipelineConfig cfg = loopbackConfig();
    cfg.bind_address = "203.0.113.7";
    TrackingPipeline pipeline(cfg);
    EXPECT_THROW(pipeline.start(), std::system_error);
    EXPECT_FALSE(pipeline.running());
    pipeline.stop();
}

TEST(TrackingPipeline, RejectsGarbageDatagrams) {
    TrackingPipeline pipeline(loopbackConfig());
    pipeline.start();
    UdpSocket sender;
    sender.connect("127.0.0.1", pipeline.port());

    ASSERT_TRUE(sender.send({0xDE, 0xAD, 0xBE, 0xEF}));
    auto corrupted = protocol::encode(ScanBatch{});
    corrupted[0] = 'X';
    ASSERT_TRUE(sender.send(corrupted));

    EXPECT_TRUE(waitFor([&] { return pipeline.snapshot().stats.datagrams_rejected == 2; }));
    const PipelineSnapshot snap = pipeline.snapshot();
    EXPECT_EQ(snap.stats.datagrams_received, 2U);
    EXPECT_EQ(snap.stats.batches_processed, 0U);
    pipeline.stop();
}

TEST(TrackingPipeline, TracksSimulatedTargetOverUdp) {
    TrackingPipeline pipeline(loopbackConfig());
    pipeline.start();
    UdpSocket sender;
    sender.connect("127.0.0.1", pipeline.port());

    sim::SensorConfig sensor_cfg;
    sensor_cfg.clutter_per_revolution = 10.0;
    test::SimHarness sim(sensor_cfg, 2025);
    sim.addTarget(test::makeTarget(22.0, 130.0, 240.0, 300.0));

    std::uint64_t sent = 0;
    for (int rev = 0; rev < 12; ++rev) {
        for (const ScanBatch& batch : sim.nextRevolution()) {
            for (const ScanBatch& part : protocol::splitBatch(batch)) {
                ASSERT_TRUE(sender.send(protocol::encode(part)));
                ++sent;
            }
        }
        // Pace per revolution so neither the socket buffer nor the queue overflows.
        ASSERT_TRUE(waitFor([&] { return pipeline.snapshot().stats.batches_processed == sent; }))
            << "pipeline stalled at revolution " << rev;
    }

    // One garbage datagram mid-stream must be counted but not disturb tracking.
    ASSERT_TRUE(sender.send({1, 2, 3}));
    ASSERT_TRUE(waitFor([&] { return pipeline.snapshot().stats.datagrams_rejected == 1; }));

    const PipelineSnapshot snap = pipeline.snapshot();
    pipeline.stop();

    EXPECT_EQ(snap.stats.datagrams_received, sent + 1);
    EXPECT_EQ(snap.stats.datagrams_rejected, 1U);
    EXPECT_EQ(snap.stats.batches_processed, sent);
    EXPECT_EQ(snap.stats.batches_dropped, 0U);
    EXPECT_EQ(snap.stats.sequence_gaps, 0U);
    EXPECT_GT(snap.stats.plots_received, 0U);
    EXPECT_FALSE(snap.plots.empty());
    EXPECT_EQ(snap.sensor_time_us, static_cast<std::uint64_t>(sim.timeS() * 1e6));

    const sim::Target& truth = sim.targets().front();
    int confirmed_near = 0;
    for (const TrackState& t : snap.tracks) {
        if (t.status != TrackStatus::Confirmed) {
            continue;
        }
        const double err = std::hypot(t.x - truth.x, t.y - truth.y);
        if (err < 500.0) {
            ++confirmed_near;
            EXPECT_NEAR(t.speed(), truth.speed(), 30.0);
            EXPECT_GE(t.hits, 8);
        }
    }
    EXPECT_EQ(confirmed_near, 1);
}

TEST(TrackingPipeline, DetectsSequenceGaps) {
    TrackingPipeline pipeline(loopbackConfig());
    pipeline.start();
    UdpSocket sender;
    sender.connect("127.0.0.1", pipeline.port());

    for (std::uint32_t seq : {0U, 1U, 1U, 5U}) {  // repeated 1 = split sector, 2..4 lost
        ScanBatch b;
        b.sequence = seq;
        b.timestamp_us = 1000U * (seq + 1U);
        ASSERT_TRUE(sender.send(protocol::encode(b)));
    }
    EXPECT_TRUE(waitFor([&] { return pipeline.snapshot().stats.batches_processed == 4; }));
    EXPECT_EQ(pipeline.snapshot().stats.sequence_gaps, 3U);
    pipeline.stop();
}

}  // namespace
}  // namespace radar

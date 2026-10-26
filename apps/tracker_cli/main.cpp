// Headless tracker: runs the receive/track pipeline and prints the track
// table once per second. Handy over SSH and for checking the pipeline
// without a display.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <thread>

#include "radar/cli.hpp"
#include "radar/coordinates.hpp"
#include "radar/tracking_pipeline.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int /*signal*/) {
    g_stop = true;
}

void printUsage() {
    std::cout << "Usage: tracker_cli [options]\n"
                 "  --port <n>        UDP port to listen on              (default 5005)\n"
                 "  --duration <s>    exit after this many seconds, 0 = run forever\n"
                 "  --all             also list tentative tracks\n";
}

void printSnapshot(const radar::PipelineSnapshot& snap, bool show_tentative) {
    using radar::radToDeg;
    const auto& s = snap.stats;
    std::printf(
        "\n[t=%7.1fs] datagrams=%llu rejected=%llu lost=%llu queue=%zu dropped=%llu  "
        "last batch %.0f us\n",
        static_cast<double>(snap.sensor_time_us) * 1e-6,
        static_cast<unsigned long long>(s.datagrams_received),
        static_cast<unsigned long long>(s.datagrams_rejected),
        static_cast<unsigned long long>(s.sequence_gaps), s.queue_depth,
        static_cast<unsigned long long>(s.batches_dropped), s.last_processing_us);
    std::printf("  %-5s %-10s %9s %8s %8s %8s %8s\n", "ID", "STATUS", "RANGE km", "BRG deg",
                "SPD m/s", "HDG deg", "SIGMA m");
    for (const auto& t : snap.tracks) {
        if (!show_tentative && t.status != radar::TrackStatus::Confirmed) {
            continue;
        }
        std::printf("  %-5u %-10s %9.2f %8.1f %8.1f %8.1f %8.0f\n", t.id, radar::toString(t.status),
                    t.range() / 1000.0, radToDeg(t.azimuth()), t.speed(), radToDeg(t.heading()),
                    t.position_sigma_m);
    }
    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const radar::cli::Args args(argc, argv);
        if (args.has("help")) {
            printUsage();
            return 0;
        }

        radar::PipelineConfig config;
        config.port = static_cast<std::uint16_t>(args.get("port", 5005L));
        const double duration_s = args.get("duration", 0.0);
        const bool show_tentative = args.has("all");

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        radar::TrackingPipeline pipeline(config);
        pipeline.start();
        std::cout << "Listening for radar plots on UDP port " << pipeline.port() << '\n';

        const auto start = std::chrono::steady_clock::now();
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            printSnapshot(pipeline.snapshot(), show_tentative);
            const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
            if (duration_s > 0.0 && elapsed.count() >= duration_s) {
                break;
            }
        }
        pipeline.stop();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "tracker_cli: " << e.what() << '\n';
        return 1;
    }
}

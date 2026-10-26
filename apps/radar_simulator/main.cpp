// Radar simulator: flies a scenario of targets around a rotating radar and
// streams the resulting plots, one datagram per antenna sector, over UDP.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "radar/cli.hpp"
#include "radar/protocol.hpp"
#include "radar/sim/radar_sensor.hpp"
#include "radar/sim/scenario.hpp"
#include "radar/udp_socket.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int /*signal*/) {
    g_stop = true;
}

void printUsage() {
    std::cout << "Usage: radar_simulator [options]\n"
                 "  --host <addr>        destination address        (default 127.0.0.1)\n"
                 "  --port <n>           destination UDP port       (default 5005)\n"
                 "  --range <m>          instrumented range         (default 40000)\n"
                 "  --period <s>         antenna rotation period    (default 2.0)\n"
                 "  --targets <n>        extra random targets       (default 6)\n"
                 "  --pd <p>             detection probability      (default 0.9)\n"
                 "  --clutter <n>        false alarms per scan      (default 20)\n"
                 "  --seed <n>           random seed                (default 42)\n"
                 "  --duration <s>       stop after this many seconds, 0 = run forever\n"
                 "  --speedup <x>        run faster than real time  (default 1)\n";
}

}  // namespace

int main(int argc, char** argv) {
    using namespace radar;
    using Clock = std::chrono::steady_clock;

    try {
        const cli::Args args(argc, argv);
        if (args.has("help")) {
            printUsage();
            return 0;
        }

        sim::SensorConfig sensor_cfg;
        sensor_cfg.max_range_m = args.get("range", sensor_cfg.max_range_m);
        sensor_cfg.rotation_period_s = args.get("period", sensor_cfg.rotation_period_s);
        sensor_cfg.detection_probability = args.get("pd", sensor_cfg.detection_probability);
        sensor_cfg.clutter_per_revolution = args.get("clutter", sensor_cfg.clutter_per_revolution);

        const std::string host = args.get("host", std::string("127.0.0.1"));
        const auto port = static_cast<std::uint16_t>(args.get("port", 5005L));
        const auto seed = static_cast<std::uint32_t>(args.get("seed", 42L));
        const auto extra_targets = static_cast<int>(args.get("targets", 6L));
        const double duration_s = args.get("duration", 0.0);
        const double speedup = std::max(0.01, args.get("speedup", 1.0));

        sim::Scenario scenario =
            sim::Scenario::standard(sensor_cfg.max_range_m, extra_targets, seed);
        sim::RadarSensor sensor(sensor_cfg, seed + 1);

        UdpSocket socket;
        socket.connect(host, port);

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        const int sectors = sensor_cfg.sectors_per_revolution;
        const double sector_dt = sensor_cfg.rotation_period_s / sectors;
        const double sector_width = kTwoPi / sectors;
        const auto wall_step = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(sector_dt / speedup));

        std::cout << "Streaming " << scenario.targets().size() << " targets to " << host << ':'
                  << port << " (" << 60.0 / sensor_cfg.rotation_period_s << " RPM, " << sectors
                  << " sectors/scan). Ctrl+C to stop.\n";

        std::uint64_t sim_time_us = 0;
        std::uint64_t datagrams = 0;
        auto next_wakeup = Clock::now();

        for (long step = 0; !g_stop; ++step) {
            const double sim_time_s = static_cast<double>(step + 1) * sector_dt;
            if (duration_s > 0.0 && sim_time_s > duration_s) {
                break;
            }

            scenario.advance(sector_dt);
            sim_time_us = static_cast<std::uint64_t>(sim_time_s * 1e6);

            const int sector = static_cast<int>(step % sectors);
            const double start = sector * sector_width;
            const ScanBatch batch =
                sensor.scanSector(scenario.targets(), sim_time_us, start, start + sector_width);

            for (const ScanBatch& part : protocol::splitBatch(batch)) {
                if (socket.send(protocol::encode(part))) {
                    ++datagrams;
                }
            }

            if (sector == sectors - 1) {
                std::cout << "t=" << sim_time_s << "s  scans=" << (step + 1) / sectors
                          << "  datagrams=" << datagrams << '\n';
            }

            next_wakeup += wall_step;
            std::this_thread::sleep_until(next_wakeup);
        }

        std::cout << "Sent " << datagrams << " datagrams.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "radar_simulator: " << e.what() << '\n';
        printUsage();
        return 1;
    }
}

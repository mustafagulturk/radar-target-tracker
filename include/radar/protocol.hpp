#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "radar/types.hpp"

namespace radar::protocol {

/// Wire format of one UDP datagram (all fields big-endian / network order):
///
///   offset size  field
///   0      4     magic            'R','D','R','1'
///   4      1     version          1
///   5      1     reserved         0
///   6      2     plot_count       N
///   8      4     sequence         monotonically increasing per sector
///   12     8     timestamp_us     sensor time at end of sector
///   20     4     sector_start     float32, radians
///   24     4     sector_end       float32, radians
///   28     12*N  plots            {range_m, azimuth_rad, amplitude_db} float32
///
/// Floats are sent as IEEE-754 binary32 so the format does not depend on the
/// host's struct padding or endianness.
inline constexpr std::uint32_t kMagic = 0x52445231;  // "RDR1"
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 28;
inline constexpr std::size_t kPlotSize = 12;

/// Keeps each datagram below a typical Ethernet MTU to avoid IP fragmentation.
inline constexpr std::size_t kMaxDatagramSize = 1400;
inline constexpr std::size_t kMaxPlotsPerDatagram = (kMaxDatagramSize - kHeaderSize) / kPlotSize;

/// Serialises a batch. The batch must hold at most kMaxPlotsPerDatagram plots
/// (see splitBatch).
std::vector<std::uint8_t> encode(const ScanBatch& batch);

/// Parses a datagram. Returns std::nullopt for anything malformed: wrong magic
/// or version, truncated payload, trailing bytes, or non-finite values.
std::optional<ScanBatch> decode(const std::uint8_t* data, std::size_t size);

inline std::optional<ScanBatch> decode(const std::vector<std::uint8_t>& bytes) {
    return decode(bytes.data(), bytes.size());
}

/// Splits a batch with many plots into datagram-sized batches that share the
/// same timestamp and sector.
std::vector<ScanBatch> splitBatch(const ScanBatch& batch);

}  // namespace radar::protocol

#include "radar/protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

namespace radar::protocol {
namespace {

ScanBatch makeBatch(std::size_t plot_count) {
    ScanBatch b;
    b.sequence = 0xA1B2C3D4U;
    b.timestamp_us = 0x0102030405060708ULL;
    b.sector_start_rad = 6.1;
    b.sector_end_rad = 0.2;
    for (std::size_t i = 0; i < plot_count; ++i) {
        const double k = static_cast<double>(i);
        b.plots.push_back({1000.0 + 123.456 * k, std::fmod(0.01 * k, 6.28), 5.0 + 0.1 * k});
    }
    return b;
}

void expectFloatEqual(double decoded, double original) {
    EXPECT_EQ(decoded, static_cast<double>(static_cast<float>(original)));
}

void writeFloatAt(std::vector<std::uint8_t>& bytes, std::size_t offset, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (std::size_t i = 0; i < 4; ++i) {
        bytes[offset + i] = static_cast<std::uint8_t>(bits >> (24 - 8 * i));
    }
}

TEST(Protocol, RoundTripPreservesFieldsToFloatPrecision) {
    const ScanBatch in = makeBatch(17);
    const auto bytes = encode(in);
    const auto out = decode(bytes);
    ASSERT_TRUE(out.has_value());

    EXPECT_EQ(out->sequence, in.sequence);
    EXPECT_EQ(out->timestamp_us, in.timestamp_us);
    expectFloatEqual(out->sector_start_rad, in.sector_start_rad);
    expectFloatEqual(out->sector_end_rad, in.sector_end_rad);
    ASSERT_EQ(out->plots.size(), in.plots.size());
    for (std::size_t i = 0; i < in.plots.size(); ++i) {
        expectFloatEqual(out->plots[i].range_m, in.plots[i].range_m);
        expectFloatEqual(out->plots[i].azimuth_rad, in.plots[i].azimuth_rad);
        expectFloatEqual(out->plots[i].amplitude_db, in.plots[i].amplitude_db);
        // float32 is ~1e-7 relative; good to centimetres at 40 km.
        EXPECT_NEAR(out->plots[i].range_m, in.plots[i].range_m, 1e-2);
    }
}

TEST(Protocol, HeaderIsBigEndian) {
    const auto bytes = encode(makeBatch(2));
    ASSERT_GE(bytes.size(), kHeaderSize);
    EXPECT_EQ(bytes[0], 'R');
    EXPECT_EQ(bytes[1], 'D');
    EXPECT_EQ(bytes[2], 'R');
    EXPECT_EQ(bytes[3], '1');
    EXPECT_EQ(bytes[4], kVersion);
    EXPECT_EQ(bytes[5], 0);
    EXPECT_EQ(bytes[6], 0);
    EXPECT_EQ(bytes[7], 2);
    EXPECT_EQ(bytes[8], 0xA1);
    EXPECT_EQ(bytes[11], 0xD4);
    EXPECT_EQ(bytes[12], 0x01);
    EXPECT_EQ(bytes[19], 0x08);
}

TEST(Protocol, DatagramSizeIsExact) {
    for (std::size_t n : {std::size_t{0}, std::size_t{1}, std::size_t{10}, kMaxPlotsPerDatagram}) {
        EXPECT_EQ(encode(makeBatch(n)).size(), kHeaderSize + n * kPlotSize) << n;
    }
    EXPECT_LE(encode(makeBatch(kMaxPlotsPerDatagram)).size(), kMaxDatagramSize);
}

TEST(Protocol, EncodeTruncatesOversizedBatch) {
    const auto bytes = encode(makeBatch(kMaxPlotsPerDatagram + 5));
    EXPECT_EQ(bytes.size(), kHeaderSize + kMaxPlotsPerDatagram * kPlotSize);
    const auto out = decode(bytes);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->plots.size(), kMaxPlotsPerDatagram);
}

TEST(Protocol, EmptyBatchRoundTrips) {
    ScanBatch in;
    in.sequence = 7;
    in.timestamp_us = 123;
    const auto bytes = encode(in);
    EXPECT_EQ(bytes.size(), kHeaderSize);
    const auto out = decode(bytes);
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(out->plots.empty());
    EXPECT_EQ(out->sequence, 7U);
    EXPECT_EQ(out->timestamp_us, 123U);
}

TEST(Protocol, RejectsBadMagic) {
    auto bytes = encode(makeBatch(3));
    bytes[0] = 'X';
    EXPECT_FALSE(decode(bytes).has_value());
}

TEST(Protocol, RejectsBadVersion) {
    auto bytes = encode(makeBatch(3));
    bytes[4] = kVersion + 1;
    EXPECT_FALSE(decode(bytes).has_value());
}

TEST(Protocol, RejectsTruncatedData) {
    const auto bytes = encode(makeBatch(3));
    for (std::size_t len = 0; len < bytes.size(); ++len) {
        EXPECT_FALSE(decode(bytes.data(), len).has_value()) << "len=" << len;
    }
    EXPECT_FALSE(decode(nullptr, 0).has_value());
}

TEST(Protocol, RejectsTrailingBytes) {
    auto bytes = encode(makeBatch(3));
    bytes.push_back(0);
    EXPECT_FALSE(decode(bytes).has_value());
}

TEST(Protocol, RejectsPlotCountMismatch) {
    auto bytes = encode(makeBatch(3));
    bytes[7] = 2;  // claims fewer plots than present
    EXPECT_FALSE(decode(bytes).has_value());
    bytes[7] = 4;  // claims more plots than present
    EXPECT_FALSE(decode(bytes).has_value());
}

TEST(Protocol, RejectsNonFiniteValues) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto good = encode(makeBatch(2));
    ASSERT_TRUE(decode(good).has_value());

    // Sector bounds, and each field of the second plot.
    for (std::size_t offset : {std::size_t{20}, std::size_t{24}, kHeaderSize + kPlotSize,
                               kHeaderSize + kPlotSize + 4, kHeaderSize + kPlotSize + 8}) {
        for (float bad : {nan, inf, -inf}) {
            auto bytes = good;
            writeFloatAt(bytes, offset, bad);
            EXPECT_FALSE(decode(bytes).has_value()) << "offset=" << offset;
        }
    }
}

TEST(Protocol, RejectsNegativeRange) {
    auto bytes = encode(makeBatch(1));
    writeFloatAt(bytes, kHeaderSize, -5.0F);
    EXPECT_FALSE(decode(bytes).has_value());
}

TEST(Protocol, SplitBatchKeepsAllPlotsInOrder) {
    const std::size_t total = 2 * kMaxPlotsPerDatagram + 7;
    const ScanBatch big = makeBatch(total);
    const auto parts = splitBatch(big);
    ASSERT_EQ(parts.size(), 3U);
    EXPECT_EQ(parts[0].plots.size(), kMaxPlotsPerDatagram);
    EXPECT_EQ(parts[1].plots.size(), kMaxPlotsPerDatagram);
    EXPECT_EQ(parts[2].plots.size(), 7U);

    std::size_t index = 0;
    for (const ScanBatch& part : parts) {
        EXPECT_EQ(part.sequence, big.sequence);
        EXPECT_EQ(part.timestamp_us, big.timestamp_us);
        EXPECT_EQ(part.sector_start_rad, big.sector_start_rad);
        EXPECT_EQ(part.sector_end_rad, big.sector_end_rad);

        const auto bytes = encode(part);
        EXPECT_LE(bytes.size(), kMaxDatagramSize);
        const auto decoded = decode(bytes);
        ASSERT_TRUE(decoded.has_value());
        for (const PolarMeasurement& p : decoded->plots) {
            expectFloatEqual(p.range_m, big.plots[index].range_m);
            ++index;
        }
    }
    EXPECT_EQ(index, total);
}

TEST(Protocol, SplitBatchExactMultiple) {
    const auto parts = splitBatch(makeBatch(kMaxPlotsPerDatagram));
    ASSERT_EQ(parts.size(), 1U);
    EXPECT_EQ(parts[0].plots.size(), kMaxPlotsPerDatagram);
}

TEST(Protocol, SplitEmptyBatchYieldsOneEmptyPart) {
    // An empty sector must still be sent so the receiver sees the antenna move.
    const ScanBatch empty = makeBatch(0);
    const auto parts = splitBatch(empty);
    ASSERT_EQ(parts.size(), 1U);
    EXPECT_TRUE(parts[0].plots.empty());
    EXPECT_EQ(parts[0].timestamp_us, empty.timestamp_us);
}

}  // namespace
}  // namespace radar::protocol

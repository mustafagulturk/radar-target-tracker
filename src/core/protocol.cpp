#include "radar/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace radar::protocol {

namespace {

class Writer {
public:
    explicit Writer(std::vector<std::uint8_t>& out) : out_(out) {}

    void u8(std::uint8_t v) { out_.push_back(v); }

    void u16(std::uint16_t v) {
        out_.push_back(static_cast<std::uint8_t>(v >> 8));
        out_.push_back(static_cast<std::uint8_t>(v));
    }

    void u32(std::uint32_t v) {
        for (int shift = 24; shift >= 0; shift -= 8) {
            out_.push_back(static_cast<std::uint8_t>(v >> shift));
        }
    }

    void u64(std::uint64_t v) {
        for (int shift = 56; shift >= 0; shift -= 8) {
            out_.push_back(static_cast<std::uint8_t>(v >> shift));
        }
    }

    void f32(double v) {
        const auto f = static_cast<float>(v);
        std::uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        u32(bits);
    }

private:
    std::vector<std::uint8_t>& out_;
};

class Reader {
public:
    Reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

    std::size_t remaining() const { return size_ - pos_; }

    std::uint8_t u8() { return data_[pos_++]; }

    std::uint16_t u16() {
        const auto v = static_cast<std::uint16_t>((data_[pos_] << 8) | data_[pos_ + 1]);
        pos_ += 2;
        return v;
    }

    std::uint32_t u32() {
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            v = (v << 8) | data_[pos_++];
        }
        return v;
    }

    std::uint64_t u64() {
        std::uint64_t v = 0;
        for (int i = 0; i < 8; ++i) {
            v = (v << 8) | data_[pos_++];
        }
        return v;
    }

    double f32() {
        const std::uint32_t bits = u32();
        float f = 0.0F;
        std::memcpy(&f, &bits, sizeof(f));
        return static_cast<double>(f);
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
};

}  // namespace

std::vector<std::uint8_t> encode(const ScanBatch& batch) {
    const std::size_t count = std::min(batch.plots.size(), kMaxPlotsPerDatagram);

    std::vector<std::uint8_t> out;
    out.reserve(kHeaderSize + count * kPlotSize);
    Writer w(out);
    w.u32(kMagic);
    w.u8(kVersion);
    w.u8(0);
    w.u16(static_cast<std::uint16_t>(count));
    w.u32(batch.sequence);
    w.u64(batch.timestamp_us);
    w.f32(batch.sector_start_rad);
    w.f32(batch.sector_end_rad);
    for (std::size_t i = 0; i < count; ++i) {
        w.f32(batch.plots[i].range_m);
        w.f32(batch.plots[i].azimuth_rad);
        w.f32(batch.plots[i].amplitude_db);
    }
    return out;
}

std::optional<ScanBatch> decode(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size < kHeaderSize) {
        return std::nullopt;
    }

    Reader r(data, size);
    if (r.u32() != kMagic || r.u8() != kVersion) {
        return std::nullopt;
    }
    r.u8();  // reserved
    const std::uint16_t count = r.u16();
    if (r.remaining() != (kHeaderSize - 8) + static_cast<std::size_t>(count) * kPlotSize) {
        return std::nullopt;
    }

    ScanBatch batch;
    batch.sequence = r.u32();
    batch.timestamp_us = r.u64();
    batch.sector_start_rad = r.f32();
    batch.sector_end_rad = r.f32();
    if (!std::isfinite(batch.sector_start_rad) || !std::isfinite(batch.sector_end_rad)) {
        return std::nullopt;
    }

    batch.plots.reserve(count);
    for (std::uint16_t i = 0; i < count; ++i) {
        PolarMeasurement p;
        p.range_m = r.f32();
        p.azimuth_rad = r.f32();
        p.amplitude_db = r.f32();
        if (!std::isfinite(p.range_m) || !std::isfinite(p.azimuth_rad) ||
            !std::isfinite(p.amplitude_db) || p.range_m < 0.0) {
            return std::nullopt;
        }
        batch.plots.push_back(p);
    }
    return batch;
}

std::vector<ScanBatch> splitBatch(const ScanBatch& batch) {
    std::vector<ScanBatch> parts;
    std::size_t offset = 0;
    do {
        ScanBatch part;
        part.sequence = batch.sequence;
        part.timestamp_us = batch.timestamp_us;
        part.sector_start_rad = batch.sector_start_rad;
        part.sector_end_rad = batch.sector_end_rad;
        const std::size_t n = std::min(kMaxPlotsPerDatagram, batch.plots.size() - offset);
        part.plots.assign(batch.plots.begin() + static_cast<std::ptrdiff_t>(offset),
                          batch.plots.begin() + static_cast<std::ptrdiff_t>(offset + n));
        offset += n;
        parts.push_back(std::move(part));
    } while (offset < batch.plots.size());
    return parts;
}

}  // namespace radar::protocol

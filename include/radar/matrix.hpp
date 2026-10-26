#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <optional>

namespace radar {

/// Small fixed-size, row-major matrix. Sizes are compile-time constants so
/// dimension mismatches in filter equations are caught by the compiler and
/// nothing is heap-allocated on the hot path.
template <std::size_t R, std::size_t C>
class Matrix {
public:
    static constexpr std::size_t kRows = R;
    static constexpr std::size_t kCols = C;

    constexpr Matrix() = default;

    Matrix(std::initializer_list<double> values) {
        assert(values.size() == R * C);
        std::copy(values.begin(), values.end(), data_.begin());
    }

    static Matrix zero() { return Matrix{}; }

    static Matrix identity() {
        static_assert(R == C, "identity() requires a square matrix");
        Matrix m;
        for (std::size_t i = 0; i < R; ++i) {
            m(i, i) = 1.0;
        }
        return m;
    }

    double& operator()(std::size_t r, std::size_t c) { return data_[r * C + c]; }
    double operator()(std::size_t r, std::size_t c) const { return data_[r * C + c]; }

    /// Element access for column vectors.
    double& operator[](std::size_t i) {
        static_assert(C == 1, "operator[] is only available on column vectors");
        return data_[i];
    }
    double operator[](std::size_t i) const {
        static_assert(C == 1, "operator[] is only available on column vectors");
        return data_[i];
    }

    Matrix<C, R> transposed() const {
        Matrix<C, R> t;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                t(c, r) = (*this)(r, c);
            }
        }
        return t;
    }

    Matrix& operator+=(const Matrix& other) {
        for (std::size_t i = 0; i < R * C; ++i) {
            data_[i] += other.data_[i];
        }
        return *this;
    }

    Matrix& operator-=(const Matrix& other) {
        for (std::size_t i = 0; i < R * C; ++i) {
            data_[i] -= other.data_[i];
        }
        return *this;
    }

    Matrix& operator*=(double s) {
        for (double& v : data_) {
            v *= s;
        }
        return *this;
    }

    friend Matrix operator+(Matrix a, const Matrix& b) { return a += b; }
    friend Matrix operator-(Matrix a, const Matrix& b) { return a -= b; }
    friend Matrix operator*(Matrix a, double s) { return a *= s; }
    friend Matrix operator*(double s, Matrix a) { return a *= s; }

    friend bool operator==(const Matrix& a, const Matrix& b) { return a.data_ == b.data_; }
    friend bool operator!=(const Matrix& a, const Matrix& b) { return !(a == b); }

private:
    std::array<double, R * C> data_{};
};

template <std::size_t R, std::size_t K, std::size_t C>
Matrix<R, C> operator*(const Matrix<R, K>& a, const Matrix<K, C>& b) {
    Matrix<R, C> out;
    for (std::size_t r = 0; r < R; ++r) {
        for (std::size_t k = 0; k < K; ++k) {
            const double a_rk = a(r, k);
            for (std::size_t c = 0; c < C; ++c) {
                out(r, c) += a_rk * b(k, c);
            }
        }
    }
    return out;
}

using Vec2 = Matrix<2, 1>;
using Vec4 = Matrix<4, 1>;
using Mat2 = Matrix<2, 2>;
using Mat4 = Matrix<4, 4>;

/// Closed-form 2x2 inverse. Returns std::nullopt for (near-)singular input.
inline std::optional<Mat2> inverse(const Mat2& m) {
    const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
    if (std::abs(det) < 1e-12) {
        return std::nullopt;
    }
    const double inv = 1.0 / det;
    return Mat2{m(1, 1) * inv, -m(0, 1) * inv, -m(1, 0) * inv, m(0, 0) * inv};
}

}  // namespace radar

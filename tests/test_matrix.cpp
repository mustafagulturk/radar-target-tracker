#include "radar/matrix.hpp"

#include <gtest/gtest.h>

namespace radar {
namespace {

TEST(Matrix, DefaultIsZero) {
    const Mat2 m;
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 2; ++c) {
            EXPECT_EQ(m(r, c), 0.0);
        }
    }
    EXPECT_EQ(Mat4::zero(), Mat4{});
}

TEST(Matrix, InitializerListIsRowMajor) {
    const Matrix<2, 3> m{1.0, 2.0, 3.0,  //
                         4.0, 5.0, 6.0};
    EXPECT_EQ(m(0, 2), 3.0);
    EXPECT_EQ(m(1, 0), 4.0);
}

TEST(Matrix, MultiplyRectangular) {
    const Matrix<2, 3> a{1.0, 2.0, 3.0,  //
                         4.0, 5.0, 6.0};
    const Matrix<3, 2> b{7.0,  8.0,   //
                         9.0,  10.0,  //
                         11.0, 12.0};
    const Mat2 expected{58.0, 64.0,  //
                        139.0, 154.0};
    EXPECT_EQ(a * b, expected);
}

TEST(Matrix, MultiplyMatrixVector) {
    const Mat2 a{2.0, -1.0,  //
                 0.5, 3.0};
    const Vec2 v{4.0, 2.0};
    const Vec2 out = a * v;
    EXPECT_DOUBLE_EQ(out[0], 6.0);
    EXPECT_DOUBLE_EQ(out[1], 8.0);
}

TEST(Matrix, ElementwiseAndScalarOps) {
    const Mat2 a{1.0, 2.0, 3.0, 4.0};
    const Mat2 b{4.0, 3.0, 2.0, 1.0};
    EXPECT_EQ(a + b, (Mat2{5.0, 5.0, 5.0, 5.0}));
    EXPECT_EQ(a - b, (Mat2{-3.0, -1.0, 1.0, 3.0}));
    EXPECT_EQ(a * 2.0, (Mat2{2.0, 4.0, 6.0, 8.0}));
    EXPECT_EQ(0.5 * a, (Mat2{0.5, 1.0, 1.5, 2.0}));
    EXPECT_NE(a, b);
}

TEST(Matrix, Transpose) {
    const Matrix<2, 3> m{1.0, 2.0, 3.0,  //
                         4.0, 5.0, 6.0};
    const Matrix<3, 2> t = m.transposed();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_EQ(t(c, r), m(r, c));
        }
    }
    EXPECT_EQ(t.transposed(), m);
}

TEST(Matrix, TransposeOfProductReversesOrder) {
    const Mat2 a{1.0, 2.0, 3.0, 4.0};
    const Mat2 b{0.0, -1.0, 5.0, 2.0};
    EXPECT_EQ((a * b).transposed(), b.transposed() * a.transposed());
}

TEST(Matrix, IdentityIsNeutral) {
    const Mat4 i = Mat4::identity();
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            EXPECT_EQ(i(r, c), r == c ? 1.0 : 0.0);
        }
    }
    Mat4 m;
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            m(r, c) = static_cast<double>(r * 4 + c) - 7.5;
        }
    }
    EXPECT_EQ(i * m, m);
    EXPECT_EQ(m * i, m);
}

TEST(Matrix, Inverse2x2) {
    const Mat2 m{4.0, 7.0,  //
                 2.0, 6.0};
    const auto inv = inverse(m);
    ASSERT_TRUE(inv.has_value());
    EXPECT_NEAR((*inv)(0, 0), 0.6, 1e-12);
    EXPECT_NEAR((*inv)(0, 1), -0.7, 1e-12);
    EXPECT_NEAR((*inv)(1, 0), -0.2, 1e-12);
    EXPECT_NEAR((*inv)(1, 1), 0.4, 1e-12);

    const Mat2 product = m * *inv;
    const Mat2 eye = Mat2::identity();
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 2; ++c) {
            EXPECT_NEAR(product(r, c), eye(r, c), 1e-12);
        }
    }
}

TEST(Matrix, InverseOfSingularIsNullopt) {
    EXPECT_FALSE(inverse(Mat2{1.0, 2.0, 2.0, 4.0}).has_value());
    EXPECT_FALSE(inverse(Mat2::zero()).has_value());
}

}  // namespace
}  // namespace radar

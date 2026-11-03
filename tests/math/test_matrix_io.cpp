#include "gtest/gtest.h"
#include "base/atomic_types.h"
#include "math/linalg/vector.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"   // IWYU pragma: keep -- Conversion template definitions.
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"     // IWYU pragma: keep -- Conversion template definitions.
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"     // IWYU pragma: keep -- Conversion template definitions.
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp" // IWYU pragma: keep -- Conversion template definitions.
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"     // IWYU pragma: keep -- Conversion template definitions.
#include "trackingLib/math/linalg/diagonal_matrix.h"
#include "trackingLib/math/linalg/matrix.h"
#include "trackingLib/math/linalg/matrix_io.h"
#include "trackingLib/math/linalg/square_matrix.h"
#include "trackingLib/math/linalg/triangular_matrix.h"
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

using tracking::math::DiagonalMatrix;
using tracking::math::Matrix;
using tracking::math::SquareMatrix;
using tracking::math::TriangularMatrix;
using tracking::math::Vector;

TEST(MatrixIO, BasicMatrixOutput) // NOLINT
{
  auto matResult = Matrix<float32, 2, 2, true>::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.0000000000,    +2.0000000000\n"
                               "   +3.0000000000,    +4.0000000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, IntegerMatrixOutput) // NOLINT
{
  auto matResult = Matrix<sint32, 2, 2, true>::FromList({{1, 2}, {3, 4}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "       1,        2\n"
                               "       3,        4\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, SquareMatrixOutput) // NOLINT
{
  auto matResult = SquareMatrix<float32, 3, true>::FromList({{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {7.0F, 8.0F, 9.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.0000000000,    +2.0000000000,    +3.0000000000\n"
                               "   +4.0000000000,    +5.0000000000,    +6.0000000000\n"
                               "   +7.0000000000,    +8.0000000000,    +9.0000000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, DiagonalMatrixOutput) // NOLINT
{
  auto matResult = DiagonalMatrix<float32, 3>::FromList({1.0F, 2.0F, 3.0F});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.0000000000,    +0.0000000000,    +0.0000000000\n"
                               "   +0.0000000000,    +2.0000000000,    +0.0000000000\n"
                               "   +0.0000000000,    +0.0000000000,    +3.0000000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, TriangularMatrixOutput) // NOLINT
{
  auto matResult =
      TriangularMatrix<float32, 3, true, true>::FromList({{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 0.0F}, {4.0F, 5.0F, 6.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.0000000000,    +0.0000000000,    +0.0000000000\n"
                               "   +2.0000000000,    +3.0000000000,    +0.0000000000\n"
                               "   +4.0000000000,    +5.0000000000,    +6.0000000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, FileOutput) // NOLINT
{
  auto matResult = Matrix<float32, 2, 2, true>::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::ofstream file("test_matrix_io_output.txt");
  file << mat;
  file.close();

  // Verify file was created and has content
  std::ifstream     inFile("test_matrix_io_output.txt");
  const std::string content((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
  inFile.close();

  EXPECT_FALSE(content.empty());
  EXPECT_NE(content.find("+1.000000"), std::string::npos);
}

TEST(MatrixIO, ChainingOutput) // NOLINT
{
  auto matResult = Matrix<float32, 2, 2, true>::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << "Matrix: " << mat << "End";

  const std::string result = ss.str();
  EXPECT_NE(result.find("Matrix:"), std::string::npos);
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("End"), std::string::npos);
}

TEST(MatrixIO, DifferentValueTypes) // NOLINT
{
  // Test float64
  auto float64MatResult = Matrix<float64, 2, 2, true>::FromList({{1.0, 2.0}, {3.0, 4.0}});
  ASSERT_TRUE(float64MatResult.has_value());
  const auto float64Mat = float64MatResult.value();

  std::stringstream ss1;
  ss1 << float64Mat;
  EXPECT_NE(ss1.str().find("+1.000000"), std::string::npos);

  // Test sint32
  auto sint32MatResult = Matrix<sint32, 2, 2, true>::FromList({{1, 2}, {3, 4}});
  ASSERT_TRUE(sint32MatResult.has_value());
  const auto sint32Mat = sint32MatResult.value();

  std::stringstream ss2;
  ss2 << sint32Mat;
  EXPECT_NE(ss2.str().find('1'), std::string::npos);
  EXPECT_EQ(ss2.str().find("+1.000000"), std::string::npos); // Should not have decimal places
}

TEST(MatrixIO, SingleElementMatrix) // NOLINT
{
  auto matResult = Matrix<float32, 1, 1, true>::FromList({{42.0F}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();

  std::stringstream ss;
  ss << mat;

  const std::string expected = "  +42.0000000000\n";
  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, LargeMatrix) // NOLINT
{
  // clang-format off
  auto matResult = Matrix<float32, 4, 4, true>::FromList({
  {1.0F, 2.0F, 3.0F, 4.0F}, 
  {5.0F, 6.0F, 7.0F, 8.0F}, 
  {9.0F, 10.0F, 11.0F, 12.0F}, 
  {13.0F, 14.0F, 15.0F, 16.0F}
  });
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows = 4 newlines)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

// Critical missing tests for Matrix I/O functions - addressing 0% coverage

TEST(MatrixIO, MatrixDouble4x3RowMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = Matrix<double, 4, 3, true>::FromList({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, MatrixDouble6x3RowMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = Matrix<double, 6, 3, true>::FromList({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0},
      {16.0, 17.0, 18.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+18.000000"), std::string::npos);

  // Count newlines (should be 6 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 6);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = SquareMatrix<double, 4, true>::FromList({
      {1.0, 2.0, 3.0, 4.0},
      {5.0, 6.0, 7.0, 8.0},
      {9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, SquareMatrixDouble6x6RowMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = SquareMatrix<double, 6, true>::FromList({
      {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0, 17.0, 18.0},
      {19.0, 20.0, 21.0, 22.0, 23.0, 24.0},
      {25.0, 26.0, 27.0, 28.0, 29.0, 30.0},
      {31.0, 32.0, 33.0, 34.0, 35.0, 36.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+36.000000"), std::string::npos);

  // Count newlines (should be 6 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 6);
}

TEST(MatrixIO, MatrixDouble4x3ColumnMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = Matrix<double, 4, 3, false>::FromList({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, SquareMatrixDouble4x4ColumnMajor__Success) // NOLINT
{
  // clang-format off
  auto matResult = SquareMatrix<double, 4, false>::FromList({
      {1.0, 2.0, 3.0, 4.0},
      {5.0, 6.0, 7.0, 8.0},
      {9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, MatrixDouble4x3RowMajor_NegativeValues__Success) // NOLINT
{
  // clang-format off
  auto matResult = Matrix<double, 4, 3, true>::FromList({
      {-1.0, -2.0, 3.0},
      {4.0, -5.0, -6.0},
      {-7.0, 8.0, -9.0},
      {10.0, -11.0, 12.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("-1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor_NegativeValues__Success) // NOLINT
{
  // clang-format off
  auto matResult = SquareMatrix<double, 4, true>::FromList({
      {-1.0, -2.0, -3.0, -4.0},
      {-5.0, -6.0, -7.0, -8.0},
      {-9.0, -10.0, -11.0, -12.0},
      {-13.0, -14.0, -15.0, -16.0}});
  ASSERT_TRUE(matResult.has_value());
  const auto mat = matResult.value();
  // clang-format on

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  EXPECT_NE(result.find("-1.000000"), std::string::npos);
  EXPECT_NE(result.find("-16.000000"), std::string::npos);
}

TEST(MatrixIO, MatrixDouble4x3RowMajor_ZeroMatrix__Success) // NOLINT
{
  using MatrixType = Matrix<double, 4, 3, true>;
  const auto mat   = MatrixType::Zeros();

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  // All values should be +0.000000
  EXPECT_NE(result.find("+0.000000"), std::string::npos);
  // Count occurrences of zero (should be 12 for 4x3 matrix)
  const std::size_t zeroCount = std::count(result.begin(), result.end(), '+');
  EXPECT_EQ(zeroCount, 12);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor_ZeroMatrix__Success) // NOLINT
{
  using MatrixType = SquareMatrix<double, 4, true>;
  const auto mat   = MatrixType::Zeros();

  std::stringstream ss;
  ss << mat;

  const std::string result = ss.str();
  // All values should be +0.000000
  EXPECT_NE(result.find("+0.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  const std::size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, VectorFloat3__Success) // NOLINT
{
  auto vecResult = Vector<float32, 3>::FromList({1.0F, 2.0F, 3.0F});
  ASSERT_TRUE(vecResult.has_value());
  const auto vec = vecResult.value();

  std::stringstream ss;
  ss << vec;

  const std::string expected = "   +1.0000000000\n"
                               "   +2.0000000000\n"
                               "   +3.0000000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, VectorInt4__Success) // NOLINT
{
  auto vecResult = Vector<sint32, 4>::FromList({10, 20, 30, 40});
  ASSERT_TRUE(vecResult.has_value());
  const auto vec = vecResult.value();

  std::stringstream ss;
  ss << vec;

  const std::string expected = "      10\n"
                               "      20\n"
                               "      30\n"
                               "      40\n";

  EXPECT_EQ(ss.str(), expected);
}

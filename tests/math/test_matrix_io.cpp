#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/diagonal_conversions.hpp"
#include "trackingLib/math/linalg/conversions/matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/conversions/triangular_conversions.hpp"
#include "trackingLib/math/linalg/conversions/vector_conversions.hpp"
#include "trackingLib/math/linalg/diagonal_matrix.h"
#include "trackingLib/math/linalg/matrix.h"
#include "trackingLib/math/linalg/matrix_io.h"
#include "trackingLib/math/linalg/square_matrix.h"
#include "trackingLib/math/linalg/triangular_matrix.h"
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace tracking::math;

TEST(MatrixIO, BasicMatrixOutput) // NOLINT
{
  const auto mat = conversions::MatrixFromList<float32, 2, 2, true>({{1.0F, 2.0F}, {3.0F, 4.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.000000,    +2.000000\n"
                               "   +3.000000,    +4.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, IntegerMatrixOutput) // NOLINT
{
  const auto mat = conversions::MatrixFromList<sint32, 2, 2, true>({{1, 2}, {3, 4}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "       1,        2\n"
                               "       3,        4\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, SquareMatrixOutput) // NOLINT
{
  const auto mat = conversions::SquareFromList<float32, 3, true>({{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {7.0F, 8.0F, 9.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.000000,    +2.000000,    +3.000000\n"
                               "   +4.000000,    +5.000000,    +6.000000\n"
                               "   +7.000000,    +8.000000,    +9.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, DiagonalMatrixOutput) // NOLINT
{
  const auto mat = conversions::DiagonalFromList<float32, 3>({1.0F, 2.0F, 3.0F});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.000000,    +0.000000,    +0.000000\n"
                               "   +0.000000,    +2.000000,    +0.000000\n"
                               "   +0.000000,    +0.000000,    +3.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, TriangularMatrixOutput) // NOLINT
{
  const auto mat =
      conversions::TriangularFromList<float32, 3, true, true>({{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 0.0F}, {4.0F, 5.0F, 6.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "   +1.000000,    +0.000000,    +0.000000\n"
                               "   +2.000000,    +3.000000,    +0.000000\n"
                               "   +4.000000,    +5.000000,    +6.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, FileOutput) // NOLINT
{
  const auto mat = conversions::MatrixFromList<float32, 2, 2, true>({{1.0F, 2.0F}, {3.0F, 4.0F}});

  std::ofstream file("test_matrix_io_output.txt");
  file << mat;
  file.close();

  // Verify file was created and has content
  std::ifstream inFile("test_matrix_io_output.txt");
  std::string   content((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
  inFile.close();

  EXPECT_FALSE(content.empty());
  EXPECT_NE(content.find("+1.000000"), std::string::npos);
}

TEST(MatrixIO, ChainingOutput) // NOLINT
{
  const auto mat = conversions::MatrixFromList<float32, 2, 2, true>({{1.0F, 2.0F}, {3.0F, 4.0F}});

  std::stringstream ss;
  ss << "Matrix: " << mat << "End";

  std::string result = ss.str();
  EXPECT_NE(result.find("Matrix:"), std::string::npos);
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("End"), std::string::npos);
}

TEST(MatrixIO, DifferentValueTypes) // NOLINT
{
  // Test float64
  const auto float64Mat = conversions::MatrixFromList<float64, 2, 2, true>({{1.0, 2.0}, {3.0, 4.0}});

  std::stringstream ss1;
  ss1 << float64Mat;
  EXPECT_NE(ss1.str().find("+1.000000"), std::string::npos);

  // Test sint32
  const auto sint32Mat = conversions::MatrixFromList<sint32, 2, 2, true>({{1, 2}, {3, 4}});

  std::stringstream ss2;
  ss2 << sint32Mat;
  EXPECT_NE(ss2.str().find("1"), std::string::npos);
  EXPECT_EQ(ss2.str().find("+1.000000"), std::string::npos); // Should not have decimal places
}

TEST(MatrixIO, SingleElementMatrix) // NOLINT
{
  const auto mat = conversions::MatrixFromList<float32, 1, 1, true>({{42.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "  +42.000000\n";
  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, LargeMatrix) // NOLINT
{
  const auto mat = conversions::MatrixFromList<float32, 4, 4, true>(
      {{1.0F, 2.0F, 3.0F, 4.0F}, {5.0F, 6.0F, 7.0F, 8.0F}, {9.0F, 10.0F, 11.0F, 12.0F}, {13.0F, 14.0F, 15.0F, 16.0F}});

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows = 4 newlines)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

// Critical missing tests for Matrix I/O functions - addressing 0% coverage

TEST(MatrixIO, MatrixDouble4x3RowMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::MatrixFromList<double, 4, 3, true>({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, MatrixDouble6x3RowMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::MatrixFromList<double, 6, 3, true>({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0},
      {16.0, 17.0, 18.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+18.000000"), std::string::npos);

  // Count newlines (should be 6 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 6);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::SquareFromList<double, 4, true>({
      {1.0, 2.0, 3.0, 4.0},
      {5.0, 6.0, 7.0, 8.0},
      {9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, SquareMatrixDouble6x6RowMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::SquareFromList<double, 6, true>({
      {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0, 17.0, 18.0},
      {19.0, 20.0, 21.0, 22.0, 23.0, 24.0},
      {25.0, 26.0, 27.0, 28.0, 29.0, 30.0},
      {31.0, 32.0, 33.0, 34.0, 35.0, 36.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+36.000000"), std::string::npos);

  // Count newlines (should be 6 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 6);
}

TEST(MatrixIO, MatrixDouble4x3ColumnMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::MatrixFromList<double, 4, 3, false>({
      {1.0, 2.0, 3.0},
      {4.0, 5.0, 6.0},
      {7.0, 8.0, 9.0},
      {10.0, 11.0, 12.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, SquareMatrixDouble4x4ColumnMajor__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::SquareFromList<double, 4, false>({
      {1.0, 2.0, 3.0, 4.0},
      {5.0, 6.0, 7.0, 8.0},
      {9.0, 10.0, 11.0, 12.0},
      {13.0, 14.0, 15.0, 16.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("+1.000000"), std::string::npos);
  EXPECT_NE(result.find("+16.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, MatrixDouble4x3RowMajor_NegativeValues__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::MatrixFromList<double, 4, 3, true>({
      {-1.0, -2.0, 3.0},
      {4.0, -5.0, -6.0},
      {-7.0, 8.0, -9.0},
      {10.0, -11.0, 12.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("-1.000000"), std::string::npos);
  EXPECT_NE(result.find("+12.000000"), std::string::npos);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor_NegativeValues__Success) // NOLINT
{
  // clang-format off
  const auto mat = conversions::SquareFromList<double, 4, true>({
      {-1.0, -2.0, -3.0, -4.0},
      {-5.0, -6.0, -7.0, -8.0},
      {-9.0, -10.0, -11.0, -12.0},
      {-13.0, -14.0, -15.0, -16.0}});
  // clang-format on

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  EXPECT_NE(result.find("-1.000000"), std::string::npos);
  EXPECT_NE(result.find("-16.000000"), std::string::npos);
}

TEST(MatrixIO, MatrixDouble4x3RowMajor_ZeroMatrix__Success) // NOLINT
{
  using MatrixType = Matrix<double, 4, 3, true>;
  const auto mat   = MatrixType::Zeros();

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  // All values should be +0.000000
  EXPECT_NE(result.find("+0.000000"), std::string::npos);
  // Count occurrences of zero (should be 12 for 4x3 matrix)
  size_t zeroCount = std::count(result.begin(), result.end(), '+');
  EXPECT_EQ(zeroCount, 12);
}

TEST(MatrixIO, SquareMatrixDouble4x4RowMajor_ZeroMatrix__Success) // NOLINT
{
  using MatrixType = SquareMatrix<double, 4, true>;
  const auto mat   = MatrixType::Zeros();

  std::stringstream ss;
  ss << mat;

  std::string result = ss.str();
  // All values should be +0.000000
  EXPECT_NE(result.find("+0.000000"), std::string::npos);

  // Count newlines (should be 4 rows)
  size_t newlineCount = std::count(result.begin(), result.end(), '\n');
  EXPECT_EQ(newlineCount, 4);
}

TEST(MatrixIO, VectorFloat3__Success) // NOLINT
{
  const auto vec = conversions::VectorFromList<float32, 3>({1.0F, 2.0F, 3.0F});

  std::stringstream ss;
  ss << vec;

  const std::string expected = "   +1.000000\n"
                               "   +2.000000\n"
                               "   +3.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, VectorInt4__Success) // NOLINT
{
  const auto vec = conversions::VectorFromList<sint32, 4>({10, 20, 30, 40});

  std::stringstream ss;
  ss << vec;

  const std::string expected = "      10\n"
                               "      20\n"
                               "      30\n"
                               "      40\n";

  EXPECT_EQ(ss.str(), expected);
}

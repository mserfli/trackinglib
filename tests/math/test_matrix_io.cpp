#include "gtest/gtest.h"
#include "trackingLib/math/linalg/diagonal_matrix.h"
#include "trackingLib/math/linalg/matrix.h"
#include "trackingLib/math/linalg/matrix_io.h"
#include "trackingLib/math/linalg/square_matrix.h"
#include "trackingLib/math/linalg/triangular_matrix.h"
#include "trackingLib/math/linalg/vector.h"
#include <algorithm>
#include <fstream>
#include <sstream>

TEST(MatrixIO, BasicMatrixOutput) // NOLINT
{
  using MatrixType = tracking::math::Matrix<float32, 2, 2, true>;
  const auto mat   = MatrixType::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "     +1.000000,      +2.000000\n"
                               "     +3.000000,      +4.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, IntegerMatrixOutput) // NOLINT
{
  using MatrixType = tracking::math::Matrix<sint32, 2, 2, true>;
  const auto mat   = MatrixType::FromList({{1, 2}, {3, 4}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "       1,        2\n"
                               "       3,        4\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, SquareMatrixOutput) // NOLINT
{
  using MatrixType = tracking::math::SquareMatrix<float32, 3, true>;
  const auto mat   = MatrixType::FromList({{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {7.0F, 8.0F, 9.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "     +1.000000,      +2.000000,      +3.000000\n"
                               "     +4.000000,      +5.000000,      +6.000000\n"
                               "     +7.000000,      +8.000000,      +9.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, DiagonalMatrixOutput) // NOLINT
{
  using MatrixType = tracking::math::DiagonalMatrix<float32, 3>;
  const auto mat   = MatrixType::FromList({1.0F, 2.0F, 3.0F});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "     +1.000000,      +0.000000,      +0.000000\n"
                               "     +0.000000,      +2.000000,      +0.000000\n"
                               "     +0.000000,      +0.000000,      +3.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, TriangularMatrixOutput) // NOLINT
{
  using MatrixType = tracking::math::TriangularMatrix<float32, 3, true, true>; // Lower triangular
  const auto mat   = MatrixType::FromList({{1.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 0.0F}, {4.0F, 5.0F, 6.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "     +1.000000,      +0.000000,      +0.000000\n"
                               "     +2.000000,      +3.000000,      +0.000000\n"
                               "     +4.000000,      +5.000000,      +6.000000\n";

  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, FileOutput) // NOLINT
{
  using MatrixType = tracking::math::Matrix<float32, 2, 2, true>;
  const auto mat   = MatrixType::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});

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
  using MatrixType = tracking::math::Matrix<float32, 2, 2, true>;
  const auto mat   = MatrixType::FromList({{1.0F, 2.0F}, {3.0F, 4.0F}});

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
  using Float64Matrix   = tracking::math::Matrix<float64, 2, 2, true>;
  const auto float64Mat = Float64Matrix::FromList({{1.0, 2.0}, {3.0, 4.0}});

  std::stringstream ss1;
  ss1 << float64Mat;
  EXPECT_NE(ss1.str().find("+1.000000"), std::string::npos);

  // Test sint32
  using Sint32Matrix   = tracking::math::Matrix<sint32, 2, 2, true>;
  const auto sint32Mat = Sint32Matrix::FromList({{1, 2}, {3, 4}});

  std::stringstream ss2;
  ss2 << sint32Mat;
  EXPECT_NE(ss2.str().find("1"), std::string::npos);
  EXPECT_EQ(ss2.str().find("+1.000000"), std::string::npos); // Should not have decimal places
}

TEST(MatrixIO, EmptyMatrix) // NOLINT
{
  using MatrixType = tracking::math::Matrix<float32, 0, 0, true>;
  const auto mat   = MatrixType{};

  std::stringstream ss;
  ss << mat;

  // Should produce empty output for 0x0 matrix
  EXPECT_EQ(ss.str(), "");
}

TEST(MatrixIO, SingleElementMatrix) // NOLINT
{
  using MatrixType = tracking::math::Matrix<float32, 1, 1, true>;
  const auto mat   = MatrixType::FromList({{42.0F}});

  std::stringstream ss;
  ss << mat;

  const std::string expected = "     +42.000000\n";
  EXPECT_EQ(ss.str(), expected);
}

TEST(MatrixIO, LargeMatrix) // NOLINT
{
  using MatrixType = tracking::math::Matrix<float32, 4, 4, true>;
  const auto mat   = MatrixType::FromList(
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

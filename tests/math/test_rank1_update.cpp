#include "gtest/gtest.h"
#include "trackingLib/math/linalg/rank1_update.hpp" // IWYU pragma: keep

using UpperTriangularMatrix = tracking::math::TriangularMatrix<float64, 4, false, true>;
using LowerTriangularMatrix = tracking::math::TriangularMatrix<float64, 4, true, true>;
using DiagonalMatrix        = tracking::math::DiagonalMatrix<float64, 4>;

TEST(Rank1Update, udu_update) // NOLINT
{
  // clang-format off
  auto U = UpperTriangularMatrix::FromList({
    {1.000000000000000,  1.100446217617032,  0.451685496026601,  0.566995242706580},
    {                0,  1.000000000000000, -1.706345585301695, -0.639830177853812},
    {                0,                  0,  1.000000000000000,  0.379147889723543},
    {                0,                  0,                  0,  1.000000000000000}
  });
  auto D = DiagonalMatrix::FromList({
  1.744764006421280e+00, 2.623463785299294e-01, 1.298863681833844e+00, 5.511441032938980e-02
  });
  const auto w = tracking::math::Vector<float64, 4>::FromList({
    3.829541723464068e-01, 1.613328887439391e-01, 9.379321662496187e-02, 5.884958503513444e-01
  });
  const auto sigma = 3.0;

  const auto expU = UpperTriangularMatrix::FromList({
    {1.000000000000000,  1.058842071465176,  0.450068003458539,  0.646515574457400},
    {                0,  1.000000000000000, -1.711109275424165,  0.228103592207166},
    {                0,                  0,  1.000000000000000,  0.170448646290043},
    {                0,                  0,                  0,  1.000000000000000}
  });
  const auto expD = DiagonalMatrix::FromList({
    1.753069000330330,  0.277520518292310,  1.301391550356693,  1.094096507971646
  });
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<float64, 4, true>::run(U, D, sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D.at_unsafe(row), expD.at_unsafe(row));
    for (auto col = row; col < 4; ++col)
    {
      EXPECT_FLOAT_EQ(U.at_unsafe(row, col), expU.at_unsafe(row, col));
    }
  }
}

TEST(Rank1Update, ldl_update) // NOLINT
{
  // clang-format off
  auto L = LowerTriangularMatrix::FromList({
    {1.000000000000000,  0.000000000000000,  0.000000000000000,  0.000000000000000},
    {1.100446217617032,  1.000000000000000,  0.000000000000000,  0.000000000000000},
    {0.451685496026601, -1.706345585301695,  1.000000000000000,  0.000000000000000},
    {0.566995242706580, -0.639830177853812,  0.379147889723543,  1.000000000000000}
  });
  auto D = DiagonalMatrix::FromList({
      1.744764006421280e+00, 2.623463785299294e-01, 1.298863681833844e+00, 5.511441032938980e-02
  });
  const auto w = tracking::math::Vector<float64, 4>::FromList({
    3.829541723464068e-01, 1.613328887439391e-01, 9.379321662496187e-02, 5.884958503513444e-01
  });
  const auto sigma = 3.0;

  const auto expL = LowerTriangularMatrix::FromList({
    {1.000000000000000,                  0,                  0,                  0},
    {0.963676245213974,  1.000000000000000,                  0,                  0},
    {0.410046948367661, -0.938497570939665,  1.000000000000000,                  0},
    {0.762280461386267, -0.940741810056261,  0.195856283828150,  1.000000000000000}  
  });
  const auto expD = DiagonalMatrix::FromList({
    2.184725700773844, 0.424415494704770, 1.703921082760869, 0.238675924218862
  });
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<float64, 4, true>::run(L, D, sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D.at_unsafe(row), expD.at_unsafe(row));
    for (auto col = 0; col < row; ++col)
    {
      EXPECT_FLOAT_EQ(L.at_unsafe(row, col), expL.at_unsafe(row, col));
    }
  }
}

TEST(Rank1Update, ldl_downdate) // NOLINT
{
  // clang-format off
  auto L = LowerTriangularMatrix::FromList({
    {1.000000000000000,                  0,                  0,                  0},
    {0.963676245213974,  1.000000000000000,                  0,                  0},
    {0.410046948367661, -0.938497570939665,  1.000000000000000,                  0},
    {0.762280461386267, -0.940741810056261,  0.195856283828150,  1.000000000000000}
  });
  auto D = DiagonalMatrix::FromList({
    2.184725700773844, 0.424415494704770, 1.703921082760869, 0.238675924218862
  });
  const auto w = tracking::math::Vector<float64, 4>::FromList({
    3.829541723464068e-01, 1.613328887439391e-01, 9.379321662496187e-02, 5.884958503513444e-01
  });
  const auto sigma = 3.0;

  const auto expL = LowerTriangularMatrix::FromList({
    {1.000000000000000,                  0,                  0,                  0},
    {1.100446217617032,  1.000000000000000,                  0,                  0},
    {0.451685496026601, -1.706345585301694,  1.000000000000000,                  0},
    {0.566995242706581, -0.639830177853813,  0.379147889723543,  1.000000000000000}
  });
  const auto expD = DiagonalMatrix::FromList({
    1.744764006421280e+00, 2.623463785299295e-01, 1.298863681833845e+00, 5.511441032938941e-02
  });
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<float64, 4, true>::run(L, D, -sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D.at_unsafe(row), expD.at_unsafe(row));
    for (auto col = 0; col < row; ++col)
    {
      EXPECT_FLOAT_EQ(L.at_unsafe(row, col), expL.at_unsafe(row, col));
    }
  }
}

TEST(Rank1Update, ldl_downdate_belowEpsilon) // NOLINT
{
  // clang-format off
  auto L = LowerTriangularMatrix::FromList({
    {1.000000000000000,                  0,                  0,                  0},
    {0.963676245213974,  1.000000000000000,                  0,                  0},
    {0.410046948367661, -0.938497570939665,  1.000000000000000,                  0},
    {0.762280461386267, -0.940741810056261,  0.195856283828150,  1.000000000000000}
  });
  auto D = DiagonalMatrix::FromList({
    2.184725700773844, 0.424415494704770, 1.703921082760869, 0.238675924218862
  });
  const auto w = tracking::math::Vector<float64, 4>::FromList({
    3.829541723464068e-01, 1.613328887439391e-01, 9.379321662496187e-02, 5.884958503513444e-01
  });
  const auto sigma = 20.0;

  const auto expL = LowerTriangularMatrix::FromList({
    {1.000000000000000,                  0,                  0,                  0},
    {1.117144553721805,  1.000000000000000,                  0,                  0},
    {0.456769173763075, -1.870378532677200,  1.000000000000000,                  0},
    {0.543152741604089, -0.575547375780341,  0.434181260080981,  1.000000000000000}
  });
  const auto expD = DiagonalMatrix::FromList({
    1.702895413761838e+00, 2.425592532820298e-01, 1.212332547953755e+00, 2.746814709096464e-17
  });
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<float64, 4, true>::run(L, D, -sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D.at_unsafe(row), expD.at_unsafe(row));
    for (auto col = 0; col < row; ++col)
    {
      EXPECT_FLOAT_EQ(L.at_unsafe(row, col), expL.at_unsafe(row, col));
    }
  }
}
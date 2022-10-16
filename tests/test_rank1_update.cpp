#include "gtest/gtest.h"
#include "trackingLib/math/linalg/rank1_update.h"

TEST(Rank1Update, ldl_update)
{
  // clang-format off
  tracking::math::TriangularMatrix<double, 4, true> L(
    {{1.000000000000000,                  0,                  0,                  0},
     {1.100446217617032,  1.000000000000000,                  0,                  0},
     {0.451685496026601, -1.706345585301695,  1.000000000000000,                  0},
     {0.566995242706580, -0.639830177853812,  0.379147889723543,  1.000000000000000}});

  tracking::math::DiagonalMatrix<double, 4> D(
    {1.744764006421280e+00, 2.623463785299294e-01, 1.298863681833844e+00, 5.511441032938980e-02});

  tracking::math::TriangularMatrix<double, 4, true> expL(
    {{1.000000000000000,                  0,                  0,                  0},
     {0.963676245213974,  1.000000000000000,                  0,                  0},
     {0.410046948367661, -0.938497570939665,  1.000000000000000,                  0},
     {0.762280461386267, -0.940741810056261,  0.195856283828150,  1.000000000000000}});

  tracking::math::DiagonalMatrix<double, 4> expD(
    {2.184725700773844, 0.424415494704770, 1.703921082760869, 0.238675924218862});

  tracking::math::Matrix<double, 4,1> w(
    {{3.829541723464068e-01}, {1.613328887439391e-01}, {9.379321662496187e-02}, {5.884958503513444e-01}});
  auto sigma = 3.0;
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<double, 4>::run(L, D, sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D[row], expD[row]);
    for (auto col = 0; col < row; ++col)
    {
      EXPECT_FLOAT_EQ(L(row, col), expL(row, col));
    }
  }
}

TEST(Rank1Update, ldl_downdate)
{
  // clang-format off
  tracking::math::TriangularMatrix<double, 4, true> L(
    {{1.000000000000000,                  0,                  0,                  0},
     {0.963676245213974,  1.000000000000000,                  0,                  0},
     {0.410046948367661, -0.938497570939665,  1.000000000000000,                  0},
     {0.762280461386267, -0.940741810056261,  0.195856283828150,  1.000000000000000}});

  tracking::math::DiagonalMatrix<double, 4> D(
    {2.184725700773844, 0.424415494704770, 1.703921082760869, 0.238675924218862});

  tracking::math::TriangularMatrix<double, 4, true> expL(
    {{1.000000000000000,                  0,                  0,                  0},
     {1.100446217617032,  1.000000000000000,                  0,                  0},
     {0.451685496026601, -1.706345585301695,  1.000000000000000,                  0},
     {0.566995242706580, -0.639830177853812,  0.379147889723543,  1.000000000000000}});

  tracking::math::DiagonalMatrix<double, 4> expD(
    {1.744764006421280e+00, 2.623463785299294e-01, 1.298863681833844e+00, 5.511441032938980e-02});

  tracking::math::Matrix<double, 4,1> w(
    {{3.829541723464068e-01}, {1.613328887439391e-01}, {9.379321662496187e-02}, {5.884958503513444e-01}});
  auto sigma = 3.0;
  // clang-format on

  // call UUT
  tracking::math::Rank1Update<double, 4>::run(L, D, -sigma, w);

  for (auto row = 0; row < 4; ++row)
  {
    EXPECT_FLOAT_EQ(D[row], expD[row]);
    for (auto col = 0; col < row; ++col)
    {
      EXPECT_FLOAT_EQ(L(row, col), expL(row, col));
    }
  }
}

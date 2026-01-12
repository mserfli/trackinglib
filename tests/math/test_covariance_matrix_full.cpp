#include "gtest/gtest.h"
#include "trackingLib/math/linalg/conversions/covariance_matrix_conversions.hpp"
#include "trackingLib/math/linalg/conversions/square_conversions.hpp"
#include "trackingLib/math/linalg/covariance_matrix_factored.hpp" // IWYU pragma: keep
#include "trackingLib/math/linalg/covariance_matrix_full.hpp"     // IWYU pragma: keep
#include "trackingLib/math/linalg/square_matrix.h"
#include <cmath>

// instatiate all templates for full coverage report
template class tracking::math::CovarianceMatrixFull<float32, 3>;
template class tracking::math::CovarianceMatrixFull<float64, 4>;
template class tracking::math::CovarianceMatrixFactored<float32, 3>;
template class tracking::math::CovarianceMatrixFactored<float64, 4>;

using namespace tracking::math;

// Helper function to create a symmetric positive definite matrix
template <typename FloatType, sint32 Size>
auto createSymmetricPositiveDefiniteMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  // Create a diagonal matrix with positive values and add small symmetric perturbations
  CovarianceMatrixFull<FloatType, Size> result{};
  for (sint32 i = 0; i < Size; ++i)
  {
    result.at_unsafe(i, i) = static_cast<FloatType>(1.0 + i * 0.5); // Diagonal dominance
    for (sint32 j = i + 1; j < Size; ++j)
    {
      FloatType val          = static_cast<FloatType>(0.1 * (i + 1) * (j + 1));
      result.at_unsafe(i, j) = val;
      result.at_unsafe(j, i) = val; // Ensure symmetry
    }
  }
  return result;
}

// Helper function to create an ill-conditioned matrix
template <typename FloatType, sint32 Size>
auto createFactoredIllConditionedMatrix() -> CovarianceMatrixFull<FloatType, Size>
{
  CovarianceMatrixFull<FloatType, Size> result{};

  // Create a matrix with a mix of very large and very small eigenvalues
  for (sint32 i = 0; i < Size; ++i)
  {
    for (sint32 j = 0; j < Size; ++j)
    {
      // Create a matrix that's close to singular
      FloatType val          = static_cast<FloatType>(1.0) + static_cast<FloatType>(0.001 * (i == j ? Size - i : i + j));
      result.at_unsafe(i, j) = val;
      if (i != j)
      {
        result.at_unsafe(j, i) = val; // Ensure symmetry
      }
    }
  }
  return result;
}

TEST(CovarianceMatrixFull, compose) // NOLINT
{
  // clang-format off
  const auto cov = CovarianceMatrixFull<float32, 3>::FromList({
    {45, 52, 12},
    {52, 66, 16},
    {12, 16,  4}
  });
  // clang-format on

  // call UUT
  auto composed = cov();

  EXPECT_EQ(cov._data, composed._data);
}

TEST(CovarianceMatrixFull, apaT) // NOLINT
{
  // clang-format off
  auto cov = CovarianceMatrixFull<float64,4>::FromList({
    { 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
    {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
    { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
    {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}
  });
  const auto A = conversions::SquareFromList<float64, 4, true>({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}
  });
    
  const auto expCov = CovarianceMatrixFull<float64,4>::FromList({
    { 2.752859732130674,  -1.285839545719224,   0.473376178291911,   4.416126653289711},
    {-1.285839545719224,   1.851835913837325,   0.811096749802654,  -2.464137199670983},
    { 0.473376178291911,   0.811096749802654,   0.967827910529903,   0.408774316031078},
    { 4.416126653289711,  -2.464137199670983,   0.408774316031078,   7.233518281911786}
  });
  // clang-format on

  // call UUT
  cov.apaT(A);

  // verify
  for (auto row = 0; row < 4; row++)
  {
    for (auto col = 0; col < 4; col++)
    {
      EXPECT_FLOAT_EQ(cov.at_unsafe(row, col), expCov.at_unsafe(row, col));
    }
  }
}

TEST(CovarianceMatrixFull, apaT_const) // NOLINT
{
  // clang-format off
  const auto cov = CovarianceMatrixFull<float64,4>::FromList({
    { 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
    {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
    { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
    {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}
  });
  const auto A = conversions::SquareFromList<float64, 4, true>({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}
  });
    
  const auto expCov = CovarianceMatrixFull<float64,4>::FromList({
    { 2.752859732130674,  -1.285839545719224,   0.473376178291911,   4.416126653289711},
    {-1.285839545719224,   1.851835913837325,   0.811096749802654,  -2.464137199670983},
    { 0.473376178291911,   0.811096749802654,   0.967827910529903,   0.408774316031078},
    { 4.416126653289711,  -2.464137199670983,   0.408774316031078,   7.233518281911786}
  });
  // clang-format on

  // call UUT
  const auto res = cov.apaT(A);

  // verify
  for (auto row = 0; row < 4; row++)
  {
    for (auto col = 0; col < 4; col++)
    {
      EXPECT_FLOAT_EQ(res.at_unsafe(row, col), expCov.at_unsafe(row, col));
    }
  }
}

TEST(CovarianceMatrixFull, apaT_equality)
{
  // clang-format off
  const auto cov = CovarianceMatrixFull<float64,4>::FromList({
    { 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
    {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
    { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
    {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}
  });
  const auto A = conversions::SquareFromList<float64, 4, true>({
    {6.240853754330984e-01,   3.581644763169872e-01,   5.385448223130755e-01,   6.122387805324014e-01},
    {9.666405567486658e-04,   9.261665823497265e-01,   5.262033279133882e-02,   4.022802305927367e-01},
    {4.208940256150708e-01,   9.768163860098072e-01,   3.480474524537769e-01,   9.884169020892678e-01},
    {9.687082960197513e-01,   1.310361955580941e-01,   4.530398432949093e-01,   5.183403919872129e-01}
  });
  // clang-format on

  // A*P*A' = inv(inv(A)'*inv(P)*inv(A))
  const auto invCov = cov.inverse().value();

  const SquareMatrix<float64, 4, false> invA = A.inverse();
  // calc apaT
  const auto apaT = cov.apaT(A);

  // calc inv(inv(A)'*inv(P)*inv(A))
  const auto apaT_ = invCov.apaT(SquareMatrix<float64, 4, true>{invA.transpose()}).inverse().value();

  // verify
  for (auto row = 0; row < 4; row++)
  {
    for (auto col = 0; col < 4; col++)
    {
      EXPECT_FLOAT_EQ(apaT.at_unsafe(row, col), apaT_.at_unsafe(row, col));
    }
  }
}

TEST(CovarianceMatrixFull, inverse_const) // NOLINT
{
  // clang-format off
  auto cov = CovarianceMatrixFull<float64,4>::FromList({
    { 7.095005365385164,  -2.002405585706988,   3.186228227810577,  -1.796274062161563},
    {-2.002405585706988,   1.993403311363277,  -1.262526372481200,   0.223417952190492},
    { 3.186228227810577,  -1.262526372481200,   1.638799986245814,  -0.810174937419070},
    {-1.796274062161563,   0.223417952190492,  -0.810174937419070,   0.821337656508534}
  });
  const auto expInv = CovarianceMatrixFull<float64,4>::FromList({
    { 1.443780121231659e+00, -5.460031722987547e-01, -3.109739702037713e+00,  2.386089616571337e-01},
    {-5.460031722987547e-01,  1.771103154474497e+00,  3.118011666625712e+00,  1.399749934294501e+00},
    {-3.109739702037713e+00,  3.118011666625712e+00,  1.029938326077342e+01,  2.510219303968749e+00},
    { 2.386089616571337e-01,  1.399749934294501e+00,  2.510219303968749e+00,  3.834713491421974e+00}
  });
  // clang-format on

  // call UUT
  const auto inv = cov.inverse().value();

  // verify
  for (auto row = 0; row < 4; row++)
  {
    for (auto col = 0; col < 4; col++)
    {
      EXPECT_FLOAT_EQ(inv.at_unsafe(row, col), expInv.at_unsafe(row, col));
    }
  }
}

TEST(CovarianceMatrixFull, setVariance) // NOLINT
{
  // clang-format off
  auto cov = CovarianceMatrixFull<float32, 3>::FromList({
    {45,  76,  72},
    {76, 132, 120},
    {72, 120, 144}
  });

  const auto expCov = CovarianceMatrixFull<float32, 3>::FromList({
    {4,  0,  0},
    {0, 132, 120},
    {0, 120, 144}
  });

  // call UUT
  cov.setVariance(0,4);

  // verify
  EXPECT_EQ(cov._data, expCov._data);
}

TEST(CovarianceMatrixFull, symmetry_preservation_apaT__Success) // NOLINT
{
  // Test that apaT operation preserves symmetry
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 4>();
  auto A = conversions::SquareFromList<float64, 4, true>({
    {0.9, 0.1, 0.2, 0.3},
    {0.1, 0.8, 0.1, 0.2},
    {0.2, 0.1, 0.7, 0.1},
    {0.3, 0.2, 0.1, 0.6}
  });

  // Verify initial symmetry
  EXPECT_TRUE(cov.isSymmetric());

  // Apply apaT operation
  cov.apaT(A);

  // Verify symmetry is preserved
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, symmetry_preservation_setVariance__Success) // NOLINT
{
  // Test that setVariance operation preserves symmetry
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // Verify initial symmetry
  EXPECT_TRUE(cov.isSymmetric());

  // Apply setVariance operation
  cov.setVariance(1, 5.0f);

  // Verify symmetry is preserved
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, positive_semi_definite_apaT__Success) // NOLINT
{
  // Test that apaT operation preserves positive semi-definiteness
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 3>();
  auto A = conversions::SquareFromList<float64, 3, true>({
    {0.9, 0.1, 0.2},
    {0.1, 0.8, 0.1},
    {0.2, 0.1, 0.7}
  });

  // Verify initial positive semi-definiteness
  EXPECT_TRUE(cov.isPositiveSemiDefinite());

  // Apply apaT operation
  cov.apaT(A);

  // Verify positive semi-definiteness is preserved
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, positive_semi_definite_inverse__Success) // NOLINT
{
  // Test that inverse operation preserves positive semi-definiteness
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Verify initial positive semi-definiteness
  EXPECT_TRUE(cov.isPositiveSemiDefinite());

  // Compute inverse
  auto invResult = cov.inverse();
  ASSERT_TRUE(invResult.has_value());

  // Verify positive semi-definiteness is preserved
  EXPECT_TRUE(invResult.value().isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, conversion_to_factored__Success) // NOLINT
{
  // Test conversion from full to factored form
  auto fullCov = createSymmetricPositiveDefiniteMatrix<float32, 3>();

  // Create nested initializer list from full matrix data for conversion
  std::initializer_list<std::initializer_list<float32>> covList = {
    {fullCov.at_unsafe(0, 0), fullCov.at_unsafe(0, 1), fullCov.at_unsafe(0, 2)},
    {fullCov.at_unsafe(1, 0), fullCov.at_unsafe(1, 1), fullCov.at_unsafe(1, 2)},
    {fullCov.at_unsafe(2, 0), fullCov.at_unsafe(2, 1), fullCov.at_unsafe(2, 2)}
  };

  // Convert to factored form using conversion function
  auto factoredCov = conversions::CovarianceMatrixFactoredFromList<float32, 3>(covList);

  // Convert back to full form
  auto convertedFull = factoredCov();

  // Verify equivalence (within numerical tolerance)
  for (sint32 i = 0; i < 3; ++i) {
    for (sint32 j = 0; j < 3; ++j) {
      EXPECT_NEAR(fullCov.at_unsafe(i, j), convertedFull.at_unsafe(i, j), 1e-5f);
    }
  }
}

TEST(CovarianceMatrixFull, conversion_roundtrip__Success) // NOLINT
{
  // Test roundtrip conversion preserves properties
  auto original = createSymmetricPositiveDefiniteMatrix<float64, 4>();

  // Verify original properties
  EXPECT_TRUE(original.isSymmetric());
  EXPECT_TRUE(original.isPositiveSemiDefinite());

  // Create nested initializer list from original matrix data for conversion
  std::initializer_list<std::initializer_list<float64>> covList = {
    {original.at_unsafe(0, 0), original.at_unsafe(0, 1), original.at_unsafe(0, 2), original.at_unsafe(0, 3)},
    {original.at_unsafe(1, 0), original.at_unsafe(1, 1), original.at_unsafe(1, 2), original.at_unsafe(1, 3)},
    {original.at_unsafe(2, 0), original.at_unsafe(2, 1), original.at_unsafe(2, 2), original.at_unsafe(2, 3)},
    {original.at_unsafe(3, 0), original.at_unsafe(3, 1), original.at_unsafe(3, 2), original.at_unsafe(3, 3)}
  };

  // Convert to factored and back
  auto factored = conversions::CovarianceMatrixFactoredFromList<float64, 4>(covList);
  auto roundtrip = factored();

  // Verify properties are preserved
  EXPECT_TRUE(roundtrip.isSymmetric());
  EXPECT_TRUE(roundtrip.isPositiveSemiDefinite());

  // Verify numerical equivalence
  for (sint32 i = 0; i < 4; ++i) {
    for (sint32 j = 0; j < 4; ++j) {
      EXPECT_NEAR(original.at_unsafe(i, j), roundtrip.at_unsafe(i, j), 1e-6);
    }
  }
}

TEST(CovarianceMatrixFull, large_matrix_apaT__Success) // NOLINT
{
  // Test with larger matrix size (6x6)
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 6>();
  auto A = SquareMatrix<float32, 6, true>::Identity();

  // Scale the identity matrix to create a non-trivial transformation
  for (sint32 i = 0; i < 6; ++i) {
    A.at_unsafe(i, i) = 0.9f + i * 0.02f;
  }

  // Verify initial properties
  EXPECT_TRUE(cov.isSymmetric());
  EXPECT_TRUE(cov.isPositiveSemiDefinite());

  // Apply apaT operation
  cov.apaT(A);

  // Verify properties are preserved
  EXPECT_TRUE(cov.isSymmetric());
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, numerical_stability_ill_conditioned__Success) // NOLINT
{
  // Test numerical stability with ill-conditioned matrix
  auto illCond = createFactoredIllConditionedMatrix<float64, 4>();
  auto A = conversions::SquareFromList<float64, 4, true>({
      {0.95, 0.01, 0.01, 0.01},
      {0.01, 0.95, 0.01, 0.01},
      {0.01, 0.01, 0.95, 0.01},
      {0.01, 0.01, 0.01, 0.95}
  });

  // Apply apaT operation
  illCond.apaT(A);

  // Verify symmetry is preserved even with ill-conditioned matrix
  EXPECT_TRUE(illCond.isSymmetric());
}

TEST(CovarianceMatrixFull, ctor_from_Matrix_const_float3__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto matrix = conversions::SquareFromList<float32, 3, true>({{4, 1, 2}, {1, 5, 3}, {2, 3, 6}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float32, 3>(matrix);
  
  // Verify data is copied correctly
  EXPECT_EQ(cov._data, matrix._data);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, ctor_from_Matrix_const_double4__Success) // NOLINT
{
  // Create a symmetric positive definite matrix
  auto matrix = conversions::SquareFromList<float64, 4, true>({{4, 1, 2, 1}, {1, 5, 3, 2}, {2, 3, 6, 3}, {1, 2, 3, 4}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float64, 4>(matrix);
  
  // Verify data is copied correctly
  EXPECT_EQ(cov._data, matrix._data);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, ctor_from_Matrix_const_symmetry__Success) // NOLINT
{
  // Create a symmetric matrix
  auto matrix = conversions::SquareFromList<float32, 3, true>({{2, 1, 1}, {1, 3, 1}, {1, 1, 4}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float32, 3>(matrix);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
  
  // Verify positive definiteness
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, ctor_from_Matrix_const_positive_definite__Success) // NOLINT
{
  // Create a known positive definite matrix
  auto matrix = conversions::SquareFromList<float64, 3, true>({{6, 2, 1}, {2, 5, 2}, {1, 2, 4}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float64, 3>(matrix);
  
  // Verify positive definiteness is maintained
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, ctor_from_SquareMatrix_const_float3__Success) // NOLINT
{
  // Create a symmetric positive definite square matrix
  auto squareMat = conversions::SquareFromList<float32, 3, true>({{4, 1, 2}, {1, 5, 3}, {2, 3, 6}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float32, 3>(squareMat);
  
  // Verify data is copied correctly
  EXPECT_EQ(cov._data, squareMat._data);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, ctor_from_SquareMatrix_const_double4__Success) // NOLINT
{
  // Create a symmetric positive definite square matrix
  auto squareMat = conversions::SquareFromList<float64, 4, true>({{4, 1, 2, 1}, {1, 5, 3, 2}, {2, 3, 6, 3}, {1, 2, 3, 4}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float64, 4>(squareMat);
  
  // Verify data is copied correctly
  EXPECT_EQ(cov._data, squareMat._data);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, ctor_from_SquareMatrix_const_symmetry__Success) // NOLINT
{
  // Create a symmetric square matrix
  auto squareMat = conversions::SquareFromList<float32, 3, true>({{3, 1, 1}, {1, 4, 1}, {1, 1, 5}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float32, 3>(squareMat);
  
  // Verify symmetry is maintained
  EXPECT_TRUE(cov.isSymmetric());
  
  // Verify positive definiteness
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, ctor_from_SquareMatrix_const_positive_definite__Success) // NOLINT
{
  // Create a known positive definite square matrix
  auto squareMat = conversions::SquareFromList<float64, 3, true>({{5, 1, 1}, {1, 6, 2}, {1, 2, 5}});
  
  // Test constructor
  auto cov = CovarianceMatrixFull<float64, 3>(squareMat);
  
  // Verify positive definiteness is maintained
  EXPECT_TRUE(cov.isPositiveSemiDefinite());
}

TEST(CovarianceMatrixFull, Identity_float3__Success) // NOLINT
{
  // Test Identity() method for float32, 3x3
  auto cov = CovarianceMatrixFull<float32, 3>::Identity();
  
  // Verify it's an identity matrix
  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
  
  // Verify symmetry
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, Identity_float4__Success) // NOLINT
{
  // Test Identity() method for float32, 4x4
  auto cov = CovarianceMatrixFull<float32, 4>::Identity();
  
  // Verify it's an identity matrix
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
  
  // Verify symmetry
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, Identity_float6__Success) // NOLINT
{
  // Test Identity() method for float32, 6x6
  auto cov = CovarianceMatrixFull<float32, 6>::Identity();
  
  // Verify it's an identity matrix
  for (sint32 i = 0; i < 6; ++i)
  {
    for (sint32 j = 0; j < 6; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
  
  // Verify symmetry
  EXPECT_TRUE(cov.isSymmetric());
}

TEST(CovarianceMatrixFull, operator_call_const_float3__Success) // NOLINT
{
  // Create a covariance matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();
  
  // Test operator()() const
  const auto& result = cov();
  
  // Verify it returns the same object
  EXPECT_EQ(&result, &cov);
  
  // Verify data is unchanged
  EXPECT_EQ(result._data, cov._data);
}

TEST(CovarianceMatrixFull, operator_call_const_double4__Success) // NOLINT
{
  // Create a covariance matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 4>();
  
  // Test operator()() const
  const auto& result = cov();
  
  // Verify it returns the same object
  EXPECT_EQ(&result, &cov);
  
  // Verify data is unchanged
  EXPECT_EQ(result._data, cov._data);
}

TEST(CovarianceMatrixFull, operator_call_const_identity__Success) // NOLINT
{
  // Test with identity matrix
  auto cov = CovarianceMatrixFull<float32, 3>::Identity();
  
  // Test operator()() const
  const auto& result = cov();
  
  // Verify it returns the same object
  EXPECT_EQ(&result, &cov);
  
  // Verify it's still an identity matrix
  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(result.at_unsafe(i, j), expected);
    }
  }
}

TEST(CovarianceMatrixFull, setIdentity_float3__Success) // NOLINT
{
  // Create a non-identity covariance matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 3>();
  
  // Apply setIdentity()
  cov.setIdentity();
  
  // Verify it's now an identity matrix
  for (sint32 i = 0; i < 3; ++i)
  {
    for (sint32 j = 0; j < 3; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
}

TEST(CovarianceMatrixFull, setIdentity_double4__Success) // NOLINT
{
  // Create a non-identity covariance matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float64, 4>();
  
  // Apply setIdentity()
  cov.setIdentity();
  
  // Verify it's now an identity matrix
  for (sint32 i = 0; i < 4; ++i)
  {
    for (sint32 j = 0; j < 4; ++j)
    {
      float64 expected = (i == j) ? 1.0 : 0.0;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
}

TEST(CovarianceMatrixFull, setIdentity_float6__Success) // NOLINT
{
  // Create a non-identity covariance matrix
  auto cov = createSymmetricPositiveDefiniteMatrix<float32, 6>();
  
  // Apply setIdentity()
  cov.setIdentity();
  
  // Verify it's now an identity matrix
  for (sint32 i = 0; i < 6; ++i)
  {
    for (sint32 j = 0; j < 6; ++j)
    {
      float32 expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(cov.at_unsafe(i, j), expected);
    }
  }
}

// Error Handling Tests for Covariance Matrix Functions Returning tl::unexpected

TEST(CovarianceMatrixFull, inverse_NotPositiveDefinite_ExpectError) // NOLINT
{
    // Create a non-positive definite matrix (negative eigenvalue)
    auto nonPosDef = CovarianceMatrixFull<float32, 3>::FromList({
        {2,  1,  1},
        {1,  2,  1},
        {1,  1, -1}}); // Negative diagonal element makes it non-positive definite

    // Test inverse() - should return error
    auto result = nonPosDef.inverse();
    EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFull, inverse_SingularMatrix_ExpectError) // NOLINT
{
    // Create a singular matrix (determinant = 0)
    auto singular = CovarianceMatrixFull<float64,3>::FromList({
        {1, 2, 3},
        {2, 4, 6},
        {3, 6, 0} // diagonal element zero makes it singular
      }); 

    // Test inverse() - should return error
    auto result = singular.inverse();
    EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFull, inverse_NonSymmetricMatrix_ExpectError) // NOLINT
{
    // Create a non-symmetric matrix
    auto nonSymmetric = createSymmetricPositiveDefiniteMatrix<float64, 3>();
    nonSymmetric.at_unsafe(0, 1) += 1.0; // Break symmetry
    
    // Test inverse() - should return error
    auto result = nonSymmetric.inverse();
    EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFull, inverse_ZeroMatrix_ExpectError) // NOLINT
{
    // Create a zero matrix
    auto zero = createSymmetricPositiveDefiniteMatrix<float64, 3>();
    zero.setZeros();
    
    // Test inverse() - should return error
    auto result = zero.inverse();
    EXPECT_FALSE(result.has_value());
}

TEST(CovarianceMatrixFull, inverse_NegativeDefiniteMatrix_ExpectError) // NOLINT
{
    // Create a negative definite matrix
    auto negDef = createSymmetricPositiveDefiniteMatrix<float64, 3>();
    negDef.at_unsafe(0, 0) = -1.0;
    negDef.at_unsafe(1, 1) = -2.0;
    negDef.at_unsafe(2, 2) = -3.0; // All negative eigenvalues

    // Test inverse() - should return error
    auto result = negDef.inverse();
    EXPECT_FALSE(result.has_value());
}

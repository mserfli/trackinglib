#include <gtest/gtest.h>
#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"    // for operator<<
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <cmath>                                  // for std::isfinite
#include <type_traits>                            // for std::is_same_v

using Testvalue_type = float32;

// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
template class tracking::motion::MotionModelCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;

template <typename CovarianceMatrixPolicy_, template <typename CovarianceMatrixPolicy> class FilterType>
struct TestPredictCV
{
  using MM             = tracking::motion::MotionModelCV<CovarianceMatrixPolicy_>;
  using EgoMotionInst  = tracking::env::EgoMotion<CovarianceMatrixPolicy_>;
  using FilterTypeInst = FilterType<CovarianceMatrixPolicy_>;
  using value_type     = typename CovarianceMatrixPolicy_::value_type;

  static void init(typename MM::StateVec& vec,
                   typename MM::StateVec& expVec,
                   typename MM::StateCov& cov,
                   typename MM::StateCov& expCov,
                   const FilterTypeInst& /*filter*/)
  {
    if constexpr (std::is_same_v<FilterTypeInst, tracking::filter::InformationFilter<CovarianceMatrixPolicy_>>)
    {
      // InformationFilter behavior: transform state and covariance to information space for later comparison
      // Transform covariance to information space
      cov    = cov.inverse().value();
      expCov = expCov.inverse().value();

      // Transform state vector to information space
      vec    = static_cast<typename MM::StateVec>(cov() * vec); // y = Y * x
      expVec = static_cast<typename MM::StateVec>(expCov() * expVec);
    }
    else
    {
      // KalmanFilter behavior: do nothing (identity)
      // No change required
    }
  }

  static auto tolerance(const FilterTypeInst& /*filter*/) -> value_type
  {
    if constexpr (std::is_same_v<FilterTypeInst, tracking::filter::InformationFilter<CovarianceMatrixPolicy_>>)
    {
      return static_cast<value_type>(3e-6);
    }
    else
    {
      return static_cast<value_type>(1.1e-6);
    }
  }

  static void run_without_ego_motion_compensation()
  {
    const int      steps     = 5;
    const auto     dt        = static_cast<value_type>(0.1);
    const auto     motion    = typename EgoMotionInst::InertialMotion{};
    const auto     geometry  = typename EgoMotionInst::Geometry{};
    const auto     egoMotion = EgoMotionInst(motion, geometry, dt);
    FilterTypeInst filter{};

    // clang-format off
    auto vec = MM::StateVecFromList({10, 2, 0, 0});
    auto cov = MM::StateCovFromList({
      {5, 0, 0, 0.0},
      {0, 1, 0, 0.0},
      {0, 0, 1, 0.0},
      {0, 0, 0, 0.1}
    });
    auto expVec = MM::StateVecFromList({11, 2, 0, 0});
    auto expCov = MM::StateCovFromList({
      {+5.29125, +0.62500, +0.00000, +0.00000},
      {+0.62500, +1.50000, +0.00000, +0.00000},
      {+0.00000, +0.00000, +1.06625, +0.17500},
      {+0.00000, +0.00000, +0.17500, +0.60000}
    });
    // clang-format on

    init(vec, expVec, cov, expCov, filter);
    const auto tol = tolerance(filter);

    // instantiate MM with mocked EgoMotion compensation
    testing::NiceMock<test::MotionModelNoEgoMotionMock<MM>> mm{vec, cov};
    mm.delegate();
    EXPECT_CALL(mm, computeEgoMotionCompensationMatrices).Times(steps);

    // call UUT
    for (auto i = 0; i < steps; ++i)
    {
      mm.predict(dt, filter, egoMotion);
    }

    for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    {
      EXPECT_NEAR(mm._vec.at_unsafe(row), expVec.at_unsafe(row), tol);
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_NEAR(mm._cov.at_unsafe(row, col), expCov.at_unsafe(row, col), tol);
      }
    }
  }

  static void run_with_ego_motion_compensation()
  {
    const int  steps = 5;
    const auto dt    = static_cast<value_type>(0.1);
    // clang-format off
    const auto motion   = typename EgoMotionInst::InertialMotion{
        .v = 2.0, .a = 1.0, .w = 0.1, 
        .sv = 0.1, .sa = 0.1, .sw = 0.02};
    const auto geometry = typename EgoMotionInst::Geometry{
        .width = 1.8, .length = 4.5, .height = 1.5, 
        .distCog2Ego = 1.0, .distFrontAxle2Ego = 2.5, .distFrontAxle2RearAxle = 2.5};
    const auto     egoMotion = EgoMotionInst(motion, geometry, dt);
    // clang-format on
    FilterTypeInst filter{};

    // clang-format off
    auto vec = MM::StateVecFromList({10, 2, 0, 0});
    auto cov = MM::StateCovFromList({
      {5, 0, 0, 0.0},
      {0, 1, 0, 0.0},
      {0, 0, 1, 0.0},
      {0, 0, 0, 0.1}
    });
    auto expVec = MM::StateVecFromList({+9.9604310989, +1.9975004196, -0.5741304159, -0.0999583453});
    auto expCov = MM::StateCovFromList({
      {+5.2812027931,  +0.6238771081,  -0.2107825130,  -0.0224393737},
      {+0.6238771081,  +1.4977520704,  -0.0224399883,  -0.0449210368},
      {-0.2107825130,  -0.0224399883,  +1.0793461800,  +0.1765742004},
      {-0.0224393737,  -0.0449210368,  +0.1765742004,  +0.6023279428}
    });
    // clang-format on

    init(vec, expVec, cov, expCov, filter);
    auto tol{MM::StateMatrix::Ones()};
    tol *= tolerance(filter);
    if constexpr (std::is_same_v<FilterTypeInst,
                                 tracking::filter::InformationFilter<
                                     tracking::math::FullCovarianceMatrixPolicy<typename CovarianceMatrixPolicy_::value_type>>>)
    {
      // Woodbury-via-QR reformulation introduces slightly more floating-point noise than the
      // UDU one-step path; needs a small uniform bump over the shared tolerance() baseline.
      tol *= 2;
    }

    // instantiate regular MM (no mocking)
    MM mm{vec, cov};

    // call UUT
    for (auto i = 0; i < steps; ++i)
    {
      mm.predict(dt, filter, egoMotion);
    }

    for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    {
      EXPECT_NEAR(mm._vec.at_unsafe(row), expVec.at_unsafe(row), tol.at_unsafe(2, 2));
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_NEAR(mm._cov().at_unsafe(row, col), expCov.at_unsafe(row, col), tol.at_unsafe(row, col));
      }
    }
  }
};

TEST(MotionModelCV, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_kalmanFilter_egoMotion) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_kalmanFilter_egoMotion) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_informationFilter_egoMotion) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_informationFilter_egoMotion) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_informationFilter_singularPe_updatesY) // NOLINT
{
  // Reproduces the InformationFilter/full-covariance ego-motion-compensation bug: with
  // w == 0, calcLinearMotionJacobian() produces a J whose rows 1/2 (dy, dpsi) are scalar
  // multiples of each other, so Pe's bottom-right 2x2 block is an exact rank-1 outer
  // product; with dt/v/sw chosen so the block's entries are exactly representable
  // (0.25/0.25/0.25), decomposeLLT()'s Cholesky pivot at (2,2) becomes exactly 0.0,
  // reproducing the silent-NaN inverse() described in the bug report (verified standalone:
  // Pe.inverse() currently reports has_value()==true with an all-NaN/Inf payload). The
  // full-covariance branch must still update Y with a finite result instead of silently
  // leaving it stale or NaN-poisoned.
  using CovPolicy      = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
  using MM             = tracking::motion::MotionModelCV<CovPolicy>;
  using EgoMotionInst  = tracking::env::EgoMotion<CovPolicy>;
  using FilterTypeInst = tracking::filter::InformationFilter<CovPolicy>;

  const auto dt = static_cast<Testvalue_type>(0.5);
  // clang-format off
  const auto motion   = typename EgoMotionInst::InertialMotion{
      .v = 4.0, .a = 0.0, .w = 0.0,
      .sv = 0.1, .sa = 0.1, .sw = 1.0};
  const auto geometry = typename EgoMotionInst::Geometry{
      .width = 1.8, .length = 4.5, .height = 1.5,
      .distCog2Ego = 1.0, .distFrontAxle2Ego = 2.5, .distFrontAxle2RearAxle = 2.5};
  const auto egoMotion = EgoMotionInst(motion, geometry, dt);
  // clang-format on

  ASSERT_FALSE(egoMotion.getDisplacementCog().vec.isZeros());
  // Sanity-check Pe is (numerically) singular by hand, independent of decomposeLLT()/inverse()'s
  // own correctness (that's covered separately by the decomposeLLT/CovarianceMatrixFull tests) --
  // a 3x3 determinant computed directly from Pe's entries.
  const auto& pe  = egoMotion.getDisplacementCog().cov;
  const auto  det = pe.at_unsafe(0, 0) * (pe.at_unsafe(1, 1) * pe.at_unsafe(2, 2) - pe.at_unsafe(1, 2) * pe.at_unsafe(2, 1)) -
                    pe.at_unsafe(0, 1) * (pe.at_unsafe(1, 0) * pe.at_unsafe(2, 2) - pe.at_unsafe(1, 2) * pe.at_unsafe(2, 0)) +
                    pe.at_unsafe(0, 2) * (pe.at_unsafe(1, 0) * pe.at_unsafe(2, 1) - pe.at_unsafe(1, 1) * pe.at_unsafe(2, 0));
  ASSERT_NEAR(det, 0.0, 1e-6);

  FilterTypeInst filter{};

  // clang-format off
  auto vec = MM::StateVecFromList({10, 2, 0, 0});
  auto cov = MM::StateCovFromList({
    {5, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 0.1}
  });
  // clang-format on

  // InformationFilter convention: transform state/covariance into information space
  cov = cov.inverse().value();
  vec = static_cast<typename MM::StateVec>(cov() * vec);

  MM        mm{vec, cov};
  const auto YBefore = mm._cov;

  // call UUT
  mm.predict(dt, filter, egoMotion);

  auto changed  = false;
  auto allFinite = true;
  for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
  {
    for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
    {
      if (std::abs(mm._cov.at_unsafe(row, col) - YBefore.at_unsafe(row, col)) > static_cast<Testvalue_type>(1e-9))
      {
        changed = true;
      }
      if (!std::isfinite(mm._cov.at_unsafe(row, col)))
      {
        allFinite = false;
      }
    }
  }
  EXPECT_TRUE(changed) << "Y is stale after predict() despite non-zero ego motion with a singular Pe";
  EXPECT_TRUE(allFinite) << "Y is NaN/Inf-poisoned after predict() due to the singular Pe inversion";
}

TEST(MotionModelCV, predict_informationFilter_singularPe_fullMatchesFactored) // NOLINT
{
  // Regression test: Full and Factored/UDU InformationFilter must stay in agreement across
  // several prediction steps of non-zero ego motion with an exactly singular Pe.
  using CovPolicyFull     = tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>;
  using CovPolicyFactored = tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>;
  using MMFull            = tracking::motion::MotionModelCV<CovPolicyFull>;
  using MMFactored        = tracking::motion::MotionModelCV<CovPolicyFactored>;
  using EgoMotionInst     = tracking::env::EgoMotion<CovPolicyFull>;

  const auto dt = static_cast<Testvalue_type>(0.5);
  // clang-format off
  const auto motion   = typename EgoMotionInst::InertialMotion{
      .v = 4.0, .a = 0.0, .w = 0.0,
      .sv = 0.1, .sa = 0.1, .sw = 1.0};
  const auto geometry = typename EgoMotionInst::Geometry{
      .width = 1.8, .length = 4.5, .height = 1.5,
      .distCog2Ego = 1.0, .distFrontAxle2Ego = 2.5, .distFrontAxle2RearAxle = 2.5};
  const auto egoMotionFull = EgoMotionInst(motion, geometry, dt);

  using EgoMotionInstFactored = tracking::env::EgoMotion<CovPolicyFactored>;
  const auto motionFactored   = typename EgoMotionInstFactored::InertialMotion{
      .v = 4.0, .a = 0.0, .w = 0.0,
      .sv = 0.1, .sa = 0.1, .sw = 1.0};
  const auto geometryFactored = typename EgoMotionInstFactored::Geometry{
      .width = 1.8, .length = 4.5, .height = 1.5,
      .distCog2Ego = 1.0, .distFrontAxle2Ego = 2.5, .distFrontAxle2RearAxle = 2.5};
  const auto egoMotionFactored = EgoMotionInstFactored(motionFactored, geometryFactored, dt);
  // clang-format on

  tracking::filter::InformationFilter<CovPolicyFull>     filterFull{};
  tracking::filter::InformationFilter<CovPolicyFactored> filterFactored{};

  // clang-format off
  auto vecFull = MMFull::StateVecFromList({10, 2, 0, 0});
  auto covFull = MMFull::StateCovFromList({
    {5, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 0.1}
  });
  // clang-format on
  auto vecFactored = vecFull;
  auto covFactored = MMFactored::StateCovFromList({
      {5, 0, 0, 0},
      {0, 1, 0, 0},
      {0, 0, 1, 0},
      {0, 0, 0, 0.1},
  });

  covFull     = covFull.inverse().value();
  vecFull     = static_cast<typename MMFull::StateVec>(covFull() * vecFull);
  covFactored = covFactored.inverse().value();
  vecFactored = static_cast<typename MMFactored::StateVec>(covFactored() * vecFactored);

  MMFull     mmFull{vecFull, covFull};
  MMFactored mmFactored{vecFactored, covFactored};

  const auto steps = 5;
  const auto tol    = static_cast<Testvalue_type>(4.5e-4);
  for (auto i = 0; i < steps; ++i)
  {
    mmFull.predict(dt, filterFull, egoMotionFull);
    mmFactored.predict(dt, filterFactored, egoMotionFactored);
  }

  for (auto row = 0; row < MMFull::NUM_STATE_VARIABLES; ++row)
  {
    for (auto col = 0; col < MMFull::NUM_STATE_VARIABLES; ++col)
    {
      EXPECT_NEAR(mmFull._cov.at_unsafe(row, col), mmFactored._cov().at_unsafe(row, col), tol);
    }
  }
}

TEST(MotionModelCV, convertCA_fullCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
  auto vec = MMCA::StateVecFromList({10, 2, 0, 0, 2, 0.1});
  auto cov = MMCA::StateCovFromList({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  MMCA mm_ca{vec, cov};
  MMCV mm_cv{};

  // call UUT
  mm_cv.convertFrom(mm_ca);
  
  // verify
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::X),  mm_cv._vec.at_unsafe(MMCV::X));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::VX), mm_cv._vec.at_unsafe(MMCV::VX));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::Y),  mm_cv._vec.at_unsafe(MMCV::Y));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::VY), mm_cv._vec.at_unsafe(MMCV::VY));
  const auto& caFull = mm_ca._cov;
  const auto& cvFull = mm_cv._cov;
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::X),  cvFull.at_unsafe(MMCV::X,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::VX), cvFull.at_unsafe(MMCV::X,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::Y),  cvFull.at_unsafe(MMCV::X,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::VY), cvFull.at_unsafe(MMCV::X,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::X),  cvFull.at_unsafe(MMCV::VX, MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::VX), cvFull.at_unsafe(MMCV::VX, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::Y),  cvFull.at_unsafe(MMCV::VX, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::VY), cvFull.at_unsafe(MMCV::VX, MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::X),  cvFull.at_unsafe(MMCV::Y,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::VX), cvFull.at_unsafe(MMCV::Y,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::Y),  cvFull.at_unsafe(MMCV::Y,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::VY), cvFull.at_unsafe(MMCV::Y,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::X),  cvFull.at_unsafe(MMCV::VY, MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::VX), cvFull.at_unsafe(MMCV::VY, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::Y),  cvFull.at_unsafe(MMCV::VY, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::VY), cvFull.at_unsafe(MMCV::VY, MMCV::VY));
  // clang-format on
}

TEST(MotionModelCV, convertCA_facCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;
  auto vec = MMCA::StateVecFromList({10, 2, 0, 0, 2, 0.1});
  auto cov = MMCA::StateCovFromList({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  auto mm_ca = MMCA{vec, cov};
  MMCV mm_cv{};

  // call UUT
  mm_cv.convertFrom(mm_ca);

  // verify
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::X),  mm_cv._vec.at_unsafe(MMCV::X));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::VX), mm_cv._vec.at_unsafe(MMCV::VX));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::Y),  mm_cv._vec.at_unsafe(MMCV::Y));
  EXPECT_FLOAT_EQ(mm_ca._vec.at_unsafe(MMCA::VY), mm_cv._vec.at_unsafe(MMCV::VY));
  const auto caFull = mm_ca._cov();
  const auto cvFull = mm_cv._cov();
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::X),  cvFull.at_unsafe(MMCV::X,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::VX), cvFull.at_unsafe(MMCV::X,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::Y),  cvFull.at_unsafe(MMCV::X,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::X,  MMCA::VY), cvFull.at_unsafe(MMCV::X,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::X),  cvFull.at_unsafe(MMCV::VX, MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::VX), cvFull.at_unsafe(MMCV::VX, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::Y),  cvFull.at_unsafe(MMCV::VX, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VX, MMCA::VY), cvFull.at_unsafe(MMCV::VX, MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::X),  cvFull.at_unsafe(MMCV::Y,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::VX), cvFull.at_unsafe(MMCV::Y,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::Y),  cvFull.at_unsafe(MMCV::Y,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::Y,  MMCA::VY), cvFull.at_unsafe(MMCV::Y,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::X),  cvFull.at_unsafe(MMCV::VY, MMCV::X));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::VX), cvFull.at_unsafe(MMCV::VY, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::Y),  cvFull.at_unsafe(MMCV::VY, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::VY, MMCA::VY), cvFull.at_unsafe(MMCV::VY, MMCV::VY));
  // clang-format on
}

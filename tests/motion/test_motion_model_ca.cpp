#include <gtest/gtest.h>
#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"   // IWYU pragma: keep
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <type_traits>                            // for std::is_same_v

using Testvalue_type = float32;

// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
template class tracking::motion::MotionModelCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;

template <typename CovarianceMatrixPolicy_, template <typename CovarianceMatrixPolicy> class FilterType>
struct TestPredictCA
{
  using MM             = tracking::motion::MotionModelCA<CovarianceMatrixPolicy_>;
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
      return static_cast<value_type>(4e-6);
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
    auto vec = MM::StateVecFromList({
      10, 2, 2, 0.0125, 0.05, 0.1
    });
    auto cov = MM::StateCovFromList({
      {5, 0, 0, 0, 0.0, 0},
      {0, 1, 0, 0, 0.0, 0},
      {0, 0, 1, 0, 0.0, 0},
      {0, 0, 0, 1, 0.0, 0},
      {0, 0, 0, 0, 0.1, 0},
      {0, 0, 0, 0, 0.0, 1}
    });

    auto expVec = MM::StateVecFromList({
      11.25, 3, 2, 0.05, 0.1, 0.1
    });
    auto expCov = MM::StateCovFromList({
      {5.29010, 0.67500, 0.40000,       0,       0,       0},
      {0.67500, 1.80000, 2.00000,       0,       0,       0},
      {0.40000, 2.00000, 6.00000,       0,       0,       0},
      {      0,       0,       0, 1.06510, 0.22500, 0.40000},
      {      0,       0,       0, 0.22500, 0.90000, 2.00000},
      {      0,       0,       0, 0.40000, 2.00000, 6.00000},
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
      mm.predict(static_cast<value_type>(0.1), filter, egoMotion);
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
    auto vec = MM::StateVecFromList({
      10, 2, 2, 0.0125, 0.05, 0.1
    });
    auto cov = MM::StateCovFromList({
      {5, 0, 0, 0, 0.0, 0},
      {0, 1, 0, 0, 0.0, 0},
      {0, 0, 1, 0, 0.0, 0},
      {0, 0, 0, 1, 0.0, 0},
      {0, 0, 0, 0, 0.1, 0},
      {0, 0, 0, 0, 0.0, 1}
    });
    auto expVec = MM::StateVecFromList({
      10.2126188278, 3.0012483597, 2.0024981499, -0.5366877317, -0.0500624813, -8.3314255e-05
    });
    auto expCov = MM::StateCovFromList({
      {+5.2800531387,    +0.6738764048,    +0.4000000358,    -0.2107882202,    -0.0224299952,    +0.0000216986},
      {+0.6738764048,    +1.7977516651,    +2.0000000000,    -0.0224509798,    -0.0449220277,    +0.0000020047},
      {+0.4000000358,    +2.0000000000,    +6.0000000000,    +0.0000000194,    +0.0000000038,    -0.0000000031},
      {-0.2107882202,    -0.0224509798,    +0.0000000194,    +1.0783109665,    +0.2268155664,    +0.4004613757},
      {-0.0224299952,    -0.0449220277,    +0.0000000038,    +0.2268155664,    +0.9024284482,    +2.0001204014},
      {+0.0000216986,    +0.0000020047,    -0.0000000031,    +0.4004613757,    +2.0001204014,    +6.0000801086},
    });
    // clang-format on

    init(vec, expVec, cov, expCov, filter);
    auto tol{MM::StateMatrix::Ones()};
    if constexpr (std::is_same_v<FilterTypeInst,
                                 tracking::filter::InformationFilter<
                                     tracking::math::FullCovarianceMatrixPolicy<typename CovarianceMatrixPolicy_::value_type>>>)
    {
      // InformationFilter with full covariance does a two step approach to apply egomotion compensation
      // causing larger differences compared to the UDU one-step approach due to correlations being ignored
      tol *= 4.5e-4;
      tol.at_unsafe(3, 3) = 0.002;
      tol.at_unsafe(3, 4) = 0.0012;
      tol.at_unsafe(4, 3) = tol.at_unsafe(3, 4);
      tol.at_unsafe(4, 4) = 0.00063;
    }
    else
    {
      tol *= tolerance(filter);
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
      EXPECT_NEAR(mm._vec.at_unsafe(row), expVec.at_unsafe(row), tol.at_unsafe(0, 0));
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_NEAR(mm._cov().at_unsafe(row, col), expCov.at_unsafe(row, col), tol.at_unsafe(row, col));
      }
    }
  }
};

TEST(MotionModelCA, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_fullCov_kalmanFilter_egoMotion) // NOLINT
{
  TestPredictCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_kalmanFilter_egoMotion) // NOLINT
{
  TestPredictCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::KalmanFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCA, predict_fullCov_informationFilter_egoMotion) // NOLINT
{
  TestPredictCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_informationFilter_egoMotion) // NOLINT
{
  TestPredictCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>,
                tracking::filter::InformationFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCA, convertCV_fullCov) // NOLINT
{
  // clang-format off
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<Testvalue_type>>;
  auto vec = MMCV::StateVecFromList({10, 2, 0, 2});
  auto cov = MMCV::StateCovFromList({
    {10.9911,   -3.3077,    5.0849,   -0.4707},
    {-3.3077,   13.7164,   -1.1132,    0.3277},
    { 5.0849,   -1.1132,    2.6187,   -0.1260},
    {-0.4707,    0.3277,   -0.1260,    1.2990},
  });
  MMCV mm_cv{vec, cov};
  MMCA mm_ca{};

  // call UUT
  mm_ca.convertFrom(mm_cv);
  
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
  for(sint32 i=0;i<MMCA::NUM_STATE_VARIABLES;++i)
  {
    if(i!=MMCA::AX)
    {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AX, i), 0.0);
      EXPECT_FLOAT_EQ(caFull.at_unsafe(i, MMCA::AX), 0.0);
    }
    else {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AX, i), 1.0);
    }

    if(i!=MMCA::AY)
    {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AY, i), 0.0);
      EXPECT_FLOAT_EQ(caFull.at_unsafe(i, MMCA::AY), 0.0);
    }
    else {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AY, i), 1.0);
    }
  }
  // clang-format on}
}

TEST(MotionModelCA, convertCV_facCov) // NOLINT
{
  // clang-format off
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FactoredCovarianceMatrixPolicy<Testvalue_type>>;
  auto vec = MMCV::StateVecFromList({10, 2, 0, 2});
  auto cov = MMCV::StateCovFromList({
    {10.9911,   -3.3077,    5.0849,   -0.4707},
    {-3.3077,   13.7164,   -1.1132,    0.3277},
    { 5.0849,   -1.1132,    2.6187,   -0.1260},
    {-0.4707,    0.3277,   -0.1260,    1.2990},
  });
  MMCV mm_cv{vec, cov};
  MMCA mm_ca{};

  // call UUT
  mm_ca.convertFrom(mm_cv);
  
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
  for(sint32 i=0;i<MMCA::NUM_STATE_VARIABLES;++i)
  {
    if(i!=MMCA::AX)
    {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AX, i), 0.0);
      EXPECT_FLOAT_EQ(caFull.at_unsafe(i, MMCA::AX), 0.0);
    }
    else {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AX, i), 1.0);
    }

    if(i!=MMCA::AY)
    {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AY, i), 0.0);
      EXPECT_FLOAT_EQ(caFull.at_unsafe(i, MMCA::AY), 0.0);
    }
    else {
      EXPECT_FLOAT_EQ(caFull.at_unsafe(MMCA::AY, i), 1.0);
    }
  }
  // clang-format on
}

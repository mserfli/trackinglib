#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <type_traits>                            // for std::is_same_v

using TestFloatType = float32;

// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, TestFloatType>;
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, TestFloatType>;

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          template <typename FloatType>
          class FilterType,
          typename FloatType>
struct TestPredictCA
{
  using MM             = tracking::motion::MotionModelCA<CovarianceMatrixType, FloatType>;
  using EgoMotionInst  = typename MM::EgoMotion;
  using FilterTypeInst = FilterType<FloatType>;

  static void init(typename MM::StateCov& cov, typename MM::StateCov& expCov, const FilterTypeInst& /*filter*/)
  {
    if constexpr (std::is_same_v<FilterTypeInst, tracking::filter::InformationFilter<FloatType>>)
    {
      // InformationFilter behavior: invert covariance matrices
      cov    = cov.inverse().value();
      expCov = expCov.inverse().value();
    }
    else
    {
      // KalmanFilter behavior: do nothing (identity)
      // No change required
    }
  }

  static auto tolerance(const FilterTypeInst& /*filter*/) -> FloatType
  {
    if constexpr (std::is_same_v<FilterTypeInst, tracking::filter::InformationFilter<FloatType>>)
    {
      return static_cast<FloatType>(1e-3);
    }
    else
    {
      return static_cast<FloatType>(1e-6);
    }
  }

  static void run_without_ego_motion_compensation()
  {
    const int      steps     = 5;
    const auto     dt        = static_cast<FloatType>(0.1);
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

    init(cov, expCov, filter);
    const auto tol = tolerance(filter);

    // instantiate MM with mocked EgoMotion compensation
    testing::NiceMock<test::MotionModelNoEgoMotionMock<MM>> mm{vec, cov};
    mm.delegate();
    EXPECT_CALL(mm, compensateEgoMotion).Times(steps);

    // call UUT
    for (auto i = 0; i < steps; ++i)
    {
      mm.predict(static_cast<FloatType>(0.1), filter, egoMotion);
    }

    for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    {
      EXPECT_FLOAT_EQ(mm._vec.at_unsafe(row), expVec.at_unsafe(row));
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_NEAR(mm._cov.at_unsafe(row, col), expCov.at_unsafe(row, col), tol);
      }
    }
  }
};

TEST(MotionModelCA, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFull, tracking::filter::KalmanFilter, TestFloatType>::
      run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFactored, tracking::filter::KalmanFilter, TestFloatType>::
      run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFull, tracking::filter::InformationFilter, TestFloatType>::
      run_without_ego_motion_compensation();
}

TEST(MotionModelCA, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFactored, tracking::filter::InformationFilter, TestFloatType>::
      run_without_ego_motion_compensation();
}

TEST(MotionModelCA, convertCV_fullCov) // NOLINT
{
  // clang-format off
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, TestFloatType>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, TestFloatType>;
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
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, TestFloatType>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, TestFloatType>;
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

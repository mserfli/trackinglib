#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"   // IWYU pragma: keep
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <type_traits>                            // for std::is_same_v

using TestFloatType = float32;

// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, TestFloatType>;
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, TestFloatType>;

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          template <typename FloatType>
          class FilterType,
          typename FloatType>
struct TestPredictCV
{
  using MM = tracking::motion::MotionModelCV<CovarianceMatrixType, FloatType>;

  template <typename FilterType_>
  static void init(typename MM::StateCov& cov, typename MM::StateCov& expCov, const FilterType_& /*filter*/)
  {
    if constexpr (std::is_same_v<FilterType_, tracking::filter::InformationFilter<FloatType>>)
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

  template <typename FilterType_>
  static auto tolerance(const FilterType_& /*filter*/) -> FloatType
  {
    return static_cast<FloatType>(1e-6);
  }

  static void run()
  {
    tracking::env::EgoMotion<FloatType> egoMotion{};
    FilterType<FloatType>               filter{};

    // clang-format off
    const int steps = 3;
    auto vec = MM::StateVecFromList({10, 2, 0, 0});
    auto cov = MM::StateCovFromList({
      {5, 0, 0, 0.0},
      {0, 1, 0, 0.0},
      {0, 0, 1, 0.0},
      {0, 0, 0, 0.1}
    });
    auto expVec = MM::StateVecFromList({10.6, 2, 0, 0});
    auto expCov = MM::StateCovFromList({
      {+5.09875, +0.34500, +0.00000, +0.00000},
      {+0.34500, +1.30000, +0.00000, +0.00000},
      {+0.00000, +0.00000, +1.01775, +0.07500},
      {+0.00000, +0.00000, +0.07500, +0.40000}
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

TEST(MotionModelCV, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFull, tracking::filter::KalmanFilter, TestFloatType>::run();
}

TEST(MotionModelCV, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFactored, tracking::filter::KalmanFilter, TestFloatType>::run();
}

TEST(MotionModelCV, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFull, tracking::filter::InformationFilter, TestFloatType>::run();
}

TEST(MotionModelCV, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFactored, tracking::filter::InformationFilter, TestFloatType>::run();
}

TEST(MotionModelCV, convertCA_fullCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, TestFloatType>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, TestFloatType>;
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
  // clang-format on}
}

TEST(MotionModelCV, convertCA_facCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, TestFloatType>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, TestFloatType>;
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

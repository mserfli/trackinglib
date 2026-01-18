#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"   // IWYU pragma: keep
#include "trackingLib/math/linalg/matrix_io.h"    // for operator<<
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep
#include "trackingLib/motion/motion_model_cv.hpp" // IWYU pragma: keep
#include <type_traits>                            // for std::is_same_v

using TestFloatType = float32;

// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>>;
template class tracking::motion::MotionModelCV<tracking::math::FactoredCovarianceMatrixPolicy<TestFloatType>>;

template <typename CovarianceMatrixPolicy_, template <typename CovarianceMatrixPolicy> class FilterType>
struct TestPredictCV
{
  using MM             = tracking::motion::MotionModelCV<CovarianceMatrixPolicy_>;
  using EgoMotionInst  = tracking::env::EgoMotion<CovarianceMatrixPolicy_>;
  using FilterTypeInst = FilterType<CovarianceMatrixPolicy_>;
  using FloatType      = typename CovarianceMatrixPolicy_::FloatType;

  static void init(typename MM::StateCov& cov, typename MM::StateCov& expCov, const FilterTypeInst& /*filter*/)
  {
    if constexpr (std::is_same_v<FilterTypeInst, tracking::filter::InformationFilter<CovarianceMatrixPolicy_>>)
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

  static auto tolerance(const FilterTypeInst& /*filter*/) -> FloatType { return static_cast<FloatType>(1e-6); }

  static void run_without_ego_motion_compensation()
  {
    const int      steps     = 5;
    const auto     dt        = static_cast<FloatType>(0.1);
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

    init(cov, expCov, filter);
    const auto tol = tolerance(filter);

    // instantiate MM with mocked EgoMotion compensation
    testing::NiceMock<test::MotionModelNoEgoMotionMock<MM>> mm{vec, cov};
    mm.delegate();
    EXPECT_CALL(mm, compensateEgoMotion).Times(steps);

    // call UUT
    for (auto i = 0; i < steps; ++i)
    {
      mm.predict(dt, filter, egoMotion);
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

  static void run_with_ego_motion_compensation()
  {
    const int  steps = 5;
    const auto dt    = static_cast<FloatType>(0.1);
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
    auto expVec = MM::StateVecFromList({11, 2, 0, 0});
    auto expCov = MM::StateCovFromList({
      {+5.29125, +0.62500, +0.00000, +0.00000},
      {+0.62500, +1.50000, +0.00000, +0.00000},
      {+0.00000, +0.00000, +1.06625, +0.17500},
      {+0.00000, +0.00000, +0.17500, +0.60000}
    });
    // clang-format on

    init(cov, expCov, filter);
    const auto tol = tolerance(filter);

    // instantiate regular MM (no mocking)
    MM mm{vec, cov};

    // call UUT
    for (auto i = 0; i < steps; ++i)
    {
      mm.predict(dt, filter, egoMotion);
    }

    std::cout << mm._vec << std::endl;
    std::cout << mm._cov << std::endl;
    (void)tol;
    // for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    // {
    //   EXPECT_FLOAT_EQ(mm._vec.at_unsafe(row), expVec.at_unsafe(row));
    //   for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
    //   {
    //     EXPECT_NEAR(mm._cov.at_unsafe(row, col), expCov.at_unsafe(row, col), tol);
    //   }
    // }
  }
};

TEST(MotionModelCV, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_kalmanFilter_egoMotion) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>,
                tracking::filter::KalmanFilter>::run_with_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<TestFloatType>,
                tracking::filter::KalmanFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::FactoredCovarianceMatrixPolicy<TestFloatType>,
                tracking::filter::InformationFilter>::run_without_ego_motion_compensation();
}

TEST(MotionModelCV, convertCA_fullCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FullCovarianceMatrixPolicy<TestFloatType>>;
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
  using MMCV = tracking::motion::MotionModelCV<tracking::math::FactoredCovarianceMatrixPolicy<TestFloatType>>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::FactoredCovarianceMatrixPolicy<TestFloatType>>;
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

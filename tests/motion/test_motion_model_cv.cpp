#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"
#include "trackingLib/motion/motion_model_cv.hpp"


// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          template <typename FloatType>
          class FilterType,
          typename FloatType>
struct TestPredictCV
{
  using MM = tracking::motion::MotionModelCV<CovarianceMatrixType, FloatType>;

  static void init(typename MM::StateCov& cov, typename MM::StateCov& expCov, const tracking::filter::KalmanFilter<FloatType>&)
  {
    // no change required
  }

  static void init(typename MM::StateCov& cov,
                   typename MM::StateCov& expCov,
                   const tracking::filter::InformationFilter<FloatType>&)
  {
    cov    = cov.inverse();
    expCov = expCov.inverse();
  }

  static void run()
  {
    tracking::env::EgoMotion<FloatType> egoMotion{};
    FilterType<FloatType>               filter{};

    typename MM::StateVec vec{{{10}, {2}, {0}, {0}}};
    typename MM::StateCov cov{{{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}}};
    typename MM::StateVec expVec{{{12}, {2}, {0}, {0}}};
    typename MM::StateCov expCov{{{8.5, 6, 0, 0}, {6, 11, 0, 0}, {0, 0, 3.6, 5.1}, {0, 0, 5.1, 10.1}}};
    init(cov, expCov, filter);

    // instantiate MM with mocked EgoMotion compensation
    testing::NiceMock<test::MotionModelNoEgoMotionMock<MM>> mm{vec, cov};
    mm.delegate();
    EXPECT_CALL(mm, compensateEgoMotion);

    // call UUT
    mm.predict(static_cast<FloatType>(1.0), filter, egoMotion);

    for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    {
      EXPECT_FLOAT_EQ(mm._vec[row], expVec[row]);
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_FLOAT_EQ(mm._cov(row, col), expCov(row, col));
      }
    }
  }
};

TEST(MotionModelCV, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFull, tracking::filter::KalmanFilter, float64>::run();
}

TEST(MotionModelCV, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFactored, tracking::filter::KalmanFilter, float64>::run();
}

TEST(MotionModelCV, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFull, tracking::filter::InformationFilter, float64>::run();
}

TEST(MotionModelCV, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCV<tracking::math::CovarianceMatrixFactored, tracking::filter::InformationFilter, float64>::run();
}

TEST(MotionModelCV, convertCA_fullCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
  MMCA::StateVec vec({{10}, {2}, {0}, {0}, {2}, {0.1}});
  MMCA::StateCov::compose_type cov({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  MMCA mm_ca{vec, MMCA::StateCov(cov)};
  MMCV mm_cv{};

  // call UUT
  mm_cv.convertFrom(mm_ca);
  
  // verify
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::X],  mm_cv._vec[MMCV::X]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::VX], mm_cv._vec[MMCV::VX]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::Y],  mm_cv._vec[MMCV::Y]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::VY], mm_cv._vec[MMCV::VY]);
  const auto& caFull = mm_ca._cov;
  const auto& cvFull = mm_cv._cov;
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::X),  cvFull(MMCV::X,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::VX), cvFull(MMCV::X,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::Y),  cvFull(MMCV::X,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::VY), cvFull(MMCV::X,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::X),  cvFull(MMCV::VX, MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::VX), cvFull(MMCV::VX, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::Y),  cvFull(MMCV::VX, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::VY), cvFull(MMCV::VX, MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::X),  cvFull(MMCV::Y,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::VX), cvFull(MMCV::Y,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::Y),  cvFull(MMCV::Y,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::VY), cvFull(MMCV::Y,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::X),  cvFull(MMCV::VY, MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::VX), cvFull(MMCV::VY, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::Y),  cvFull(MMCV::VY, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::VY), cvFull(MMCV::VY, MMCV::VY));
  // clang-format on}
}

TEST(MotionModelCV, convertCA_facCov) // NOLINT
{
  // clang-format off
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, float32>;
  MMCA::StateVec vec({{10}, {2}, {0}, {0}, {2}, {0.1}});
  MMCA::StateCov::compose_type cov({
    {10.9911,   -3.3077,    0.4975,    5.0849,   -0.4707,    2.3979},
    {-3.3077,   13.7164,   -3.5610,   -1.1132,    0.3277,    0.1886},
    { 0.4975,   -3.5610,    2.7362,   -0.2259,   -0.9420,   -0.3686},
    { 5.0849,   -1.1132,   -0.2259,    2.6187,   -0.1260,    1.2376},
    {-0.4707,    0.3277,   -0.9420,   -0.1260,    1.2990,    0.8641},
    { 2.3979,    0.1886,   -0.3686,    1.2376,    0.8641,    1.5631},
  });
  MMCA mm_ca{vec, MMCA::StateCov(cov)};
  MMCV mm_cv{};

  // call UUT
  mm_cv.convertFrom(mm_ca);

  // verify
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::X],  mm_cv._vec[MMCV::X]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::VX], mm_cv._vec[MMCV::VX]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::Y],  mm_cv._vec[MMCV::Y]);
  EXPECT_FLOAT_EQ(mm_ca._vec[MMCA::VY], mm_cv._vec[MMCV::VY]);
  const auto caFull = mm_ca._cov();
  const auto cvFull = mm_cv._cov();
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::X),  cvFull(MMCV::X,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::VX), cvFull(MMCV::X,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::Y),  cvFull(MMCV::X,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::X,  MMCA::VY), cvFull(MMCV::X,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::X),  cvFull(MMCV::VX, MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::VX), cvFull(MMCV::VX, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::Y),  cvFull(MMCV::VX, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::VX, MMCA::VY), cvFull(MMCV::VX, MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::X),  cvFull(MMCV::Y,  MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::VX), cvFull(MMCV::Y,  MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::Y),  cvFull(MMCV::Y,  MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::Y,  MMCA::VY), cvFull(MMCV::Y,  MMCV::VY));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::X),  cvFull(MMCV::VY, MMCV::X));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::VX), cvFull(MMCV::VY, MMCV::VX));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::Y),  cvFull(MMCV::VY, MMCV::Y));
  EXPECT_FLOAT_EQ(caFull(MMCA::VY, MMCA::VY), cvFull(MMCV::VY, MMCV::VY));
  // clang-format on
}




#include "gmock/gmock.h"

#include "mocks/motion_model_no_ego_motion.hpp"
#include "trackingLib/motion/motion_model_ca.hpp" // IWYU pragma: keep


// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, float32>;

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType,
          template <typename FloatType>
          class FilterType,
          typename FloatType>
struct TestPredictCA
{
  using MM = tracking::motion::MotionModelCA<CovarianceMatrixType, FloatType>;

  static void init(typename MM::StateCov&, typename MM::StateCov&, const tracking::filter::KalmanFilter<FloatType>&)
  {
    // no change required
  }

  static void init(typename MM::StateCov& cov,
                   typename MM::StateCov& expCov,
                   const tracking::filter::InformationFilter<FloatType>&)
  {
    cov    = cov.inverse().value();
    expCov = expCov.inverse().value();
  }

  static void run()
  {
    tracking::env::EgoMotion<FloatType> egoMotion{};
    FilterType<FloatType>               filter{};

    auto vec = MM::StateVec::FromList({10, 2, 2, 0, 0, 0.1});
    auto cov = MM::StateCov::FromList({{5, 0, 0, 0, 0, 0},
                                       {0, 1, 0, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 0.1, 0},
                                       {0, 0, 0, 0, 0, 1}});

    auto expVec = MM::StateVec::FromList({13, 4, 2, 0.05, 0.1, 0.1});
    auto expCov = MM::StateCov::FromList({{31.25, 51.5, 50.5, 0, 0, 0},
                                          {51.5, 102, 101, 0, 0, 0},
                                          {50.5, 101, 101, 0, 0, 0},
                                          {0, 0, 0, 26.35, 50.6, 50.5},
                                          {0, 0, 0, 50.6, 101.1, 101},
                                          {0, 0, 0, 50.5, 101, 101}});
    init(cov, expCov, filter);

    // instantiate MM with mocked EgoMotion compensation
    testing::NiceMock<test::MotionModelNoEgoMotionMock<MM>> mm{vec, cov};
    mm.delegate();
    EXPECT_CALL(mm, compensateEgoMotion);

    // call UUT
    mm.predict(static_cast<FloatType>(1.0), filter, egoMotion);

    for (auto row = 0; row < MM::NUM_STATE_VARIABLES; ++row)
    {
      EXPECT_FLOAT_EQ(mm._vec.at_unsafe(row), expVec.at_unsafe(row));
      for (auto col = 0; col < MM::NUM_STATE_VARIABLES; ++col)
      {
        EXPECT_FLOAT_EQ(mm._cov.at_unsafe(row, col), expCov.at_unsafe(row, col));
      }
    }
  }
};

TEST(MotionModelCA, predict_fullCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFull, tracking::filter::KalmanFilter, float64>::run();
}

TEST(MotionModelCA, predict_factoredCov_kalmanFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFactored, tracking::filter::KalmanFilter, float64>::run();
}

TEST(MotionModelCA, predict_fullCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFull, tracking::filter::InformationFilter, float64>::run();
}

TEST(MotionModelCA, predict_factoredCov_informationFilter) // NOLINT
{
  TestPredictCA<tracking::math::CovarianceMatrixFactored, tracking::filter::InformationFilter, float64>::run();
}

TEST(MotionModelCA, convertCV_fullCov) // NOLINT
{
  // clang-format off
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  auto vec = MMCV::StateVec::FromList({10, 2, 0, 2});
  auto cov = MMCV::StateCov::FromList({
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
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, float32>;
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;
  auto vec = MMCV::StateVec::FromList({10, 2, 0, 2});
  auto cov = MMCV::StateCov::FromList({
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

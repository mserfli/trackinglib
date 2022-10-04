#include "gtest/gtest.h"
#include "trackingLib/env/ego_motion.h"
#include "trackingLib/motion/motion_model_ca.hpp"

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
template class tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, float32>;


TEST(MotionModelCA, predict_fullCov)
{
  using MM = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}, {2}, {0.1}});
  MM::StateCov cov(
      {{5, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 1}});
  MM mm{vec, cov};
  tracking::env::EgoMotion<float32> egoMotion{};
  egoMotion._displacementCog.vec[tracking::env::EgoMotion<float32>::DS_X] = 10.0F;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec.print();
  mm._cov.print();
}

TEST(MotionModelCA, predict_factoredCov)
{
  using MM = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFactored, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}, {2}, {0.1}});
  MM::StateCov cov(
      {{5, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 1}});
  MM mm{vec, cov};
  tracking::env::EgoMotion<float32>       egoMotion;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec.print();
  // mm._cov->print();
}

TEST(MotionModelCV, convertCV_fullCov)
{
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
  MMCV::StateVec vec({{10}, {2}, {0}, {0}});
  MMCV::StateCov cov({{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}});
  MMCV           mm_cv{vec, cov};

  MMCA mm_ca{};
  mm_ca.convertFrom(mm_cv);
}

// NOLINTEND(modernize-use-trailing-return-type)

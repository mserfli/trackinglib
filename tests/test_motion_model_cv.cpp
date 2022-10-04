#include "gtest/gtest.h"
#include "trackingLib/env/ego_motion.h"
#include "trackingLib/motion/motion_model_cv.hpp"

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;

// TODO(matthias): make this a typed test depending on CovarianceType
TEST(MotionModelCV, predict_fullCov)
{
  using MM = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}});
  MM::StateCov cov({{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}});
  MM           mm{};
  mm._vec = vec;
  mm._cov = cov;
  tracking::env::EgoMotion<float32> egoMotion{};
  egoMotion._displacementCog.vec[tracking::env::EgoMotion<float32>::DS_X] = 10.0F;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec.print();
  mm._cov.print();
}

TEST(MotionModelCV, predict_factoredCov)
{
  using MM = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}});
  MM::StateCov cov({{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}});
  MM           mm{};
  mm._vec = vec;
  mm._cov = cov;
  tracking::env::EgoMotion<float32>       egoMotion;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec.print();
  // mm._cov->print();
}

TEST(MotionModelCV, convertCA_fullCov)
{
  using MMCV = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  using MMCA = tracking::motion::MotionModelCA<tracking::math::CovarianceMatrixFull, float32>;
  MMCA::StateVec vec({{10}, {2}, {0}, {0}, {2}, {0.1}});
  MMCA::StateCov cov(
      {{5, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 1}});
  MMCA mm_ca{};
  mm_ca._vec = vec;
  mm_ca._cov = cov;

  MMCV mm_cv{};
  mm_cv.convertFrom(mm_ca);
}

// NOLINTEND(modernize-use-trailing-return-type)

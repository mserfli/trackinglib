#include "gtest/gtest.h"
#include <trackingLib/motion/motion_model_cv.h>

// NOLINTBEGIN(modernize-use-trailing-return-type)
// instatiate all templates for full coverage report
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
template class tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;


TEST(MotionModelCV, predict_fullCov)
{
  using MM = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFull, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}});
  MM::StateCov cov({{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}});
  MM           mm{};
  mm.setVec(make_unique<MM::StateVec>(vec));
  mm.setCov(make_unique<MM::StateCov>(cov));
  tracking::env::EgoMotion<float32>       egoMotion{};
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec->print();
  mm._cov->print();
}

TEST(MotionModelCV, predict_factoredCov)
{
  using MM = tracking::motion::MotionModelCV<tracking::math::CovarianceMatrixFactored, float32>;
  MM::StateVec vec({{10}, {2}, {0}, {0}});
  MM::StateCov cov({{5, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0.1}});
  MM           mm{};
  mm.setVec(make_unique<MM::StateVec>(vec));
  mm.setCov(make_unique<MM::StateCov>(cov));
  tracking::env::EgoMotion<float32>       egoMotion;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);
  
  mm._vec->print();
  //mm._cov->print();
}

// NOLINTEND(modernize-use-trailing-return-type)

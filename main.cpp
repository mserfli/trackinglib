//
//  main.cpp
//  TrackingLib
//
//  Created by Matthias Serfling on 20.09.22.
//

#include "base/covariance_matrix_full.h"
#include "base/covariance_matrix_factored.h"
#include "env/ego_motion.h"
#include "filter/kalman_filter.h"
#include "motion/motion_model_cv.h"

using tracking::base::CovarianceMatrixFull;
using tracking::base::CovarianceMatrixFactored;
using tracking::env::EgoMotion;
using tracking::filter::KalmanFilter;
using tracking::motion::IMotionModel;
using tracking::motion::MotionModelCV;

auto main() -> int
{
  std::array<std::unique_ptr<const IMotionModel<float32>>, 3> mmVec;
  for (auto& it : mmVec)
  {
    it.reset(new MotionModelCV<CovarianceMatrixFull, float32>);
  }

  MotionModelCV<CovarianceMatrixFull, float32> mm0{};
  MotionModelCV<CovarianceMatrixFactored, float32> mm1{};
  EgoMotion<float32>                           egoMotion;
  KalmanFilter<float32>                        kf;
  mm0.predict(1.0F, kf, egoMotion);
  mm1.predict(1.0F, kf, egoMotion);
  return 0;
}

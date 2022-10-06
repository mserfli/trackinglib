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
  MM           mm{vec, cov};
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
  MM           mm{vec, cov};
  tracking::env::EgoMotion<float32>       egoMotion;
  tracking::filter::KalmanFilter<float32> kf;
  mm.predict(1.0F, kf, egoMotion);

  mm._vec.print();
  // mm._cov->print();
}

TEST(MotionModelCV, convertCA_fullCov)
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
  
  mm_ca._cov.print();
  mm_cv._cov.print();

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

TEST(MotionModelCV, convertCA_facCov)
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


// NOLINTEND(modernize-use-trailing-return-type)

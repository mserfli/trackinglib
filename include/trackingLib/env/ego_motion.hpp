#ifndef FAA51682_4454_40DA_82DD_798B885BD9D8
#define FAA51682_4454_40DA_82DD_798B885BD9D8

#include "env/ego_motion.h"

#include "math/analysis/functions.h"
#include "math/linalg/conversions/covariance_matrix_conversions.hpp" // IWYU pragma: keep
#include "math/linalg/covariance_matrix_factored.hpp"                // IWYU pragma: keep
#include "math/linalg/covariance_matrix_full.hpp"                    // IWYU pragma: keep
#include "math/linalg/matrix.hpp"                                    // IWYU pragma: keep
#include "math/linalg/square_matrix.hpp"                             // IWYU pragma: keep
#include "math/linalg/vector.hpp"                                    // IWYU pragma: keep
#include <cmath>

namespace tracking
{
namespace env
{

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::compensatePosition(FloatType_&      posXNewEgo,
                                                                      FloatType_&      posYNewEgo,
                                                                      const FloatType_ posXOldEgo,
                                                                      const FloatType_ posYOldEgo) const
{
  // transfer to COG
  auto posOldCog = Point2d::FromValues(posXOldEgo, posYOldEgo);
  posOldCog.x() += _geometry.distCog2Ego;

  // translate first
  // compensate motion displacement
  const auto displacement = Point2d::FromValues(_displacementCog.vec.at_unsafe(DS_X), _displacementCog.vec.at_unsafe(DS_Y));
  const auto translated   = Point2d{posOldCog - displacement};

  // rotate according to deltaPsi
  compensateDirection(posXNewEgo, posYNewEgo, translated.x(), translated.y());

  // transfer from COG
  posXNewEgo -= _geometry.distCog2Ego;
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::compensateDirection(FloatType_&      dxNewEgo,
                                                                       FloatType_&      dyNewEgo,
                                                                       const FloatType_ dxOldEgo,
                                                                       const FloatType_ dyOldEgo) const
{
  // rotate a vector (velocity or acceleration) according to deltaPsi
  dxNewEgo = (_displacementCog.cosDeltaPsi * dxOldEgo) + (_displacementCog.sinDeltaPsi * dyOldEgo);
  dyNewEgo = -(_displacementCog.sinDeltaPsi * dxOldEgo) + (_displacementCog.cosDeltaPsi * dyOldEgo);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
auto EgoMotion<CovarianceMatrixType_, FloatType_>::isLinearMotion() const -> bool
{
  constexpr FloatType_ omegaThreshold = static_cast<FloatType_>(9e-3);
  return (math::pow<4>(_motion.w) < omegaThreshold);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::calcDisplacement()
{
  calcDisplacementVector(_displacementCog, _motion, _dt);
  const auto J = isLinearMotion() ? calcLinearMotionJacobian(_motion, _dt) : calcCircularMotionJacobian(_motion, _dt);
  calcDisplacementCovariance(_displacementCog, J, _motion);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
auto EgoMotion<CovarianceMatrixType_, FloatType_>::calcLinearMotionJacobian(const InertialMotion& motion, FloatType_ dt)
    -> math::SquareMatrix<FloatType_, 3, true>
{
  const auto T = dt;
  const auto v = motion.v;
  const auto a = motion.a;

  math::SquareMatrix<FloatType_, 3, true> J{};
  J.setZeros();
  J.at_unsafe(0, 0) = T;
  J.at_unsafe(0, 1) = (1.0 / 2.0) * pow(T, 2);
  J.at_unsafe(1, 2) = (1.0 / 4.0) * pow(T, 3) * a + (1.0 / 2.0) * pow(T, 2) * v;
  J.at_unsafe(2, 2) = T;

  return std::move(J);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
auto EgoMotion<CovarianceMatrixType_, FloatType_>::calcCircularMotionJacobian(const InertialMotion& motion, FloatType_ dt)
    -> math::SquareMatrix<FloatType_, 3, true>
{
  const auto T = dt;
  const auto v = motion.v;
  const auto a = motion.a;
  const auto w = motion.w;

  math::SquareMatrix<FloatType_, 3, true> J{};
  J.setZeros();
  J.at_unsafe(0, 0) = 2 * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w;
  J.at_unsafe(0, 1) = T * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w;
  J.at_unsafe(0, 2) = -1.0 / 2.0 * T * (T * a + 2 * v) * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w +
                      (1.0 / 2.0) * T * (T * a + 2 * v) * math::pow<2>(std::cos((1.0 / 2.0) * T * w)) / w -
                      (T * a + 2 * v) * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / (w * w);
  J.at_unsafe(1, 0) = 2 * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w;
  J.at_unsafe(1, 1) = T * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w;
  J.at_unsafe(1, 2) = T * (T * a + 2 * v) * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w -
                      (T * a + 2 * v) * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / (w * w);
  J.at_unsafe(2, 2) = T;

  return std::move(J);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::calcDisplacementVector(Displacement&         displacement,
                                                                          const InertialMotion& motion,
                                                                          FloatType_            dt)
{
  const FloatType_ T = dt;
  const FloatType_ v = motion.v;
  const FloatType_ a = motion.a;
  const FloatType_ w = motion.w;

  // Calculate angular displacement: dφ = ω·T
  const FloatType_ dphi   = w * dt;
  const FloatType_ dphi_2 = dphi / 2.0;
  // Precompute trigonometric values for efficiency
  const FloatType_ sin_dphi   = std::sin(dphi);
  const FloatType_ sin_dphi_2 = std::sin(dphi_2);
  const FloatType_ cos_dphi   = std::cos(dphi);
  const FloatType_ cos_dphi_2 = std::cos(dphi_2);

  // Calculate secant length
  // c = (2·v·T + a·T^2)/dφ · sin(dφ/2) = T * (2·v + a·T)/dφ · sin(dφ/2)
  const FloatType_ c = T * (2.0 * v + a * T) / dphi * sin_dphi_2;

  // Calculate displacement components
  const FloatType_ dx = c * cos_dphi_2;
  const FloatType_ dy = c * sin_dphi_2;

  // Set displacement vector
  displacement.vec.at_unsafe(DS_X)   = dx;
  displacement.vec.at_unsafe(DS_Y)   = dy;
  displacement.vec.at_unsafe(DS_PSI) = dphi;
  displacement.sinDeltaPsi           = sin_dphi;
  displacement.cosDeltaPsi           = cos_dphi;
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::calcDisplacementCovariance(Displacement& displacement,
                                                                              const math::SquareMatrix<FloatType_, 3, true>& J,
                                                                              const InertialMotion& motion)
{
  // clang-format off
  // Create diagonal matrix from motion uncertainties
  auto diag =
    math::DiagonalMatrix<FloatType_, 3>::FromList({
      math::pow<2>(motion.sv), math::pow<2>(motion.sa), math::pow<2>(motion.sw)
  });
  // clang-format on

  // Create covariance matrix using generic FromDiagonal constructor
  displacement.cov = std::move(DisplacementCov::FromDiagonal(diag));

  // Apply the transformation: P = J * Pin * J^T using apaT
  displacement.cov.apaT(J);
}


#if 0
// Implementation of displacement computation methods based on circular motion equations
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::calcCircularMotionDisplacement(Displacement&         displacement,
                                                                                  const InertialMotion& motion,
                                                                                  FloatType_            dt)
{
  const FloatType_ T = dt;
  const FloatType_ v = motion.v;
  const FloatType_ a = motion.a;
  const FloatType_ w = motion.w;

  // Calculate angular displacement: dφ = ω·T
  const FloatType_ dphi   = w * dt;
  const FloatType_ dphi_2 = dphi / 2.0;
  // Precompute trigonometric values for efficiency
  const FloatType_ sin_dphi   = std::sin(dphi);
  const FloatType_ sin_dphi_2 = std::sin(dphi_2);
  const FloatType_ cos_dphi   = std::cos(dphi);
  const FloatType_ cos_dphi_2 = std::cos(dphi_2);

  // Calculate secant length
  // c = (2·v·T + a·T^2)/dφ · sin(dφ/2) = T * (2·v + a·T)/dφ · sin(dφ/2)
  const FloatType_ c = T * (2.0 * v + a * T) / dphi * sin_dphi_2;

  // Calculate displacement components
  const FloatType_ dx = c * cos_dphi_2;
  const FloatType_ dy = c * sin_dphi_2;

  // Set displacement vector
  displacement.vec.at_unsafe(DS_X)   = dx;
  displacement.vec.at_unsafe(DS_Y)   = dy;
  displacement.vec.at_unsafe(DS_PSI) = dphi;
  displacement.sinDeltaPsi           = sin_dphi;
  displacement.cosDeltaPsi           = cos_dphi;

  // Compute covariance matrix using first-order error propagation
  // Based on: P = J * Pin * J^T
  // where J is the Jacobian of [dx, dy, dphi] with respect to [v, a, w]

  // Build Jacobian matrix (3x3)
  // J = [dx_dv, dx_da, dx_dw;
  //      dy_dv, dy_da, dy_dw;
  //      dphi_dv, dphi_da, dphi_dw]

  // Build input covariance matrix (diagonal)
  // Pin = diag([σ_v^2, σ_a^2, σ_ω^2])

  // Compute output covariance using matrix operations: P = J * Pin * J^T
  // Initialize covariance matrix to zeros
  DisplacementCov&& cov{};
  cov.setZeros();

  // Set diagonal elements of input covariance (Pin is diagonal with variances)
  cov.at_unsafe(0, 0) = math::pow<2>(motion.sv); // σ_v²
  cov.at_unsafe(1, 1) = math::pow<2>(motion.sa); // σ_a²
  cov.at_unsafe(2, 2) = math::pow<2>(motion.sw); // σ_ω²

  // Create Jacobian matrix (3x3)
  using JacobianMatrix = math::SquareMatrix<FloatType_, 3, true>;
  JacobianMatrix J{};

  J.at_unsafe(0, 0) = 2 * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w;
  J.at_unsafe(0, 1) = T * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w;
  J.at_unsafe(0, 2) = -1.0 / 2.0 * T * (T * a + 2 * v) * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w +
                      (1.0 / 2.0) * T * (T * a + 2 * v) * math::pow<2>(std::cos((1.0 / 2.0) * T * w)) / w -
                      (T * a + 2 * v) * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / (w * w);
  J.at_unsafe(1, 0) = 2 * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w;
  J.at_unsafe(1, 1) = T * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / w;
  J.at_unsafe(1, 2) = T * (T * a + 2 * v) * std::sin((1.0 / 2.0) * T * w) * std::cos((1.0 / 2.0) * T * w) / w -
                      (T * a + 2 * v) * math::pow<2>(std::sin((1.0 / 2.0) * T * w)) / (w * w);
  J.at_unsafe(2, 0) = 0;
  J.at_unsafe(2, 1) = 0;
  J.at_unsafe(2, 2) = T;

  // Compute covariance in-place: P = J * Pin * J^T using apaT method
  cov.apaT(J);

  std::cout << cov << std::endl;

  displacement.cov = std::move(cov);
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType_, typename FloatType_>
void EgoMotion<CovarianceMatrixType_, FloatType_>::calcLinearMotionDisplacement(Displacement&         displacement,
                                                                                const InertialMotion& motion,
                                                                                FloatType_            dt)
{
  // Small omega approximation using the limit equations
  // lim(ω→0) of the displacement equations

  const FloatType_ T = dt;
  const FloatType_ v = motion.v;
  const FloatType_ a = motion.a;
  const FloatType_ w = motion.w;

  // Calculate angular displacement: dφ = ω·T
  const FloatType_ dphi   = w * dt;
  const FloatType_ dphi_2 = dphi / 2.0;
  // Precompute trigonometric values for efficiency
  const FloatType_ sin_dphi_2 = std::sin(dphi_2);
  const FloatType_ cos_dphi_2 = std::cos(dphi_2);

  // Calculate secant length
  // c = (2·v·T + a·T^2)/dφ · sin(dφ/2) = T * (2·v + a·T)/dφ · sin(dφ/2)
  const FloatType_ c = T * (2.0 * v + a * T) / dphi * sin_dphi_2;

  // Calculate displacement components
  FloatType_ dx = c * cos_dphi_2;
  FloatType_ dy = c * sin_dphi_2;


  // Set displacement vector
  displacement.vec.at_unsafe(DS_X)   = dx;
  displacement.vec.at_unsafe(DS_Y)   = dy;
  displacement.vec.at_unsafe(DS_PSI) = dphi;

  // Precompute trigonometric values for efficiency
  displacement.sinDeltaPsi = std::sin(dphi);
  displacement.cosDeltaPsi = std::cos(dphi);

  // Compute covariance matrix using the simplified Jacobian for small omega
  // Jacobian becomes simpler in the limit


  // Limit equations
  // dx = T*v + 0.5*T^2*a - 0.5*T^2*v*w
  // dy = 0.5*T^2*v*w
  // dphi = T*w

  const FloatType_ fac0 = math::pow<2>(T);
  const FloatType_ fac1 = static_cast<FloatType_>(0.5) * fac0;
  dx                    = T * v + fac1 * (a - v * w);
  dy                    = fac1 * v * w;

  // Partial derivatives for small omega case
  const FloatType_ dx_dv_small = T - fac1 * w;
  const FloatType_ dx_da_small = fac1;
  const FloatType_ dx_dw_small = -fac0 * v;

  const FloatType_ dy_dv_small = fac1 * w;
  const FloatType_ dy_da_small = 0.0;
  const FloatType_ dy_dw_small = fac1 * v;

  const FloatType_ dphi_dv_small = 0.0;
  const FloatType_ dphi_da_small = 0.0;
  const FloatType_ dphi_dw_small = T;

  // Compute output covariance using matrix operations: P = J * Pin * J^T
  // Initialize covariance matrix to zeros

  DisplacementCov cov{};
  cov.setZeros();

  // Set diagonal elements of input covariance (Pin is diagonal with variances)
  cov.at_unsafe(0, 0) = math::pow<2>(motion.sv); // σ_v²
  cov.at_unsafe(1, 1) = math::pow<2>(motion.sa); // σ_a²
  cov.at_unsafe(2, 2) = math::pow<2>(motion.sw); // σ_ω²

  // Create Jacobian matrix (3x3)
  using JacobianMatrix = math::SquareMatrix<FloatType_, 3, true>;
  JacobianMatrix J{};

  // Set Jacobian elements for small omega case
  J.at_unsafe(0, 0) = dx_dv_small; // dx/dv
  J.at_unsafe(0, 1) = dx_da_small; // dx/da
  J.at_unsafe(0, 2) = dx_dw_small; // dx/dw

  J.at_unsafe(1, 0) = dy_dv_small; // dy/dv
  J.at_unsafe(1, 1) = dy_da_small; // dy/da
  J.at_unsafe(1, 2) = dy_dw_small; // dy/dw

  J.at_unsafe(2, 0) = dphi_dv_small; // dphi/dv
  J.at_unsafe(2, 1) = dphi_da_small; // dphi/da
  J.at_unsafe(2, 2) = dphi_dw_small; // dphi/dw

  // Compute covariance in-place: P = J * Pin * J^T using apaT method
  cov.apaT(J);

  displacement.cov = std::move(cov);
}
#endif

} // namespace env
} // namespace tracking

#endif // FAA51682_4454_40DA_82DD_798B885BD9D8

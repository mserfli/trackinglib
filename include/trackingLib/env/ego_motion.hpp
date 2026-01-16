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

template <typename FloatType_>
void EgoMotion<FloatType_>::compensatePosition(FloatType_&      posXNewEgo,
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

template <typename FloatType_>
void EgoMotion<FloatType_>::compensateDirection(FloatType_&      dxNewEgo,
                                                FloatType_&      dyNewEgo,
                                                const FloatType_ dxOldEgo,
                                                const FloatType_ dyOldEgo) const
{
  // rotate a vector (velocity or acceleration) according to deltaPsi
  dxNewEgo = (_displacementCog.cosDeltaPsi * dxOldEgo) + (_displacementCog.sinDeltaPsi * dyOldEgo);
  dyNewEgo = -(_displacementCog.sinDeltaPsi * dxOldEgo) + (_displacementCog.cosDeltaPsi * dyOldEgo);
}

template <typename FloatType_>
auto EgoMotion<FloatType_>::isLinearMotion() const -> bool
{
  constexpr FloatType_ omegaThreshold = static_cast<FloatType_>(1e-6);
  return (math::pow<4>(_motion.w) < omegaThreshold);
}

template <typename FloatType_>
void EgoMotion<FloatType_>::calcDisplacement()
{
  if (isLinearMotion())
  {
    calcLinearMotionDisplacement(_displacementCog, _motion, _dt);
  }
  else
  {
    calcCircularMotionDisplacement(_displacementCog, _motion, _dt);
  }
}

// Implementation of displacement computation methods based on circular motion equations
template <typename FloatType_>
void EgoMotion<FloatType_>::calcCircularMotionDisplacement(Displacement&         displacement,
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

  // Jacobian elements (from symbolic derivation)
  // Partial derivatives for dx
  const FloatType_ dx_dv = (2.0 * T * sin_dphi_2 * cos_dphi_2) / dphi;
  const FloatType_ dx_da = (T * T * sin_dphi_2 * cos_dphi_2) / dphi;
  const FloatType_ dx_dw = (-(2.0 * v * T + a * T * T) * sin_dphi_2 * cos_dphi_2 / (dphi * dphi)) +
                           ((2.0 * v * T + a * T * T) * sin_dphi_2 * sin_dphi_2 * T / (2.0 * dphi));

  // Partial derivatives for dy
  const FloatType_ dy_dv = (2.0 * T * sin_dphi_2 * sin_dphi_2) / dphi;
  const FloatType_ dy_da = (T * T * sin_dphi_2 * sin_dphi_2) / dphi;
  const FloatType_ dy_dw = (-(2.0 * v * T + a * T * T) * sin_dphi_2 * sin_dphi_2 / (dphi * dphi)) +
                           ((2.0 * v * T + a * T * T) * sin_dphi_2 * cos_dphi_2 * T / (2.0 * dphi));

  // Partial derivatives for dphi
  const FloatType_ dphi_dv = 0.0;
  const FloatType_ dphi_da = 0.0;
  const FloatType_ dphi_dw = T;

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

  // Set Jacobian elements
  J.at_unsafe(0, 0) = dx_dv; // dx/dv
  J.at_unsafe(0, 1) = dx_da; // dx/da
  J.at_unsafe(0, 2) = dx_dw; // dx/dw

  J.at_unsafe(1, 0) = dy_dv; // dy/dv
  J.at_unsafe(1, 1) = dy_da; // dy/da
  J.at_unsafe(1, 2) = dy_dw; // dy/dw

  J.at_unsafe(2, 0) = dphi_dv; // dphi/dv
  J.at_unsafe(2, 1) = dphi_da; // dphi/da
  J.at_unsafe(2, 2) = dphi_dw; // dphi/dw

  // Compute covariance in-place: P = J * Pin * J^T using apaT method
  cov.apaT(J);
  displacement.cov = std::move(cov);
}

template <typename FloatType_>
void EgoMotion<FloatType_>::calcLinearMotionDisplacement(Displacement& displacement, const InertialMotion& motion, FloatType_ dt)
{
  // Small omega approximation using the limit equations
  // lim(ω→0) of the displacement equations

  const FloatType_ T = dt;
  const FloatType_ v = motion.v;
  const FloatType_ a = motion.a;
  const FloatType_ w = motion.w;

  // Limit equations
  // dx = T*v + 0.5*T^2*a - 0.5*T^2*v*w
  // dy = 0.5*T^2*v*w
  // dphi = T*w

  const FloatType_ fac0 = math::pow<2>(T);
  const FloatType_ fac1 = static_cast<FloatType_>(0.5) * fac0;
  const FloatType_ dx   = T * v + fac1 * (a - v * w);
  const FloatType_ dy   = fac1 * v * w;
  const FloatType_ dpsi = T * w;

  // Set displacement vector
  displacement.vec.at_unsafe(DS_X)   = dx;
  displacement.vec.at_unsafe(DS_Y)   = dy;
  displacement.vec.at_unsafe(DS_PSI) = dpsi;

  // Precompute trigonometric values for efficiency
  displacement.sinDeltaPsi = std::sin(dpsi);
  displacement.cosDeltaPsi = std::cos(dpsi);

  // Compute covariance matrix using the simplified Jacobian for small omega
  // Jacobian becomes simpler in the limit

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

} // namespace env
} // namespace tracking

#endif // FAA51682_4454_40DA_82DD_798B885BD9D8

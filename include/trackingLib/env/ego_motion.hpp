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
#include <limits>

namespace tracking
{
namespace env
{

template <typename CovarianceMatrixPolicy_>
void EgoMotion<CovarianceMatrixPolicy_>::compensatePosition(FloatType&      posXNewEgo,
                                                            FloatType&      posYNewEgo,
                                                            const FloatType posXOldEgo,
                                                            const FloatType posYOldEgo) const
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

template <typename CovarianceMatrixPolicy_>
void EgoMotion<CovarianceMatrixPolicy_>::compensateDirection(FloatType&      dxNewEgo,
                                                             FloatType&      dyNewEgo,
                                                             const FloatType dxOldEgo,
                                                             const FloatType dyOldEgo) const
{
  // rotate a vector (velocity or acceleration) according to deltaPsi
  dxNewEgo = (_displacementCog.cosDeltaPsi * dxOldEgo) + (_displacementCog.sinDeltaPsi * dyOldEgo);
  dyNewEgo = -(_displacementCog.sinDeltaPsi * dxOldEgo) + (_displacementCog.cosDeltaPsi * dyOldEgo);
}

template <typename CovarianceMatrixPolicy_>
auto EgoMotion<CovarianceMatrixPolicy_>::isLinearMotion() const -> bool
{
  constexpr FloatType omegaThreshold = static_cast<FloatType>(9e-3);
  return (math::pow<4>(_motion.w) < omegaThreshold);
}

template <typename CovarianceMatrixPolicy_>
void EgoMotion<CovarianceMatrixPolicy_>::calcDisplacement()
{
  calcDisplacementVector(_displacementCog, _motion, _dt);
  const auto J = isLinearMotion() ? calcLinearMotionJacobian(_motion, _dt) : calcCircularMotionJacobian(_motion, _dt);
  calcDisplacementCovariance(_displacementCog, J, _motion);
}

template <typename CovarianceMatrixPolicy_>
auto EgoMotion<CovarianceMatrixPolicy_>::calcLinearMotionJacobian(const InertialMotion& motion,
                                                                  FloatType dt) -> math::SquareMatrix<FloatType, 3, true>
{
  const auto T = dt;
  const auto v = motion.v;
  const auto a = motion.a;

  math::SquareMatrix<FloatType, 3, true> J{};
  J.setZeros();
  J.at_unsafe(0, 0) = T;
  J.at_unsafe(0, 1) = (0.5) * pow(T, 2);
  J.at_unsafe(1, 2) = (0.25) * pow(T, 3) * a + (0.5) * pow(T, 2) * v;
  J.at_unsafe(2, 2) = T;

  return std::move(J);
}

template <typename CovarianceMatrixPolicy_>
auto EgoMotion<CovarianceMatrixPolicy_>::calcCircularMotionJacobian(const InertialMotion& motion,
                                                                    FloatType dt) -> math::SquareMatrix<FloatType, 3, true>
{
  const auto T = dt;
  const auto v = motion.v;
  const auto a = motion.a;
  const auto w = motion.w;

  math::SquareMatrix<FloatType, 3, true> J{};
  J.setZeros();
  J.at_unsafe(0, 0) = 2 * std::sin((0.5) * T * w) * std::cos((0.5) * T * w) / w;
  J.at_unsafe(0, 1) = T * std::sin((0.5) * T * w) * std::cos((0.5) * T * w) / w;
  J.at_unsafe(0, 2) = -0.5 * T * (T * a + 2 * v) * math::pow<2>(std::sin((0.5) * T * w)) / w +
                      (0.5) * T * (T * a + 2 * v) * math::pow<2>(std::cos((0.5) * T * w)) / w -
                      (T * a + 2 * v) * std::sin((0.5) * T * w) * std::cos((0.5) * T * w) / (w * w);
  J.at_unsafe(1, 0) = 2 * math::pow<2>(std::sin((0.5) * T * w)) / w;
  J.at_unsafe(1, 1) = T * math::pow<2>(std::sin((0.5) * T * w)) / w;
  J.at_unsafe(1, 2) = T * (T * a + 2 * v) * std::sin((0.5) * T * w) * std::cos((0.5) * T * w) / w -
                      (T * a + 2 * v) * math::pow<2>(std::sin((0.5) * T * w)) / (w * w);
  J.at_unsafe(2, 2) = T;

  return std::move(J);
}

template <typename CovarianceMatrixPolicy_>
void EgoMotion<CovarianceMatrixPolicy_>::calcDisplacementVector(Displacement&         displacement,
                                                                const InertialMotion& motion,
                                                                FloatType             dt)
{
  const FloatType T = dt;
  const FloatType v = motion.v;
  const FloatType a = motion.a;
  const FloatType w = motion.w;
  if (std::abs(T * w) < std::numeric_limits<FloatType>::epsilon())
  {
    // special case for (T*w) very close to zero
    displacement.vec.setZeros();
    displacement.vec.at_unsafe(DS_X) = 0.5 * T * (2 * v + a * T);
    displacement.sinDeltaPsi         = static_cast<FloatType>(0);
    displacement.cosDeltaPsi         = static_cast<FloatType>(1);
  }
  else
  {
    // Calculate angular displacement: dφ = ω·T
    const FloatType dphi   = w * dt;
    const FloatType dphi_2 = dphi / 2.0;
    // Precompute trigonometric values for efficiency
    const FloatType sin_dphi   = std::sin(dphi);
    const FloatType sin_dphi_2 = std::sin(dphi_2);
    const FloatType cos_dphi   = std::cos(dphi);
    const FloatType cos_dphi_2 = std::cos(dphi_2);

    // Calculate secant length
    // c = (2·v·T + a·T^2)/dφ · sin(dφ/2) = T * (2·v + a·T)/dφ · sin(dφ/2)
    const FloatType c = T * (2.0 * v + a * T) / dphi * sin_dphi_2;

    // Calculate displacement components
    const FloatType dx = c * cos_dphi_2;
    const FloatType dy = c * sin_dphi_2;

    // Set displacement vector
    displacement.vec.at_unsafe(DS_X)   = dx;
    displacement.vec.at_unsafe(DS_Y)   = dy;
    displacement.vec.at_unsafe(DS_PSI) = dphi;
    displacement.sinDeltaPsi           = sin_dphi;
    displacement.cosDeltaPsi           = cos_dphi;
  }
}

template <typename CovarianceMatrixPolicy_>
void EgoMotion<CovarianceMatrixPolicy_>::calcDisplacementCovariance(Displacement&                                 displacement,
                                                                    const math::SquareMatrix<FloatType, 3, true>& J,
                                                                    const InertialMotion&                         motion)
{
  // clang-format off
  // Create diagonal matrix from motion uncertainties
  auto diag =
    math::DiagonalMatrix<FloatType, 3>::FromList({
      math::pow<2>(motion.sv), math::pow<2>(motion.sa), math::pow<2>(motion.sw)
  });
  // clang-format on

  // Create covariance matrix using generic FromDiagonal constructor
  displacement.cov = std::move(DisplacementCov::FromDiagonal(diag));

  // Apply the transformation: P = J * Pin * J^T using apaT
  displacement.cov.apaT(J);
}

} // namespace env
} // namespace tracking

#endif // FAA51682_4454_40DA_82DD_798B885BD9D8

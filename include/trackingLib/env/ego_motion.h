#ifndef EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8
#define EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8

#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/point2d.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace env
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <typename FloatType>
class EgoMotion
{
public:
  enum DisplacementStates
  {
    DS_X = 0,
    DS_Y,
    DS_PSI,
    DS_NUM_VARIABLES
  };

  using DisplacementVec = math::Vector<FloatType, DS_NUM_VARIABLES>;
  using DisplacementCov = math::CovarianceMatrixFull<FloatType, DS_NUM_VARIABLES>;

  struct Motion
  {
  };

  struct Geometry
  {
    FloatType width;
    FloatType length;
    FloatType height;

    FloatType distCog2Ego;
    FloatType distFrontAxle2Ego;
    FloatType distFrontAxle2RearAxle;
  };

  struct Displacement
  {
    DisplacementVec vec;
    DisplacementCov cov;
    FloatType       sinDeltaPsi;
    FloatType       cosDeltaPsi;
  };

  auto getDeltaTime() const -> FloatType { return _dt; }
  auto getDisplacementCog() const -> const Displacement& { return _displacementCog; }
  auto getGeometry() const -> const Geometry& { return _geometry; }

  void compensatePosition(FloatType&      posXNewEgo,
                          FloatType&      posYNewEgo,
                          const FloatType posXOldEgo,
                          const FloatType posYOldEgo) const;

  void compensateDirection(FloatType&      dxNewEgo,
                           FloatType&      dyNewEgo,
                           const FloatType dxOldEgo,
                           const FloatType dyOldEgo) const;

private:
  using Point2d = math::Point2d<FloatType>;
  Geometry     _geometry{};
  Displacement _displacementCog{};
  FloatType    _dt{};
};

template <typename FloatType>
void EgoMotion<FloatType>::compensatePosition(FloatType&      posXNewEgo,
                                              FloatType&      posYNewEgo,
                                              const FloatType posXOldEgo,
                                              const FloatType posYOldEgo) const
{
  // transfer to COG
  Point2d posOldCog{posXOldEgo, posYOldEgo};
  posOldCog.x() += _geometry.distCog2Ego;

  // translate first
  // compensate motion displacement
  const Point2d displacement{_displacementCog.vec[DS_X], _displacementCog.vec[DS_Y]};
  const Point2d translated = posOldCog - displacement;

  // rotate according to deltaPsi
  compensateDirection(posXNewEgo, posYNewEgo, translated.x(), translated.y());

  // transfer from COG
  posXNewEgo -= _geometry.distCog2Ego;
}

template <typename FloatType>
void EgoMotion<FloatType>::compensateDirection(FloatType&      dxNewEgo,
                                               FloatType&      dyNewEgo,
                                               const FloatType dxOldEgo,
                                               const FloatType dyOldEgo) const
{
  // rotate a vector (velocity or acceleration) according to deltaPsi
  dxNewEgo = (_displacementCog.cosDeltaPsi * dxOldEgo) + (_displacementCog.sinDeltaPsi * dyOldEgo);
  dyNewEgo = -(_displacementCog.sinDeltaPsi * dxOldEgo) + (_displacementCog.cosDeltaPsi * dyOldEgo);
}

} // namespace env
} // namespace tracking

#endif // EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8

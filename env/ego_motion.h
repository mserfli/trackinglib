#ifndef ego_motion_h
#define ego_motion_h

#include "base/covariance_matrix_full.h"

namespace tracking
{
namespace env
{
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

  using DisplacementVec = base::Vector<FloatType, DS_NUM_VARIABLES>;
  using DisplacementCov = base::CovarianceMatrixFull<FloatType, DS_NUM_VARIABLES>;

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

  EgoMotion() = default;

  auto getDeltaTime() const -> FloatType { return _dt; }
  auto getDisplacementCog() const -> const Displacement& { return _displacementCog; }
  auto getGeometry() const -> const Geometry& { return _geometry; }

  void compensatePosition(FloatType&      posXNewEgo,
                          FloatType&      posYNewEgo,
                          const FloatType posXOldEgo,
                          const FloatType posYOldEgo) const
  {
    assert(0);
    static_cast<void>(posXNewEgo);
    static_cast<void>(posYNewEgo);
    static_cast<void>(posXOldEgo);
    static_cast<void>(posYOldEgo);
  }

  void compensateDirection(FloatType&      dxNewEgo,
                           FloatType&      dyNewEgo,
                           const FloatType dxOldEgo,
                           const FloatType dyOldEgo) const
  {
    assert(0);
    static_cast<void>(dxNewEgo);
    static_cast<void>(dyNewEgo);
    static_cast<void>(dxOldEgo);
    static_cast<void>(dyOldEgo);
  }

private:
  Geometry     _geometry{};
  Displacement _displacementCog{};
  FloatType    _dt{};
};

} // namespace env
} // namespace tracking
#endif /* ego_motion_h */

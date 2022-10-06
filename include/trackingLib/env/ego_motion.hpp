#ifndef FAA51682_4454_40DA_82DD_798B885BD9D8
#define FAA51682_4454_40DA_82DD_798B885BD9D8

#include "env/ego_motion.h"

namespace tracking
{
namespace env
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen

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

#endif // FAA51682_4454_40DA_82DD_798B885BD9D8

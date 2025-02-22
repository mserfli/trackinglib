#ifndef EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8
#define EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8

#include "base/first_include.h" // IWYU pragma: keep
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
    FloatType width{};
    FloatType length{};
    FloatType height{};

    FloatType distCog2Ego{};
    FloatType distFrontAxle2Ego{};
    FloatType distFrontAxle2RearAxle{};
  };

  struct Displacement
  {
    DisplacementVec vec{};
    DisplacementCov cov{DisplacementCov::Identity()};
    FloatType       sinDeltaPsi{0.0};
    FloatType       cosDeltaPsi{1.0};
  };

  auto getDeltaTime() const -> FloatType { return _dt; }
  auto getDisplacementCog() const -> const Displacement& { return _displacementCog; }
  auto getGeometry() const -> const Geometry& { return _geometry; }

  void compensatePosition(FloatType&      posXNewEgo,
                          FloatType&      posYNewEgo,
                          const FloatType posXOldEgo,
                          const FloatType posYOldEgo) const;

  void compensateDirection(FloatType& dxNewEgo, FloatType& dyNewEgo, const FloatType dxOldEgo, const FloatType dyOldEgo) const;

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  using Point2d = math::Point2d<FloatType>;
  Geometry     _geometry{};
  Displacement _displacementCog{};
  FloatType    _dt{};
};

} // namespace env
} // namespace tracking

#endif // EF810BE3_DCD7_4832_94F8_B3F34EDBC3D8

#ifndef D118B69B_C3E7_43F7_A2FC_F44B7ACF965F
#define D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"

namespace tracking
{
namespace motion
{

// TODO(matthias): add interface contract
// TODO(matthias): add doxygen
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
class StateMem
{
public:
  using StateVec = math::Vector<FloatType, Size>;
  using StateCov = CovarianceMatrixType<FloatType, Size>;
  using StateVecPtr = std::unique_ptr<StateVec>;
  using StateCovPtr = std::unique_ptr<StateCov>;

  auto getVec() const -> const StateVec& { return *_vec; }
  auto getCov() const -> const StateCov& { return *_cov; }

  auto operator[](const sint32 idx) -> FloatType& { return (*_vec)[idx]; }
  auto operator[](const sint32 idx) const -> const FloatType& { return (*_vec)[idx]; }

  // auto operator()(const sint32 row, const sint32 col) -> FloatType& { return (*_cov)(row, col); } // cannot be provided due to
  // factored covariance
  auto operator()(const sint32 row, const sint32 col) const -> const FloatType& { return (*_cov)(row, col); }

  void setVec(StateVecPtr&& vec) { _vec = std::move(vec); }
  void setCov(StateCovPtr&& cov) { _cov = std::move(cov); }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  StateVecPtr _vec{make_unique<StateVec>()};
  StateCovPtr _cov{make_unique<StateCov>(StateCov::Identity())};
};

} // namespace motion
} // namespace tracking
#endif // D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

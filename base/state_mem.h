#ifndef state_mem_h
#define state_mem_h

#include "base/covariance_matrix_full.h"
#include <memory>

namespace tracking
{
namespace base
{

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
class StateMem
{
public:
  StateMem() = default;

  using StateVec    = Vector<FloatType, Size>;
  using StateCov    = CovarianceMatrixType<FloatType, Size>;
  using StateVecPtr = std::unique_ptr<StateVec>;
  using StateCovPtr = std::unique_ptr<StateCov>;

  auto getVec() const -> const StateVec& { return *_vec; }
  auto getCov() const -> const StateCov& { return *_cov; }

  auto operator[](const sint32 idx) -> FloatType& { return (*_vec)[idx]; }
  auto operator[](const sint32 idx) const -> const FloatType& { return (*_vec)[idx]; }

  auto operator()(const sint32 row, const sint32 col) -> FloatType& { return (*_cov)(row, col); }
  auto operator()(const sint32 row, const sint32 col) const -> const FloatType& { return (*_cov)(row, col); }

  void setVec(StateVecPtr&& vec) { _vec = std::move(vec); }
  void setCov(StateCovPtr&& cov) { _cov = std::move(cov); }

private:
  StateVecPtr _vec{new StateVec};
  StateCovPtr _cov{new StateCov};
};

} // namespace base
} // namespace tracking
#endif /* state_mem_h */

#ifndef D118B69B_C3E7_43F7_A2FC_F44B7ACF965F
#define D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

#include "base/first_include.h"
#include "math/linalg/matrix.h"
#include "math/linalg/vector.h"
#include "motion/contracts/state_mem_intf.h"

namespace tracking
{
namespace motion
{

/// \brief StateMem class to represent a state vector with its uncertainty described by the state covariance matrix
/// \tparam CovarianceMatrixType  Used covariance matrix type
/// \tparam FloatType             Used floating point type
/// \tparam Size                  State dimension
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
class StateMem: public contract::StateMemIntf<StateMem<CovarianceMatrixType, FloatType, Size>>
{
public:
  using instance_type = StateMem<CovarianceMatrixType, FloatType, Size>;
  using value_type = FloatType;
  using StateVec = math::Vector<FloatType, Size>;
  using ConstStateVec = const math::Vector<FloatType, Size>;
  using StateCov = CovarianceMatrixType<FloatType, Size>;
  using ConstStateCov = const CovarianceMatrixType<FloatType, Size>;
  using StateVecPtr = std::unique_ptr<StateVec>;
  using StateCovPtr = std::unique_ptr<StateCov>;

  // rule of 5 declarations
  StateMem() = default;
  StateMem(const instance_type& other); // own implementation!!!
  StateMem(instance_type&&) noexcept = default;
  auto operator=(const instance_type& other) -> instance_type&; // own implementation!!!
  auto operator=(instance_type&&) noexcept -> instance_type& = default;
  ~StateMem() = default;

  /// \brief Read access to full state vector
  /// \return const StateVec&
  auto getVec() const -> ConstStateVec& { return *_vec; }
  /// \brief Set a new state vector
  /// \param[in] vec  New state vector
  void setVec(StateVecPtr&& vec) { _vec = std::move(vec); }

  /// \brief Read access to state covariance matrix
  /// \return const StateCov&
  auto getCov() const -> ConstStateCov& { return *_cov; }

  /// \brief Set a new state covariance matrix
  /// \param[in] cov New state covariance matrix
  void setCov(StateCovPtr&& cov) { _cov = std::move(cov); }

  /// \brief Read access to indexed element of the state vector
  /// \param[in] idx  Index in the state vector
  /// \return const FloatType&
  auto operator[](const sint32 idx) const -> FloatType { return (*_vec)[idx]; }

  /// \brief Write access to indexed element of the state vector
  /// \param[in] idx  Index in the state vector
  /// \return FloatType&
  auto operator[](const sint32 idx) -> FloatType& { return (*_vec)[idx]; }

  /// \brief Read access to indexed element of the state covariance matrix
  /// \param[in,out] row  Row index in the state covariance matrix
  /// \param[in,out] col  Col index in the state covariance matrix
  /// \return FloatType
  auto operator()(const sint32 row, const sint32 col) const -> FloatType { return _cov->operator()(row, col); }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief State vector wrapped by a unique ptr to benefit from move operator
  StateVecPtr _vec{make_unique<StateVec>()};
  /// \brief State covariance matrix wrapped by a unique ptr to benefit from move operator
  StateCovPtr _cov{make_unique<StateCov>(StateCov::Identity())};
};

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
StateMem<CovarianceMatrixType, FloatType, Size>::StateMem(const instance_type& other)
{
  // we have to implement the copy ctor as unique_ptr is not copyable
  *_vec = *other._vec;
  *_cov = *other._cov;
}

template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
inline auto StateMem<CovarianceMatrixType, FloatType, Size>::operator=(const instance_type& other) -> StateMem<CovarianceMatrixType, FloatType, Size>&
{
  if (this != &other)
  {
    *this->_vec = *other._vec;
    *this->_cov = *other._cov;
  }
  return *this;
}

} // namespace motion
} // namespace tracking
#endif // D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

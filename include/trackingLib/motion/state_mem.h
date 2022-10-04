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

// TODO(matthias): replace dynamic memory allocation by static allocator https://godbolt.org/z/qb4jxsEYM
// TODO(matthias): or introduce CTORs from all other motion models to allow conversion

/// \brief StateMem class to represent a state vector with its uncertainty described by the state covariance matrix
/// \tparam CovarianceMatrixType  Used covariance matrix type
/// \tparam FloatType             Used floating point type
/// \tparam Size                  State dimension
template <template <typename FloatType, sint32 Size> class CovarianceMatrixType, typename FloatType, sint32 Size>
class StateMem: public contract::StateMemIntf<StateMem<CovarianceMatrixType, FloatType, Size>>
{
public:
  using value_type    = FloatType;
  using StateVec      = math::Vector<FloatType, Size>;
  using ConstStateVec = const math::Vector<FloatType, Size>;
  using StateCov      = CovarianceMatrixType<FloatType, Size>;
  using ConstStateCov = const CovarianceMatrixType<FloatType, Size>;

  // rule of 5 declarations
  StateMem()                      = default;
  StateMem(const StateMem& other) = default;
  StateMem(StateMem&&) noexcept   = default;
  auto operator=(const StateMem& other) -> StateMem& = default;
  auto operator=(StateMem&&) noexcept -> StateMem& = default;
  ~StateMem()                                      = default;

  explicit StateMem(const StateVec& vec, const StateCov& cov)
      : _vec{vec}
      , _cov{cov}
  {
  }

  /// \brief Read access to full state vector
  /// \return const StateVec&
  auto getVec() const -> ConstStateVec& { return _vec; }

  /// \brief Read access to state covariance matrix
  /// \return const StateCov&
  auto getCov() const -> ConstStateCov& { return _cov; }

  /// \brief Read access to indexed element of the state vector
  /// \param[in] idx  Index in the state vector
  /// \return const FloatType&
  auto operator[](const sint32 idx) const -> FloatType { return _vec[idx]; }

  /// \brief Read access to indexed element of the state covariance matrix
  /// \param[in,out] row  Row index in the state covariance matrix
  /// \param[in,out] col  Col index in the state covariance matrix
  /// \return FloatType
  auto operator()(const sint32 row, const sint32 col) const -> FloatType { return _cov(row, col); }

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Write access to full state vector
  /// \return const StateVec&
  auto getVec() -> StateVec& { return _vec; }

  /// \brief Write access to state covariance matrix
  /// \return const StateCov&
  auto getCov() -> StateCov& { return _cov; }

  /// \brief Write access to indexed element of the state vector
  /// \param[in] idx  Index in the state vector
  /// \return FloatType&
  auto operator[](const sint32 idx) -> FloatType& { return _vec[idx]; }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief State vector
  StateVec _vec{StateVec::Zeros()};
  /// \brief State covariance matrix
  StateCov _cov{StateCov::Identity()};
};


} // namespace motion
} // namespace tracking
#endif // D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

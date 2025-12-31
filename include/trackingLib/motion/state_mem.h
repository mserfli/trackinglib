#ifndef D118B69B_C3E7_43F7_A2FC_F44B7ACF965F
#define D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

#include "base/first_include.h" // IWYU pragma: keep
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
template <template <typename FloatType_, sint32 Size_> class CovarianceMatrixType, typename FloatType_, sint32 Size_>
class StateMem: public contract::StateMemIntf<StateMem<CovarianceMatrixType, FloatType_, Size_>>
{
public:
  using value_type    = FloatType_;
  using StateVec      = math::Vector<FloatType_, Size_>;
  using ConstStateVec = const math::Vector<FloatType_, Size_>;
  using StateCov      = CovarianceMatrixType<FloatType_, Size_>;
  using ConstStateCov = const CovarianceMatrixType<FloatType_, Size_>;

  // rule of 5 declarations
  StateMem()                                         = default;
  StateMem(const StateMem& other)                    = default;
  StateMem(StateMem&&) noexcept                      = default;
  auto operator=(const StateMem& other) -> StateMem& = default;
  auto operator=(StateMem&&) noexcept -> StateMem&   = default;
  virtual ~StateMem()                                = default;

  /// \brief Read access to full state vector
  /// \return const StateVec&
  auto getVec() const -> ConstStateVec& { return _vec; }

  /// \brief Read access to state covariance matrix
  /// \return const StateCov&
  auto getCov() const -> ConstStateCov& { return _cov; }

  /// \brief Read access to indexed element of the state vector
  /// \param[in] idx  Index in the state vector
  /// \return const FloatType&
  auto operator[](const sint32 idx) const -> FloatType_ { return _vec.at_unsafe(idx); }

  /// \brief Read access to indexed element of the state covariance matrix
  /// \param[in,out] row  Row index in the state covariance matrix
  /// \param[in,out] col  Col index in the state covariance matrix
  /// \return FloatType
  auto operator()(const sint32 row, const sint32 col) const -> FloatType_ { return _cov.at_unsafe(row, col); }

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
  auto operator[](const sint32 idx) -> FloatType_& { return _vec.at_unsafe(idx); }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Testing: Construct a new State Mem object
  /// \param[in] vec
  /// \param[in] cov
  explicit StateMem(const StateVec& vec, const StateCov& cov)
      : _vec{vec}
      , _cov{cov}
  {
  }

  /// \brief State vector
  StateVec _vec{StateVec::Zeros()};
  /// \brief State covariance matrix
  StateCov _cov{StateCov::Identity()};
};


} // namespace motion
} // namespace tracking
#endif // D118B69B_C3E7_43F7_A2FC_F44B7ACF965F

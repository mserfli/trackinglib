#ifndef CE10BDD8_6874_4771_89BA_D153438C3E01
#define CE10BDD8_6874_4771_89BA_D153438C3E01

#include "base/first_include.h"
#include "base/test_config.h"
#include "base/atomic_types.h"
#include "math/linalg/matrix.h"

namespace tracking
{
namespace math
{

// TODO(matthias): add interface contract
template <typename ValueType_, sint32 Size_>
class Vector: public Matrix<ValueType_, Size_, 1, true>
{
public:
  using Matrix = Matrix<ValueType_, Size_, 1, true>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using Matrix::Matrix;
  
  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Matrix object with given initializer list representing the memory layout of the matrix
  /// \param[in] list  An initializer list describing the memory layout of the matrix
  static auto FromList(const std::initializer_list<ValueType_>& list) -> Vector;

  /// \brief Construct a Zero vector
  /// \return Zero matrix
  static auto Zeros() -> Vector;

  /// \brief Construct a matrix filled with ones
  /// \return One matrix
  static auto Ones() -> Vector;

  /// \brief Create unit vector
  /// \tparam Row_  Index position equal to 1
  /// \return Vector<ValueType_, Size_>
  template <sint32 Row_>
  static auto UnitVector() -> Vector;
  // <---

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  auto operator[](sint32 idx) const -> tl::expected<ValueType_, typename Matrix::Errors>;

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  auto operator[](sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, typename Matrix::Errors>;
  // <---

  //////////////////////////////////////////////////
  // arithmentic operators --->
  /// \brief Dot product with other vector of same size
  /// \param[in] other
  /// \return ValueType_
  auto operator*(const Vector& other) const -> ValueType_;
  // <---

  //////////////////////////////////////////////////
  // other operations --->
  /// \brief Squared L1 norm (aka squared vector length)
  /// \return ValueType_
  auto normSq() const -> ValueType_;

  /// \brief L1 norm (aka vector length)
  /// \return ValueType_
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  auto norm() const -> ValueType_;

  /// \brief In-place normalization to unit length vector
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  void normalize();

  /// \brief Create new vector normalized to unit length vector
  /// \return Vector<ValueType_, Size_>
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  auto normalize() const -> Vector;
  // <---

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  auto at_unsafe(sint32 idx) const -> ValueType_ { return Matrix::at_unsafe(idx, 0); }

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  auto at_unsafe(sint32 idx) -> ValueType_& { return Matrix::at_unsafe(idx, 0); }
  // <---

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  /// \brief Private ctor to convert a Matrix object into a Vector
  /// \param[in] other A base class object
  explicit Vector(const Matrix& other) : Matrix{other} {}
  // clang-format on

  using Matrix::Zeros;
  using Matrix::Ones;
  using Matrix::operator();
  using Matrix::setBlock;
};


template <typename ValueType_, sint32 Size_>
auto operator*(const Matrix<ValueType_, Size_, 1, true>& m, const Vector<ValueType_, Size_>& v) -> ValueType_
{
  return v * Vector<ValueType_, Size_>{m};
}

} // namespace math
} // namespace tracking

#endif // CE10BDD8_6874_4771_89BA_D153438C3E01

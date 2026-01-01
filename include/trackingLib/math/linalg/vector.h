#ifndef CE10BDD8_6874_4771_89BA_D153438C3E01
#define CE10BDD8_6874_4771_89BA_D153438C3E01

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/matrix.h" // IWYU pragma: keep

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class MatrixColumnView TEST_REMOVE_FINAL;

// TODO(matthias): add interface contract
template <typename ValueType_, sint32 Size_>
class Vector: public Matrix<ValueType_, Size_, 1, true> // LCOV_EXCL_LINE
{
public:
  using Matrix = Matrix<ValueType_, Size_, 1, true>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using Matrix::Matrix;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a new Vector<ValueType_, Size_> object
  /// \param[in] other A base class object
  explicit Vector(const Matrix& other)
      : Matrix{other}
  {
  }

  /// \brief Move construct a new Vector<ValueType_, Size_> object
  /// \param[in] other A base class object
  explicit Vector(Matrix&& other) noexcept
      : Matrix{std::move(other)}
  {
  }


  /// \brief Construct a Zero vector
  /// \return Zero matrix
  [[nodiscard]] static auto Zeros() -> Vector;

  /// \brief Construct a matrix filled with ones
  /// \return One matrix
  [[nodiscard]] static auto Ones() -> Vector;

  /// \brief Create unit vector
  /// \tparam Row_  Index position equal to 1
  /// \return Vector<ValueType_, Size_>
  template <sint32 Row_>
  [[nodiscard]] static auto UnitVector() -> Vector;
  // <---

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  [[nodiscard]] auto operator[](sint32 idx) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  [[nodiscard]] auto operator[](sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;
  // <---

  //////////////////////////////////////////////////
  // arithmentic operators --->
  /// \brief Dot product with other vector of same size
  /// \param[in] other
  /// \return ValueType_
  [[nodiscard]] auto operator*(const Vector& other) const -> ValueType_;
  // <---

  //////////////////////////////////////////////////
  // other operations --->
  /// \brief Squared L1 norm (aka squared vector length)
  /// \return ValueType_
  [[nodiscard]] auto normSq() const -> ValueType_;

  /// \brief L2 norm (aka vector length)
  /// \return ValueType_
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto norm() const -> ValueType_;

  /// \brief In-place normalization to unit length vector
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  void normalize();

  /// \brief Create new vector normalized to unit length vector
  /// \return Vector<ValueType_, Size_>
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto normalize() const -> Vector;
  // <---

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) const -> ValueType_ { return Matrix::at_unsafe(idx, 0); }

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) -> ValueType_& { return Matrix::at_unsafe(idx, 0); }
  // <---

protected:
  using Matrix::Ones;
  using Matrix::Zeros;
  using Matrix::operator();
  using Matrix::at_unsafe;
};

#if 0
template <typename ValueType_, sint32 Size_>
auto operator*(const Matrix<ValueType_, Size_, 1, true>& m, const Vector<ValueType_, Size_>& v) -> ValueType_
{
  return v * Vector<ValueType_, Size_>{m};
}
#endif

template <typename ValueType_>
class Vector<ValueType_, 1>: public Matrix<ValueType_, 1, 1, true> // LCOV_EXCL_LINE
{
public:
  /// \brief L2 norm (aka vector length)
  /// \return ValueType_
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto norm() const -> ValueType_;
};

} // namespace math
} // namespace tracking

#endif // CE10BDD8_6874_4771_89BA_D153438C3E01

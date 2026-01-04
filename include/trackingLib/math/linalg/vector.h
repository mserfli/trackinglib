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

/// \brief A column vector class template, specializing Matrix for single-column matrices.
///
/// The Vector class provides a convenient interface for column vectors, inheriting all
/// matrix operations from Matrix while adding vector-specific operations like dot product,
/// norm calculations, and normalization.
///
/// \tparam ValueType_ The data type of vector elements (e.g., float32, float64)
/// \tparam Size_ The number of elements in the vector (must be > 0)
///
/// \note This class is a specialization of Matrix<ValueType_, Size_, 1, true> where
/// the third template parameter (Cols) is fixed to 1 and IsRowMajor is fixed to true.
/// All matrix operations are inherited from the base Matrix class.
///
/// \see Matrix for the base class functionality
template <typename ValueType_, sint32 Size_>
class Vector: public Matrix<ValueType_, Size_, 1, true>
{
public:
  using BaseMatrix = Matrix<ValueType_, Size_, 1, true>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using BaseMatrix::BaseMatrix;

  //////////////////////////////////////////////////
  // additional constructors  --->
  /// \brief Construct a vector from a compatible matrix
  /// \param[in] other A matrix with dimensions Size_ x 1
  explicit Vector(const BaseMatrix& other)
      : BaseMatrix{other}
  {
  }

  /// \brief Move construct a vector from a compatible matrix
  /// \param[in] other A matrix with dimensions Size_ x 1 (moved from)
  explicit Vector(BaseMatrix&& other) noexcept
      : BaseMatrix{std::move(other)}
  {
  }

  /// \brief Construct a zero vector
  /// \return A vector filled with zeros
  [[nodiscard]] static auto Zeros() -> Vector;

  /// \brief Construct a vector filled with ones
  /// \return A vector filled with ones
  [[nodiscard]] static auto Ones() -> Vector;

  /// \brief Create a unit vector with 1 at the specified position
  /// \tparam Row_ The index position (0-based) where the 1 should be placed
  /// \return A unit vector with 1 at position Row_ and 0 elsewhere
  template <sint32 Row_>
  [[nodiscard]] static auto UnitVector() -> Vector;
  // <---

  //////////////////////////////////////////////////
  // access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx Row index of the element
  /// \return The scalar vector value wrapped in tl::expected for error handling
  /// \note Performs bounds checking
  [[nodiscard]] auto operator[](sint32 idx) const -> tl::expected<ValueType_, Errors>;

  /// \brief Element access to a scalar vector value
  /// \param[in] idx Row index of the element
  /// \return Reference to the scalar vector value wrapped in tl::expected for error handling
  /// \note Performs bounds checking
  [[nodiscard]] auto operator[](sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>;
  // <---

  //////////////////////////////////////////////////
  // arithmentic operators --->
  /// \brief Compute the dot product with another vector
  /// \param[in] other The other vector (must have the same size)
  /// \return The dot product value
  /// \note Mathematically: \f$ \mathbf{v} \cdot \mathbf{w} = \sum_{i=0}^{n-1} v_i w_i \f$
  [[nodiscard]] auto operator*(const Vector& other) const -> ValueType_;
  // <---

  //////////////////////////////////////////////////
  // other operations --->
  /// \brief Compute the squared Euclidean norm (L2 norm squared) of the vector
  /// \return The squared L2 norm value
  /// \note Mathematically: \f$ \|\mathbf{v}\|_2^2 = \sum_{i=0}^{n-1} v_i^2 \f$
  [[nodiscard]] auto normSq() const -> ValueType_;

  /// \brief Compute the Euclidean norm (L2 norm) of the vector
  /// \return The L2 norm value
  /// \note Only available for floating-point ValueType_
  /// \note Mathematically: \f$ \|\mathbf{v}\|_2 = \sqrt{\sum_{i=0}^{n-1} v_i^2} \f$
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto norm() const -> ValueType_;

  /// \brief Normalize this vector in-place to unit length
  /// \note Only available for floating-point ValueType_
  /// \note Modifies this vector in-place
  /// \warning Undefined behavior if the vector norm is zero
  /// \note Mathematically: \f$ \mathbf{v} \leftarrow \frac{\mathbf{v}}{\|\mathbf{v}\|_2} \f$
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  void normalize();

  /// \brief Create a new normalized vector of unit length
  /// \return A new vector normalized to unit length
  /// \note Only available for floating-point ValueType_
  /// \note Returns a new normalized vector, leaving this vector unchanged
  /// \warning Undefined behavior if the vector norm is zero
  /// \note Mathematically: \f$ \mathbf{w} = \frac{\mathbf{v}}{\|\mathbf{v}\|_2} \f$
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto normalize() const -> Vector;
  // <---

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar vector value (unchecked)
  /// \param[in] idx Row index of the element (must be valid)
  /// \return The scalar vector value
  /// \note No bounds checking performed - use with caution
  [[nodiscard]] auto at_unsafe(sint32 idx) const -> ValueType_ { return BaseMatrix::at_unsafe(idx, 0); }

  /// \brief Element access to a scalar vector value (unchecked)
  /// \param[in] idx Row index of the element (must be valid)
  /// \return Reference to the scalar vector value
  /// \note No bounds checking performed - use with caution
  [[nodiscard]] auto at_unsafe(sint32 idx) -> ValueType_& { return BaseMatrix::at_unsafe(idx, 0); }
  // <---

protected:
  using BaseMatrix::Ones;
  using BaseMatrix::Zeros;
  using BaseMatrix::operator();
  using BaseMatrix::at_unsafe;
};

/// \brief Specialization of Vector for single-element vectors.
///
/// This specialization provides optimized operations for 1-element vectors,
/// which are essentially scalars in vector form.
///
template <typename ValueType_>
class Vector<ValueType_, 1>: public Matrix<ValueType_, 1, 1, true>
{
public:
  /// \brief Compute the Euclidean norm (L2 norm) of the single-element vector
  /// \return The L2 norm value (absolute value of the single element)
  /// \note Specialization for 1-element vectors
  /// \note Only available for floating-point ValueType_
  template <typename U = ValueType_, std::enable_if_t<std::is_floating_point<U>::value, int> = 0>
  [[nodiscard]] auto norm() const -> ValueType_;
};

} // namespace math
} // namespace tracking

#endif // CE10BDD8_6874_4771_89BA_D153438C3E01

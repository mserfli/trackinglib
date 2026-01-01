#ifndef EDCA948E_6A98_4AF3_8A01_916736E1577B
#define EDCA948E_6A98_4AF3_8A01_916736E1577B

#include "base/first_include.h" // IWYU pragma: keep
#include "math/linalg/errors.h"

namespace tracking
{
namespace math
{

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
class Matrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_, bool IsLower_, bool IsRowMajor_>
class TriangularMatrix; // LCOV_EXCL_LINE

template <typename ValueType_, sint32 Size_>
class Vector; // LCOV_EXCL_LINE

// TODO(matthias): add interface contract
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix TEST_REMOVE_FINAL // LCOV_EXCL_LINE
{
public:
  // rule of 5 declarations
  DiagonalMatrix()                                             = default;
  DiagonalMatrix(const DiagonalMatrix& other)                  = default;
  DiagonalMatrix(DiagonalMatrix&&) noexcept                    = default;
  auto operator=(const DiagonalMatrix&) -> DiagonalMatrix&     = default;
  auto operator=(DiagonalMatrix&&) noexcept -> DiagonalMatrix& = default;

  /// \brief Construct an Identity matrix
  /// \return DiagonalMatrix  Resulting identity matrix
  [[nodiscard]] static auto Identity() -> DiagonalMatrix;

  /// \brief Construct a new Diagonal Matrix object
  /// \tparam IsRowMajor_
  /// \param[in] other
  template <bool IsRowMajor_>
  static auto FromMatrix(const SquareMatrix<ValueType_, Size_, IsRowMajor_>& other) -> DiagonalMatrix;

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  static auto FromList(const std::initializer_list<ValueType_>& list) -> DiagonalMatrix;

  /// \brief Construct a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  static auto FromList(const std::initializer_list<std::initializer_list<ValueType_>>& list) -> DiagonalMatrix;

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity();


  /// \brief Set a diagonal block matrix at given position
  /// \tparam SrcSize_    Size_ of the source block
  /// \tparam SrcCount   Number of diagonal elements to copy from source
  /// \tparam SrcIdxBeg  Begin diagonal element index in source
  /// \tparam DstIdxBeg  Begin diagonal element index in dest
  /// \param[in] block   Source block matrix to copy from
  template <sint32 SrcSize_, sint32 SrcCount_, sint32 SrcIdxBeg_, sint32 DstIdxBeg_>
  void setBlock(const DiagonalMatrix<ValueType_, SrcSize_>& block);

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing the diagonal elements
  [[nodiscard]] auto operator=(const std::initializer_list<ValueType_>& list)
      -> DiagonalMatrix&; // TODO(matthias): do we really need this

  /// \brief Assign a new DiagonalMatrix object given initializer list
  /// \param[in] list  An initializer list describing a full square matrix
  [[nodiscard]] auto operator=(const std::initializer_list<std::initializer_list<ValueType_>>& list)
      -> DiagonalMatrix&; // TODO(matthias): do we really need this

  /// \brief Multiplication with generic matrix: D * Matrix
  /// \tparam Cols_
  /// \tparam IsRowMajor_
  /// \param[in] mat
  /// \return Matrix<ValueType_, Size_, Cols_, IsRowMajor_>
  template <sint32 Cols_, bool IsRowMajor_>
  [[nodiscard]] auto operator*(const Matrix<ValueType_, Size_, Cols_, IsRowMajor_>& mat) const
      -> Matrix<ValueType_, Size_, Cols_, IsRowMajor_>;

  /// \brief Multiplication with triangular matrix: D * Matrix
  /// \tparam isLower_
  /// \tparam isRowMajor_
  /// \param[in] mat  A triangular matrix
  /// \return TriangularMatrix<ValueType_, Size_, isLower_, isRowMajor_>
  template <bool IsLower_, bool IsRowMajor_>
  [[nodiscard]] auto operator*(const TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>& mat) const
      -> TriangularMatrix<ValueType_, Size_, IsLower_, IsRowMajor_>;

  /// \brief Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  /// \return DiagonalMatrix<ValueType_, Size_>
  [[nodiscard]] auto operator*(const DiagonalMatrix& mat) const -> DiagonalMatrix;

  /// \brief Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  /// \return DiagonalMatrix<ValueType_, Size_>
  [[nodiscard]] auto operator*(const ValueType_ scalar) const -> DiagonalMatrix;

  /// \brief Inplace Multiplication with diagonal matrix: D * Matrix
  /// \param[in] mat  A diagonal matrix
  void operator*=(const DiagonalMatrix& mat);

  /// \brief Inplace Multiplication with scalar: D * scalar
  /// \param[in] scalar  A scalar value
  void operator*=(const ValueType_ scalar);

  /// \brief Element access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return tl::expected<std::reference_wrapper<ValueType_>, Errors>   either the reference at (idx) or an Error descriptor
  [[nodiscard]] auto operator[](const sint32 idx) -> tl::expected<std::reference_wrapper<ValueType_>, Errors>
  { // implemented here to solve cyclic includes
    if (!(idx >= 0 && idx < Size_))
    {
      return tl::unexpected<Errors>{Errors::invalid_access_idx};
    }
    return at_unsafe(idx);
  }

  /// \brief Element read-only access to a scalar diagonal value
  /// \param[in] idx  Row/Col index of the element
  /// \return tl::expected<ValueType_, Errors>   either the value at (idx) or an Error descriptor
  [[nodiscard]] auto operator[](const sint32 idx) const -> tl::expected<ValueType_, Errors>
  { // implemented here to solve cyclic includes
    if (!(idx >= 0 && idx < Size_))
    {
      return tl::unexpected<Errors>{Errors::invalid_access_idx};
    }
    return at_unsafe(idx);
  }

  /// \brief Calculates the inverse
  /// \return DiagonalMatrix<ValueType_, Size_>
  [[nodiscard]] auto inverse() const -> DiagonalMatrix;

  /// \brief Calculates the inverse inplace
  void inverse();

  /// \brief Checks for positive definite condition of the diagonal matrix (all elements > 0)
  [[nodiscard]] auto isPositiveDefinite() const -> bool;

  /// \brief Print the matrix
  void print() const;

  //////////////////////////////////////////////////
  // unsafe access operators  --->
  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_  Scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) const -> ValueType_ { return _data.at_unsafe(idx); }

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return ValueType_&  Reference to the scalar vector value
  [[nodiscard]] auto at_unsafe(sint32 idx) -> ValueType_& { return _data.at_unsafe(idx); }
  // <---

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  Vector<ValueType_, Size_> _data{};
};

template <typename ValueType_, sint32 Rows_, sint32 Cols_, bool IsRowMajor_>
[[nodiscard]] auto operator*(const Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>& mat,
                             const DiagonalMatrix<ValueType_, Cols_>& diag) -> Matrix<ValueType_, Rows_, Cols_, IsRowMajor_>;

} // namespace math
} // namespace tracking

#endif // EDCA948E_6A98_4AF3_8A01_916736E1577B

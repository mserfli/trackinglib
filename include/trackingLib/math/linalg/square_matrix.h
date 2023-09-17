#ifndef BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08
#define BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "base/first_include.h"
#include "math/linalg/errors.h"
#include "math/linalg/matrix.h"
#include <limits>
#include <utility>

namespace tracking
{
namespace math
{

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_, bool IsLower_>
class TriangularMatrix;

// forward declaration to prevent cyclic includes
template <typename ValueType_, sint32 Size_>
class DiagonalMatrix;

// TODO(matthias): add interface contract
template <typename ValueType_, sint32 Size_, bool IsRowMajor_>
class SquareMatrix: public Matrix<ValueType_, Size_, Size_, IsRowMajor_>
{
public:
  using Matrix = Matrix<ValueType_, Size_, Size_, IsRowMajor_>; ///< type of the parent class

  // unhide ctor of base class to allow implicit call in derived default ctors
  using Matrix::Matrix;

  //////////////////////////////////////////////////
  // additional constructors  --->
    /// \brief Construct a new Square Matrix< Float Type,  Size_> object
  /// \param[in] other A base class object
  explicit SquareMatrix(const Matrix& other) : Matrix{other} {}

  /// \brief Construct a new Square Matrix object
  /// \param[in] other A diagonal matrix
  SquareMatrix(const DiagonalMatrix<ValueType_, Size_>& other); // NOLINT(google-explicit-constructor)

  /// \brief Construct a new Square Matrix object with given initializer list representing the memory layout of the matrix
  /// \param[in] list  An initializer list describing the memory layout of the matrix
  static auto FromList(const std::initializer_list<std::initializer_list<ValueType_>>& list) -> SquareMatrix;

  /// \brief Set internal matrix to the Identity matrix
  void setIdentity();

  /// \brief Construct an Identity matrix
  /// \return SquareMatrix  Resulting identity matrix
  static auto Identity() -> SquareMatrix;

  auto householderQR() const -> std::pair<SquareMatrix, SquareMatrix>;

  /// \brief Solve A * x = b using QR decomposition 
  /// \param[in]  b  Result vector/matrix of A * x
  /// \return SquareMatrix x
  auto qrSolve(const SquareMatrix& b) const -> SquareMatrix;

#if 0
  /// \brief Decompose internal matrix into L*L' using standard Cholesky factorization
  /// \return tl::expected<TriangularMatrix<ValueType_, Size_, true>, Errors> 
  /// \precondition internal matrix is symmetric and positive definite
  auto decomposeLLT() const -> tl::expected<TriangularMatrix<ValueType_, Size_, true>, Errors>;

  /// \brief Decompose internal matrix into L*D*L' using rational Cholesky factorization
  /// \return tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, true>, DiagonalMatrix<ValueType_, Size_>>, Errors> 
  /// \precondition internal matrix is symmetric and positive semi definite
  auto decomposeLDLT() const -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, true>, DiagonalMatrix<ValueType_, Size_>>, Errors>;
#endif
  /// \brief Decompose internal matrix into U*D*U' using rational Cholesky factorization
  /// \return tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, false>, DiagonalMatrix<ValueType_, Size_>>, Errors> 
  /// \precondition internal matrix is symmetric and positive semi definite
  auto decomposeUDUT() const -> tl::expected<std::pair<TriangularMatrix<ValueType_, Size_, false>, DiagonalMatrix<ValueType_, Size_>>, Errors>;

  /// \brief Calculates the inverse based on Cholesky factorization
  /// \return SquareMatrix
  auto inverse() const -> SquareMatrix;

  // clang-format off
TEST_REMOVE_PROTECTED:
  ; // workaround for correct indentation
  // clang-format on

  /// \brief Check for symmetry of the matrix
  /// \return true
  [[nodiscard]] auto isSymmetric() const -> bool;
};

} // namespace math
} // namespace tracking
#endif // BE2AA4BF_8A4F_4A6D_8DFB_4E140C9F0A08

#include "base/first_include.h"
#include "math/linalg/covariance_matrix_full.h"
#include "math/linalg/diagonal_matrix.h"
#include "math/linalg/matrix.h"
#include "math/linalg/square_matrix.h"
#include "math/linalg/triangular_matrix.h"
#include "math/linalg/vector.h"
#include "math/linalg/point2d.h"

using tracking::math::Matrix;
using tracking::math::SquareMatrix;
using tracking::math::TriangularMatrix;
using tracking::math::DiagonalMatrix;
using tracking::math::CovarianceMatrixFull;
using tracking::math::Vector;
using tracking::math::Point2d;
auto main() -> int
{
  {
    Matrix<float32, 3, 1> m1{};
    Matrix<float32, 3, 1> m2{m1};
    Matrix<float32, 3, 1> m3{std::move(m2)};
    m2 = m1;
    m3 = std::move(m2);

    Vector<float32, 3> v1{};
    Vector<float32, 3> v2{v1};
    Vector<float32, 3> v3{std::move(v2)};
    v2 = v1;
    v3 = std::move(v2);

    Point2d<float32> p1{};
    Point2d<float32> p2{p1};
    Point2d<float32> p3{std::move(p2)};
    p2 = p1;
    p3 = std::move(p2);
  }

  {
    Vector<float32, 3> v1{};
    Matrix<float32, 3, 1> m1{v1};
  }

  {
    Matrix<float32, 2, 1> m1{};
    Vector<float32, 2> v1{m1};
    Point2d<float32> p1{};
    Point2d<float32> p2{m1};
    Point2d<float32> p3{v1};
  }

  {
    Vector<float32, 2> v1{};
    Vector<float32, 2> v2{};
    Matrix<float32, 2, 2> m1{};
    Vector<float32, 2> v3 = v1 - v2;
    float32 a = v3 * v2;
    v3 = m1 * v2;
  }

  {
    SquareMatrix<float32, 3> m1{{1.8547, 1.3984, 1.2923}, {1.3984, 1.2222, 1.2328}, {1.2923, 1.2328, 1.3428}};
    TriangularMatrix<float32, 3, true> L{};
    DiagonalMatrix<float32, 3> D{};
    m1.print();
    m1.decomposeLDLT(L,D);
    L.print();
    D.print();
    m1=L*D*L.transpose();
    m1.print();
  }

  {
    SquareMatrix<float32, 3> m1{{1.8547, 1.3984, 1.2923}, {1.3984, 1.2222, 1.2328}, {1.2923, 1.2328, 1.3428}};
    TriangularMatrix<float32, 3, false> U{};
    DiagonalMatrix<float32, 3> D{};
    m1.print();
    m1.decomposeUDUT(U,D);
    U.print();
    D.print();
    m1=U*D*U.transpose();
    m1.print();
  }

  {
    CovarianceMatrixFull<float64, 3> cov{{1.8547, 1.3984, 1.2923}, {1.3984, 1.2222, 1.2328}, {1.2923, 1.2328, 1.3428}};
    CovarianceMatrixFull<float64, 3> inv = cov.inverse();
    inv.print();
  }
  return 0;
}
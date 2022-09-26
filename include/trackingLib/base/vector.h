#ifndef CE10BDD8_6874_4771_89BA_D153438C3E01
#define CE10BDD8_6874_4771_89BA_D153438C3E01

#include "base/matrix.h"

namespace tracking
{
namespace base
{

template <typename FloatType, sint32 Size>
class Vector: public Matrix<FloatType, Size, 1>
{
public:
  /// \brief Inherit Rule of 5 behavior from base class
  using Matrix<FloatType, Size, 1>::Matrix;

  /// \brief Construct a new Vector object
  /// \param[in] other A base class object
  Vector(const Matrix<FloatType, Size, 1>& other); // NOLINT(google-explicit-constructor)

  /// \brief Element read-only access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return FloatType  Scalar vector value
  auto operator[](sint32 idx) const -> FloatType;

  /// \brief Element access to a scalar vector value
  /// \param[in] idx  Row index of the element
  /// \return FloatType&  Reference to the scalar vector value
  auto operator[](sint32 idx) -> FloatType&;

  /// \brief Dot product with other vector of same size
  /// \param[in,out] other
  /// \return FloatType 
  auto operator*(const Vector<FloatType, Size>& other) const -> FloatType;

  /// \brief Squared L1 norm (aka squared vector length) 
  /// \return FloatType 
  auto normSq() const -> FloatType;

  /// \brief L1 norm (aka vector length) 
  /// \return FloatType 
  auto norm() const -> FloatType;

private:
  /// \brief hide inherited operator() to prevent col!=0 access
  using Matrix<FloatType, Size, 1>::operator();
};

template <typename FloatType, sint32 Size>
Vector<FloatType,Size>::Vector(const Matrix<FloatType, Size, 1>& other)
    : Matrix<FloatType, Size, 1>{other}
{
}

template <typename FloatType, sint32 Size>
inline auto Vector<FloatType,Size>::operator[](sint32 idx) const -> FloatType
{
  return this->operator()(idx, 0);
}

template <typename FloatType, sint32 Size>
inline auto Vector<FloatType,Size>::operator[](sint32 idx) -> FloatType&
{
  return this->operator()(idx, 0);
}

template <typename FloatType, sint32 Size>
inline auto Vector<FloatType,Size>::operator*(const Vector<FloatType, Size>& other) const -> FloatType
{
  const Matrix<FloatType, 1, 1> temp{this->transpose() * other};
  return temp(0, 0);
}

template <typename FloatType, sint32 Size>
inline auto Vector<FloatType,Size>::normSq() const -> FloatType
{
  return this->operator*(*this);
}

template <typename FloatType, sint32 Size>
inline auto Vector<FloatType,Size>::norm() const -> FloatType
{
  return std::sqrt(normSq());
}

} // namespace base
} // namespace tracking

#endif // CE10BDD8_6874_4771_89BA_D153438C3E01

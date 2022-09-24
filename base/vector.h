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
  Vector(const Matrix<FloatType, Size, 1>& other) // NOLINT(google-explicit-constructor)
      : Matrix<FloatType, Size, 1>{other}
  {
  }

  auto operator[](sint32 idx) const -> FloatType { return this->operator()(idx, 1); }

  auto operator[](sint32 idx) -> FloatType& { return this->operator()(idx, 1); }

  auto operator*(const Vector<FloatType, Size>& other) const -> FloatType
  {
    const Matrix<FloatType, 1, 1> temp{this->transpose() * other};
    return temp(0, 0);
  }

  auto normSq() const -> FloatType { return this->operator*(*this); }

  auto norm() const -> FloatType { return std::sqrt(normSq()); }

private:
  using Matrix<FloatType, Size, 1>::operator();
};

} // namespace base
} // namespace tracking

#endif // CE10BDD8_6874_4771_89BA_D153438C3E01

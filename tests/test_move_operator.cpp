#include <array>
#include <chrono>
#include <iostream>

static constexpr int N = 600000;
struct ComplexType
{
  ComplexType() = default; // { std::cout << "ComplexType()" << std::endl; }

  ComplexType(const ComplexType& other)
      : a{other.a}
      , vec{other.vec}
  {
    // std::cout << "ComplexType(const ComplexType& other)" << std::endl;
  }

  ComplexType(ComplexType&& other) noexcept
      : a{0}
      , vec{}
  {
    // std::cout << "ComplexType(ComplexType&& other)" << std::endl;

    *this = std::move(other);
  }

  auto operator=(const ComplexType& other) -> ComplexType&
  {
    // std::cout << "operator=(const ComplexType& other)" << std::endl;
    if (this != &other)
    {
      a = other.a;
      vec = other.vec;
    }
    return *this;
  }

  auto operator=(ComplexType&& other) noexcept -> ComplexType&
  {
    // std::cout << "operator=(ComplexType&& other)" << std::endl;

    if (this != &other)
    {
      // Free the existing resource.

      // Copy the data pointer and its length from the
      // source object.
      a = other.a;
      vec = other.vec;

      // Release the data pointer from the source object so that
      // the destructor does not free the memory multiple times.
      other.a = 0;
      other.vec = {};
    }
    return *this;
  }

  int                  a{};
  std::array<float, N> vec;
};

struct ComplexTypeWithPtr
{
  using ArrayType = std::array<float, N>;

  ComplexTypeWithPtr() = default; //{ std::cout << "ComplexTypeWithPtr()" << std::endl; }

  ComplexTypeWithPtr(const ComplexTypeWithPtr& other)
  {
    // std::cout << "ComplexTypeWithPtr(const ComplexTypeWithPtr& other)" << std::endl;

    *this = other;
  }

  ComplexTypeWithPtr(ComplexTypeWithPtr&& other) noexcept
  {
    // std::cout << "ComplexTypeWithPtr(ComplexTypeWithPtr&& other)" << std::endl;

    *this = std::move(other);
  }

  auto operator=(const ComplexTypeWithPtr& other) -> ComplexTypeWithPtr&
  {
    // std::cout << "operator=(const ComplexTypeWithPtr& other)" << std::endl;
    if (this != &other)
    {
      a = other.a;
      vec.reset(new ArrayType(*other.vec));
    }
    return *this;
  }

  auto operator=(ComplexTypeWithPtr&& other) noexcept -> ComplexTypeWithPtr&
  {
    // std::cout << "operator=(ComplexTypeWithPtr&& other)" << std::endl;

    if (this != &other)
    {
      // Free the existing resource.
      vec.reset(nullptr);

      // Copy the data pointer and its length from the
      // source object.
      a = other.a;
      vec = std::move(other.vec);

      // Release the data pointer from the source object so that
      // the destructor does not free the memory multiple times.
      other.a = 0;
      other.vec.reset(nullptr);
    }
    return *this;
  }

  int                        a{};
  std::unique_ptr<ArrayType> vec{new ArrayType};
};

template <typename T>
class A
{
public:
  auto getData() const -> const T& { return _data; }
  auto getData() -> T&
  {
    // std::cout << "getData()" << std::endl;
    return _data;
  }
  void setData1(const T& other)
  {
    // std::cout << "setData1(const T& other)" << std::endl;
    _data = other;
  }
  void setData2(T&& other)
  {
    // std::cout << "setData2(T&& other)" << std::endl;
    _data = std::move(other);
  }

  // clang-format off
TEST_REMOVE_PRIVATE:
  ; // workaround for correct indentation
  // clang-format on
  T _data{};
};


template <typename T>
void measureRuntime()
{
  A<T> a;
  T    rawData{};
  auto t0 = std::chrono::high_resolution_clock::now();
  for (auto i = 0; i < 10000; ++i)
  {
    a.getData() = rawData;
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  for (auto i = 0; i < 10000; ++i)
  {
    a.setData1(rawData);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  for (auto i = 0; i < 10000; ++i)
  {
    a.setData2(std::move(rawData));
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  auto ms01_int = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  auto ms21_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  auto ms32_int = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2);
  std::cout << ms01_int.count() << ", " << ms21_int.count() << ", " << ms32_int.count() << std::endl;
}

auto main() -> int
{
  measureRuntime<ComplexType>();
  measureRuntime<ComplexTypeWithPtr>();

  return 0;
}

#ifndef LCC_UTILITIES_HPP
#define LCC_UTILITIES_HPP

#define LCC_DISABLE_SIGN_CONVERSION \
    _Pragma("clang diagnostic push") \
    _Pragma("GCC diagnostic push") \
    _Pragma("clang diagnostic ignored \"-Wsign-conversion\"") \
    _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")

#define LCC_END_DISABLE_WARNINGS _Pragma("clang diagnostic pop") \
    _Pragma("GCC diagnostic pop")

namespace legged_ctrl {

void not_implemented();

template <typename Tag, typename T>
struct Tagged
{
  explicit Tagged(const T& value) : value{value} { }
  T value;
};

}

#endif

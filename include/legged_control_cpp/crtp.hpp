#ifndef LCC_CRTP_HPP
#define LCC_CRTP_HPP

namespace legged_ctrl {

template<typename T> class Crtp
{
protected:
  T &derived() { return static_cast<T &>(*this); }
  T const &derived() const { return static_cast<T const &>(*this); }
};

}// namespace legged_ctrl

#endif// LCC_CRTP_HPP

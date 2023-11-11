#ifndef LCC_LOGGING_HPP
#define LCC_LOGGING_HPP

#include "spdlog/logger.h"
#include "spdlog/spdlog.h"
#include "sstream"

namespace legged_ctrl {

using LoggerPtr = std::shared_ptr<spdlog::logger>;

LoggerPtr null_logger();

template<typename Matrix> std::string matrix_to_str(Matrix const &mat)
{
  // NOLINTNEXTLINE
  std::ostringstream oss;
  oss << mat;
  return oss.str();
}

}// namespace legged_ctrl
#endif

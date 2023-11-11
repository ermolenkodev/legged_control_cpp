#include "legged_control_cpp/logging.hpp"
#include <spdlog/sinks/null_sink.h>

#include <sstream>
#include <utility>

namespace legged_ctrl {

LoggerPtr null_logger()
{
  return std::make_shared<spdlog::logger>("lcc_null", std::make_shared<spdlog::sinks::null_sink_st>());
}

}// namespace legged_ctrl

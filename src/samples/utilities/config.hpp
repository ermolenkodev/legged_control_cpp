#include "legged_control_cpp/type_aliases.hpp"
#include "optional"
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace legged_ctrl;

using json = nlohmann::json;

std::optional<json> read_config(std::string const &path, spdlog::logger &logger) noexcept;

VectorX get_vector(json const &config, std::string const &key);

Eigen::Matrix<double, SIX_DIM, SIX_DIM> get_matrix(json const &config, std::string const &key);

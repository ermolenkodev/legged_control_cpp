#include "config.hpp"

std::optional<json> read_config(std::string const &path, spdlog::logger &logger) noexcept
{
  std::ifstream f(path);
  try {
    return json::parse(f);
  } catch (json::parse_error const &e) {
    logger.error("Failed to parse config file: %s", e.what());
    return std::nullopt;
  }
}

VectorX get_vector(json const &config, std::string const &key)
{
  std::vector<double> vec = config.at(key);
  return VectorX(Eigen::VectorXd::Map(vec.data(), static_cast<long>(vec.size())));
}

Eigen::Matrix<double, SIX_DIM, SIX_DIM> get_matrix(json const &config, std::string const &key)
{
  std::vector<double> vec = config.at(key);
  return Eigen::Matrix<double, SIX_DIM, SIX_DIM>(
    Eigen::Matrix<double, SIX_DIM, SIX_DIM>::Map(vec.data(), SIX_DIM, SIX_DIM));
}

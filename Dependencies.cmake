include(cmake/CPM.cmake)

# Done as a function so that updates to variables like
# CMAKE_CXX_FLAGS don't propagate out to other
# targets
function(legged_control_cpp_setup_dependencies)

  # For each dependency, see if it's
  # already been provided to us by a parent project

  if(NOT TARGET fmtlib::fmtlib)
    CPMAddPackage("gh:fmtlib/fmt#9.1.0")
  endif()

  if(NOT TARGET spdlog::spdlog)
    CPMAddPackage(
      NAME
      spdlog
      VERSION
      1.9.2
      GITHUB_REPOSITORY
      "gabime/spdlog"
      OPTIONS
      "SPDLOG_FMT_EXTERNAL ON")
  endif()

  if(NOT TARGET Catch2::Catch2WithMain)
    CPMAddPackage("gh:catchorg/Catch2@3.3.2")
  endif()

  if(NOT TARGET CLI11::CLI11)
    CPMAddPackage("gh:CLIUtils/CLI11@2.3.2")
  endif()

  if(NOT TARGET tools::tools)
    CPMAddPackage("gh:lefticus/tools#update_build_system")
  endif()

  if(NOT TARGET Eigen3::Eigen)
    CPMAddPackage(
            NAME Eigen
            VERSION 3.4.0
            URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
            # Eigen's CMakelists are not intended for library use
            DOWNLOAD_ONLY YES
    )

    if(Eigen_ADDED)
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
      target_include_directories(Eigen3::Eigen INTERFACE ${Eigen_SOURCE_DIR})
    endif()
  endif()

  if (NOT TARGET nlohmann_json::nlohmann_json)
    CPMAddPackage(
            NAME nlohmann_json
            GITHUB_REPOSITORY nlohmann/json
            VERSION 3.9.1
            OPTIONS
            "JSON_BuildTests OFF")
  endif()
endfunction()

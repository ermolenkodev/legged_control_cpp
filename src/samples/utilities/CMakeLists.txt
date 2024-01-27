include(GenerateExportHeader)

add_library(utilities STATIC
        config.cpp
        trajectory_io.cpp
)

target_include_directories(utilities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_system_libraries(
        utilities
        PRIVATE
        spdlog::spdlog
        nlohmann_json::nlohmann_json)

target_link_libraries(
        utilities
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib)

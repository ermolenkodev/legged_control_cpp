include(GenerateExportHeader)

add_executable(urdf_sample urdf_sample.cpp)

target_link_system_libraries(
        urdf_sample
        PRIVATE
        spdlog::spdlog)

target_link_libraries(
        urdf_sample
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib)

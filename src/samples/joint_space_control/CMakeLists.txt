include(GenerateExportHeader)

add_executable(joint_space_control joint_space_control.cpp)

target_link_system_libraries(
        joint_space_control
        PRIVATE
        spdlog::spdlog
        nlohmann_json::nlohmann_json)

target_link_libraries(
        joint_space_control
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib)

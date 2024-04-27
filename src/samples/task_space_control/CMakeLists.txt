include(GenerateExportHeader)

add_executable(task_space_control EXCLUDE_FROM_ALL mujoco_api.cpp task_space_control.cpp)

target_compile_definitions(task_space_control PUBLIC CMAKE_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../assets")

target_include_directories(
        task_space_control
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)

if (NOT TARGET mujoco::mujoco)
    CPMAddPackage(
            NAME mujoco
            GIT_TAG 3.1.2
            GITHUB_REPOSITORY "google-deepmind/mujoco"
            OPTIONS
            "MUJOCO_BUILD_EXAMPLES OFF"
            "MUJOCO_BUILD_SIMULATE ON"
            "MUJOCO_BUILD_TESTS OFF"
            "MUJOCO_TEST_PYTHON_UTIL OFF"
            "SDFLIB_USE_SYSTEM_SPDLOG ON"
            "SIMULATE_BUILD_EXECUTABLE OFF")
endif ()

target_link_system_libraries(
        task_space_control
        PUBLIC
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        mujoco::mujoco
        mujoco::libsimulate
        glfw
        Threads::Threads
        lodepng
)

target_link_libraries(
        task_space_control
        PRIVATE
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        utilities
)

cmake_minimum_required(VERSION 3.15...3.23)

project(CmakeConfigPackageTests LANGUAGES CXX)

# ---- Test as standalone project the exported config package ----

if (PROJECT_IS_TOP_LEVEL OR TEST_INSTALLED_VERSION)
    enable_testing()

    find_package(legged_control_cpp CONFIG REQUIRED) # for intro, project_options, ...

    if (NOT TARGET legged_control_cpp_options)
        message(FATAL_ERROR "Requiered config package not found!")
        return() # be strictly paranoid for Template Janitor github action! CK
    endif ()
endif ()

# ---- Dependencies ----

include(${Catch2_SOURCE_DIR}/extras/Catch.cmake)

# Add a file containing a set of constexpr tests
add_executable(constexpr_tests constexpr_tests.cpp
        forward_kinematics_tests.cpp)
target_link_system_libraries(
        constexpr_tests
        PRIVATE
        spdlog::spdlog)
target_link_libraries(
        constexpr_tests
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        Catch2::Catch2WithMain)

catch_discover_tests(
        constexpr_tests
        TEST_PREFIX
        "constexpr."
        REPORTER
        XML
        OUTPUT_DIR
        .
        OUTPUT_PREFIX
        "constexpr."
        OUTPUT_SUFFIX
        .xml)

# Disable the constexpr portion of the test, and build again this allows us to have an executable that we can debug when
# things go wrong with the constexpr testing
add_executable(relaxed_constexpr_tests constexpr_tests.cpp)
target_link_system_libraries(
        relaxed_constexpr_tests
        PRIVATE
        spdlog::spdlog)
target_link_libraries(
        relaxed_constexpr_tests
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        Catch2::Catch2WithMain)
target_compile_definitions(relaxed_constexpr_tests PRIVATE -DCATCH_CONFIG_RUNTIME_STATIC_REQUIRE)

catch_discover_tests(
        relaxed_constexpr_tests
        TEST_PREFIX
        "relaxed_constexpr."
        REPORTER
        XML
        OUTPUT_DIR
        .
        OUTPUT_PREFIX
        "relaxed_constexpr."
        OUTPUT_SUFFIX
        .xml)

add_executable(inverse_dynamics_tests inverse_dynamics_tests.cpp)
target_link_system_libraries(
        inverse_dynamics_tests
        PRIVATE
        spdlog::spdlog)
target_link_libraries(
        inverse_dynamics_tests
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        Catch2::Catch2WithMain)

target_include_directories(
        inverse_dynamics_tests
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test_data)

target_compile_definitions(inverse_dynamics_tests PUBLIC CMAKE_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../assets")

catch_discover_tests(
        inverse_dynamics_tests
        TEST_PREFIX
        "inverse_dynamics."
        REPORTER
        XML
        OUTPUT_DIR
        .
        OUTPUT_PREFIX
        "inverse_dynamics."
        OUTPUT_SUFFIX
        .xml)

add_executable(mjxml_parsing_tests mjxml_parsing_tests.cpp)
target_link_system_libraries(
        mjxml_parsing_tests
        PRIVATE
        spdlog::spdlog)

target_link_libraries(
        mjxml_parsing_tests
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        Catch2::Catch2WithMain)

target_include_directories(
        mjxml_parsing_tests
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test_data)

target_compile_definitions(mjxml_parsing_tests PUBLIC CMAKE_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../assets")

catch_discover_tests(
        mjxml_parsing_tests
        TEST_PREFIX
        "mjxml_parsing."
        REPORTER
        XML
        OUTPUT_DIR
        .
        OUTPUT_PREFIX
        "mjxml_parsing."
        OUTPUT_SUFFIX
        .xml)


add_executable(forward_kinematics_tests forward_kinematics_tests.cpp)
target_link_system_libraries(
        forward_kinematics_tests
        PRIVATE
        spdlog::spdlog)

target_link_libraries(
        forward_kinematics_tests
        PRIVATE legged_control_cpp::legged_control_cpp_warnings
        legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_lib
        Catch2::Catch2WithMain)

target_include_directories(
        forward_kinematics_tests
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test_data)

target_compile_definitions(forward_kinematics_tests PUBLIC CMAKE_ASSETS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../assets")

catch_discover_tests(
        forward_kinematics_tests
        TEST_PREFIX
        "forward_kinematics."
        REPORTER
        XML
        OUTPUT_DIR
        .
        OUTPUT_PREFIX
        "forward_kinematics."
        OUTPUT_SUFFIX
        .xml)
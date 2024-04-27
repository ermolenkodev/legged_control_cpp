include(GenerateExportHeader)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

find_package(TinyXML2 REQUIRED)
find_package(urdfdom REQUIRED)

add_library(legged_control_lib STATIC builder.cpp model.cpp urdf.cpp spatial.cpp inverse_dynamics.cpp logging.cpp external_forces.cpp
        forward_dynamics.cpp jacobian.cpp mjxml/mjxml.cpp mjxml/mjxml_defaults.cpp mjxml/mjxml_utils.cpp mjxml/mjxml_builder.cpp forward_kinematics.cpp utilities.cpp)

add_library(legged_control_cpp::legged_control_lib ALIAS legged_control_lib)

target_link_libraries(
        legged_control_lib
        PRIVATE legged_control_cpp::legged_control_cpp_options
        legged_control_cpp::legged_control_cpp_warnings)

target_include_directories(legged_control_lib ${WARNING_GUARD} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>)

target_include_directories(legged_control_lib ${WARNING_GUARD} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/mjxml>)

target_include_directories(legged_control_lib ${WARNING_GUARD} SYSTEM PUBLIC
        $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>)

target_include_directories(legged_control_lib SYSTEM PUBLIC ${urdfdom_INCLUDE_DIRS})

target_include_directories(legged_control_lib SYSTEM PUBLIC ${spdlog_INCLUDE_DIRS})

target_include_directories(legged_control_lib SYSTEM PUBLIC ${TinyXML2_INCLUDE_DIRS})

target_link_system_libraries(
        legged_control_lib
        PRIVATE
        CLI11::CLI11
        fmt::fmt
        spdlog::spdlog
        ${urdfdom_LIBRARIES}
        lefticus::tools
        tinyxml2::tinyxml2)

target_include_directories(legged_control_lib PRIVATE "${CMAKE_BINARY_DIR}/configured_files/include")

set_target_properties(
        legged_control_lib
        PROPERTIES VERSION ${PROJECT_VERSION}
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES)

generate_export_header(legged_control_lib EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/include/legged_control_cpp/legged_control_lib_export.hpp)

if (NOT BUILD_SHARED_LIBS)
    target_compile_definitions(legged_control_lib PUBLIC LEGGED_CONTROL_LIB_STATIC_DEFINE)
endif ()

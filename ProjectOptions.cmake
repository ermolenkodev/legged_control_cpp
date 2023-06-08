include(cmake/SystemLink.cmake)
include(cmake/LibFuzzer.cmake)
include(CMakeDependentOption)
include(CheckCXXCompilerFlag)


macro(legged_control_cpp_setup_options)
  option(legged_control_cpp_ENABLE_HARDENING "Enable hardening" ON)
  option(legged_control_cpp_ENABLE_COVERAGE "Enable coverage reporting" OFF)
  cmake_dependent_option(
    legged_control_cpp_ENABLE_GLOBAL_HARDENING
    "Attempt to push hardening options to built dependencies"
    ON
    legged_control_cpp_ENABLE_HARDENING
    OFF)


  if((CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*" OR CMAKE_CXX_COMPILER_ID MATCHES ".*GNU.*") AND NOT WIN32)
    set(SUPPORTS_UBSAN ON)
  else()
    set(SUPPORTS_UBSAN OFF)
  endif()

  if((CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*" OR CMAKE_CXX_COMPILER_ID MATCHES ".*GNU.*") AND WIN32)
    set(SUPPORTS_ASAN OFF)
  else()
    set(SUPPORTS_ASAN ON)
  endif()

  if(NOT PROJECT_IS_TOP_LEVEL OR legged_control_cpp_PACKAGING_MAINTAINER_MODE)
    option(legged_control_cpp_ENABLE_IPO "Enable IPO/LTO" OFF)
    option(legged_control_cpp_WARNINGS_AS_ERRORS "Treat Warnings As Errors" OFF)
    option(legged_control_cpp_ENABLE_USER_LINKER "Enable user-selected linker" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_LEAK "Enable leak sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)
    option(legged_control_cpp_ENABLE_UNITY_BUILD "Enable unity builds" OFF)
    option(legged_control_cpp_ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
    option(legged_control_cpp_ENABLE_CPPCHECK "Enable cpp-check analysis" OFF)
    option(legged_control_cpp_ENABLE_PCH "Enable precompiled headers" OFF)
    option(legged_control_cpp_ENABLE_CACHE "Enable ccache" OFF)
  else()
    option(legged_control_cpp_ENABLE_IPO "Enable IPO/LTO" ON)
    option(legged_control_cpp_WARNINGS_AS_ERRORS "Treat Warnings As Errors" OFF)
    option(legged_control_cpp_ENABLE_USER_LINKER "Enable user-selected linker" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" ${SUPPORTS_ASAN})
    option(legged_control_cpp_ENABLE_SANITIZER_LEAK "Enable leak sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" ${SUPPORTS_UBSAN})
    option(legged_control_cpp_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
    option(legged_control_cpp_ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)
    option(legged_control_cpp_ENABLE_UNITY_BUILD "Enable unity builds" OFF)
    option(legged_control_cpp_ENABLE_CLANG_TIDY "Enable clang-tidy" ON)
    option(legged_control_cpp_ENABLE_CPPCHECK "Enable cpp-check analysis" ON)
    option(legged_control_cpp_ENABLE_PCH "Enable precompiled headers" OFF)
    option(legged_control_cpp_ENABLE_CACHE "Enable ccache" ON)
  endif()

  if(NOT PROJECT_IS_TOP_LEVEL)
    mark_as_advanced(
      legged_control_cpp_ENABLE_IPO
      legged_control_cpp_WARNINGS_AS_ERRORS
      legged_control_cpp_ENABLE_USER_LINKER
      legged_control_cpp_ENABLE_SANITIZER_ADDRESS
      legged_control_cpp_ENABLE_SANITIZER_LEAK
      legged_control_cpp_ENABLE_SANITIZER_UNDEFINED
      legged_control_cpp_ENABLE_SANITIZER_THREAD
      legged_control_cpp_ENABLE_SANITIZER_MEMORY
      legged_control_cpp_ENABLE_UNITY_BUILD
      legged_control_cpp_ENABLE_CLANG_TIDY
      legged_control_cpp_ENABLE_CPPCHECK
      legged_control_cpp_ENABLE_COVERAGE
      legged_control_cpp_ENABLE_PCH
      legged_control_cpp_ENABLE_CACHE)
  endif()
endmacro()

macro(legged_control_cpp_global_options)
  if(legged_control_cpp_ENABLE_IPO)
    include(cmake/InterproceduralOptimization.cmake)
    legged_control_cpp_enable_ipo()
  endif()

  if(legged_control_cpp_ENABLE_HARDENING AND legged_control_cpp_ENABLE_GLOBAL_HARDENING)
    include(cmake/Hardening.cmake)
    set(ENABLE_UBSAN_MINIMAL_RUNTIME NOT legged_control_cpp_ENABLE_SANITIZER_UNDEFINED)
    legged_control_cpp_enable_hardening(legged_control_cpp_options ON ${ENABLE_UBSAN_MINIMAL_RUNTIME})
  endif()
endmacro()

macro(legged_control_cpp_local_options)
  if (PROJECT_IS_TOP_LEVEL)
    include(cmake/StandardProjectSettings.cmake)
  endif()

  add_library(legged_control_cpp_warnings INTERFACE)
  add_library(legged_control_cpp_options INTERFACE)

  include(cmake/CompilerWarnings.cmake)
  legged_control_cpp_set_project_warnings(
    legged_control_cpp_warnings
    ${legged_control_cpp_WARNINGS_AS_ERRORS}
    ""
    ""
    ""
    "")

  if(legged_control_cpp_ENABLE_USER_LINKER)
    include(cmake/Linker.cmake)
    configure_linker(legged_control_cpp_options)
  endif()

  include(cmake/Sanitizers.cmake)
  legged_control_cpp_enable_sanitizers(
    legged_control_cpp_options
    ${legged_control_cpp_ENABLE_SANITIZER_ADDRESS}
    ${legged_control_cpp_ENABLE_SANITIZER_LEAK}
    ${legged_control_cpp_ENABLE_SANITIZER_UNDEFINED}
    ${legged_control_cpp_ENABLE_SANITIZER_THREAD}
    ${legged_control_cpp_ENABLE_SANITIZER_MEMORY})

  set_target_properties(legged_control_cpp_options PROPERTIES UNITY_BUILD ${legged_control_cpp_ENABLE_UNITY_BUILD})

  if(legged_control_cpp_ENABLE_PCH)
    target_precompile_headers(
      legged_control_cpp_options
      INTERFACE
      <vector>
      <string>
      <utility>)
  endif()

  if(legged_control_cpp_ENABLE_CACHE)
    include(cmake/Cache.cmake)
    legged_control_cpp_enable_cache()
  endif()

  include(cmake/StaticAnalyzers.cmake)
  if(legged_control_cpp_ENABLE_CLANG_TIDY)
    legged_control_cpp_enable_clang_tidy(legged_control_cpp_options ${legged_control_cpp_WARNINGS_AS_ERRORS})
  endif()

  if(legged_control_cpp_ENABLE_CPPCHECK)
    legged_control_cpp_enable_cppcheck(${legged_control_cpp_WARNINGS_AS_ERRORS} "" # override cppcheck options
    )
  endif()

  if(legged_control_cpp_ENABLE_COVERAGE)
    include(cmake/Tests.cmake)
    legged_control_cpp_enable_coverage(legged_control_cpp_options)
  endif()

  if(legged_control_cpp_WARNINGS_AS_ERRORS)
    check_cxx_compiler_flag("-Wl,--fatal-warnings" LINKER_FATAL_WARNINGS)
    if(LINKER_FATAL_WARNINGS)
      # This is not working consistently, so disabling for now
      # target_link_options(legged_control_cpp_options INTERFACE -Wl,--fatal-warnings)
    endif()
  endif()

  if(legged_control_cpp_ENABLE_HARDENING AND NOT legged_control_cpp_ENABLE_GLOBAL_HARDENING)
    include(cmake/Hardening.cmake)
    set(ENABLE_UBSAN_MINIMAL_RUNTIME NOT legged_control_cpp_ENABLE_SANITIZER_UNDEFINED)
    legged_control_cpp_enable_hardening(legged_control_cpp_options OFF ${ENABLE_UBSAN_MINIMAL_RUNTIME})
  endif()

endmacro()

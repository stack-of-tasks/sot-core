# Copyright 2010, 2019 JRL, CNRS/AIST, LAAS CNRS See LICENSE file.

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-core)
set(PROJECT_DESCRIPTION "Hierarchical task solver plug-in for dynamic-graph.")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR "sot/core")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_USE_TEMPLATE_CSS YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/sphinx.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(14 ENFORCE)

# Project dependencies
add_project_dependency(dynamic-graph-python 4.0.6 REQUIRED)
add_project_dependency(pinocchio REQUIRED)
add_project_dependency(Boost REQUIRED COMPONENTS regex program_options)
if(BUILD_TESTING)
  add_project_dependency(example-robot-data 3.8.0)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif()

if(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost >= 1.73
  # without dropping support for boost < 1.73
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

string(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})

# Verbosity level
if(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  add_definitions(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
endif(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/abstract-sot-external-interface.hh
    include/${CUSTOM_HEADER_DIR}/additional-functions.hh
    include/${CUSTOM_HEADER_DIR}/api.hh
    include/${CUSTOM_HEADER_DIR}/binary-int-to-uint.hh
    include/${CUSTOM_HEADER_DIR}/binary-op.hh
    include/${CUSTOM_HEADER_DIR}/causal-filter.hh
    include/${CUSTOM_HEADER_DIR}/clamp-workspace.hh
    include/${CUSTOM_HEADER_DIR}/com-freezer.hh
    include/${CUSTOM_HEADER_DIR}/contiifstream.hh
    include/${CUSTOM_HEADER_DIR}/debug.hh
    include/${CUSTOM_HEADER_DIR}/derivator.hh
    include/${CUSTOM_HEADER_DIR}/device.hh
    include/${CUSTOM_HEADER_DIR}/double-constant.hh
    include/${CUSTOM_HEADER_DIR}/event.hh
    include/${CUSTOM_HEADER_DIR}/exception-abstract.hh
    include/${CUSTOM_HEADER_DIR}/exception-dynamic.hh
    include/${CUSTOM_HEADER_DIR}/exception-factory.hh
    include/${CUSTOM_HEADER_DIR}/exception-feature.hh
    include/${CUSTOM_HEADER_DIR}/exception-signal.hh
    include/${CUSTOM_HEADER_DIR}/exception-task.hh
    include/${CUSTOM_HEADER_DIR}/exception-tools.hh
    include/${CUSTOM_HEADER_DIR}/exp-moving-avg.hh
    include/${CUSTOM_HEADER_DIR}/factory.hh
    include/${CUSTOM_HEADER_DIR}/feature-1d.hh
    include/${CUSTOM_HEADER_DIR}/feature-abstract.hh
    include/${CUSTOM_HEADER_DIR}/feature-generic.hh
    include/${CUSTOM_HEADER_DIR}/feature-joint-limits.hh
    include/${CUSTOM_HEADER_DIR}/feature-line-distance.hh
    include/${CUSTOM_HEADER_DIR}/feature-point6d-relative.hh
    include/${CUSTOM_HEADER_DIR}/feature-point6d.hh
    include/${CUSTOM_HEADER_DIR}/feature-pose.hh
    include/${CUSTOM_HEADER_DIR}/feature-posture.hh
    include/${CUSTOM_HEADER_DIR}/feature-task.hh
    include/${CUSTOM_HEADER_DIR}/feature-vector3.hh
    include/${CUSTOM_HEADER_DIR}/feature-visual-point.hh
    include/${CUSTOM_HEADER_DIR}/filter-differentiator.hh
    include/${CUSTOM_HEADER_DIR}/fir-filter.hh
    include/${CUSTOM_HEADER_DIR}/flags.hh
    include/${CUSTOM_HEADER_DIR}/fwd.hh
    include/${CUSTOM_HEADER_DIR}/gain-adaptive.hh
    include/${CUSTOM_HEADER_DIR}/gain-hyperbolic.hh
    include/${CUSTOM_HEADER_DIR}/gripper-control.hh
    include/${CUSTOM_HEADER_DIR}/integrator-abstract.hh
    include/${CUSTOM_HEADER_DIR}/integrator-euler.hh
    include/${CUSTOM_HEADER_DIR}/joint-limitator.hh
    include/${CUSTOM_HEADER_DIR}/kalman.hh
    include/${CUSTOM_HEADER_DIR}/latch.hh
    include/${CUSTOM_HEADER_DIR}/macros.hh
    include/${CUSTOM_HEADER_DIR}/macros-signal.hh
    include/${CUSTOM_HEADER_DIR}/mailbox-vector.hh
    include/${CUSTOM_HEADER_DIR}/mailbox.hh
    include/${CUSTOM_HEADER_DIR}/mailbox.hxx
    include/${CUSTOM_HEADER_DIR}/matrix-constant.hh
    include/${CUSTOM_HEADER_DIR}/matrix-geometry.hh
    include/${CUSTOM_HEADER_DIR}/matrix-svd.hh
    include/${CUSTOM_HEADER_DIR}/memory-task-sot.hh
    include/${CUSTOM_HEADER_DIR}/motion-period.hh
    include/${CUSTOM_HEADER_DIR}/multi-bound.hh
    include/${CUSTOM_HEADER_DIR}/neck-limitation.hh
    include/${CUSTOM_HEADER_DIR}/op-point-modifier.hh
    include/${CUSTOM_HEADER_DIR}/parameter-server.hh
    include/${CUSTOM_HEADER_DIR}/periodic-call.hh
    include/${CUSTOM_HEADER_DIR}/periodic-call-entity.hh
    include/${CUSTOM_HEADER_DIR}/pool.hh
    include/${CUSTOM_HEADER_DIR}/reader.hh
    include/${CUSTOM_HEADER_DIR}/robot-simu.hh
    include/${CUSTOM_HEADER_DIR}/robot-utils.hh
    include/${CUSTOM_HEADER_DIR}/sot-loader.hh
    include/${CUSTOM_HEADER_DIR}/sot.hh
    include/${CUSTOM_HEADER_DIR}/stop-watch.hh
    include/${CUSTOM_HEADER_DIR}/switch.hh
    include/${CUSTOM_HEADER_DIR}/task.hh
    include/${CUSTOM_HEADER_DIR}/task-abstract.hh
    include/${CUSTOM_HEADER_DIR}/task-conti.hh
    include/${CUSTOM_HEADER_DIR}/task-pd.hh
    include/${CUSTOM_HEADER_DIR}/task-unilateral.hh
    include/${CUSTOM_HEADER_DIR}/time-stamp.hh
    include/${CUSTOM_HEADER_DIR}/timer.hh
    include/${CUSTOM_HEADER_DIR}/trajectory.hh
    include/${CUSTOM_HEADER_DIR}/unary-op.hh
    include/${CUSTOM_HEADER_DIR}/utils-windows.hh
    include/${CUSTOM_HEADER_DIR}/variadic-op.hh
    include/${CUSTOM_HEADER_DIR}/vector-constant.hh
    include/${CUSTOM_HEADER_DIR}/vector-to-rotation.hh
    include/${CUSTOM_HEADER_DIR}/visual-point-projecter.hh)

set(${PROJECT_NAME}_SOURCES
    src/debug/debug.cpp
    src/debug/contiifstream.cpp
    src/exception/exception-abstract.cpp
    src/exception/exception-dynamic.cpp
    src/exception/exception-factory.cpp
    src/exception/exception-feature.cpp
    src/exception/exception-signal.cpp
    src/exception/exception-task.cpp
    src/exception/exception-tools.cpp
    src/signal/signal-cast.cpp
    src/feature/feature-abstract.cpp
    src/task/task-abstract.cpp
    src/task/multi-bound.cpp
    src/sot/flags.cpp
    src/sot/memory-task-sot.cpp
    src/factory/pool.cpp
    src/tools/utils-windows.cpp
    src/tools/periodic-call.cpp
    src/tools/device.cpp
    src/tools/sot-loader.cpp
    src/tools/trajectory.cpp
    src/tools/robot-utils.cpp
    src/matrix/matrix-svd.cpp
    src/filters/causal-filter.cpp
    src/utils/stop-watch.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::regex Boost::program_options
         dynamic-graph-python::dynamic-graph-python pinocchio::pinocchio)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)
add_subdirectory(doc)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

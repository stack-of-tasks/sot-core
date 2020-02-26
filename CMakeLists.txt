# Copyright 2010, 2019 JRL, CNRS/AIST, LAAS CNRS
# See LICENSE file.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-core)
SET(PROJECT_DESCRIPTION "Hierarchical task solver plug-in for dynamic-graph.")
SET(PROJECT_URL "http://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION (INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python binding" OFF)
OPTION(SUFFIX_SO_VERSION
  "Suffix shared library name by a string depending on git status of project"
  ON)

IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  # Export CMake Target
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/boost.cmake)

# Specify the project.
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  SET(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  SET(PYTHON_INSTALL_DIR
    ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/dynamic_graph/sot/core)
  #ADD_PROJECT_DEPENDENCY(dynamic-graph-python 3.0.0 REQUIRED)
  ADD_REQUIRED_DEPENDENCY(dynamic-graph-python 3.0.0 REQUIRED)
ENDIF(BUILD_PYTHON_INTERFACE)

SET(BOOST_COMPONENTS
  python filesystem system thread program_options unit_test_framework)
SEARCH_FOR_BOOST()
ADD_PROJECT_DEPENDENCY(Eigen3)

SET(DOXYGEN_USE_MATHJAX YES)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

# Specific to PKG module
# FIXME: to be changed into lib/dynamic-graph
# to avoid name collision when installing dynamic-graph in /usr.
SET(PLUGINDIR "${CMAKE_INSTALL_FULL_LIBDIR}/plugin")

IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  # Extra macros for sot-core
  install(FILES "src/sot-coreMacros.cmake"
    DESTINATION "${CONFIG_INSTALL_DIR}")
ENDIF (NOT INSTALL_PYTHON_INTERFACE_ONLY)

set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
include(\"\${CMAKE_CURRENT_LIST_DIR}/sot-coreMacros.cmake\")")

PKG_CONFIG_APPEND_LIBS("sot-core")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS
  thread filesystem program_options
  unit_test_framework system regex )

ADD_PROJECT_DEPENDENCY(dynamic-graph REQUIRED)

ADD_REQUIRED_DEPENDENCY ("pinocchio >= 2.2.1")

SEARCH_FOR_BOOST()

ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)
#ADD_SUBDIRECTORY(doc)

# **********************************
# Robot_utils_sot_py PYTHON module *
# **********************************
IF(BUILD_PYTHON_INTERFACE)
  IF(NOT CMAKE_VERSION VERSION_LESS "3.12" AND NOT WIN32)
    Python_add_library(robot_utils_sot_py MODULE src/tools/robot-utils-py.cpp)
  ELSE(NOT CMAKE_VERSION VERSION_LESS "3.12" AND NOT WIN32)
    PYTHON_ADD_MODULE(robot_utils_sot_py src/tools/robot-utils-py.cpp)
  ENDIF()
  TARGET_LINK_LIBRARIES(robot_utils_sot_py
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES} ${LIBRARY_NAME}
    dynamic-graph::dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(robot_utils_sot_py pinocchio)
  TARGET_LINK_BOOST_PYTHON(robot_utils_sot_py)
  INSTALL(TARGETS robot_utils_sot_py DESTINATION ${PYTHON_INSTALL_DIR})
ENDIF(BUILD_PYTHON_INTERFACE)

# We do not want the project to be finalized if this is
# to install only the  python interface.
IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SETUP_PROJECT_PACKAGE_FINALIZE()
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

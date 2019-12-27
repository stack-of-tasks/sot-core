# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

#define DEBUG=2 if we're building in debug mode (what for?)
IF("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

# The main library name
SET(SOTCORE_LIB_NAME ${PROJECT_NAME})

#This project will create many plugins as shared libraries, listed here
SET(plugins
  sot/sot

  math/op-point-modifier

  matrix/operator
  matrix/derivator
  matrix/double-constant
  matrix/fir-filter
  matrix/integrator-abstract
  matrix/integrator-euler
  matrix/matrix-constant
  matrix/vector-constant
  matrix/vector-to-rotation

  task/gain-adaptive
  task/task-pd
  task/constraint
  task/gain-hyperbolic
  task/task
  task/task-conti
  task/task-unilateral

  feature/feature-point6d
  feature/feature-pose
  feature/feature-vector3
  feature/feature-generic
  feature/feature-joint-limits
  feature/feature-1d
  feature/feature-point6d-relative
  feature/feature-visual-point
  feature/feature-task
  feature/feature-line-distance
  feature/feature-posture
  feature/visual-point-projecter

  filters/filter-differentiator
  filters/madgwickahrs

  traces/reader

  tools/event
  tools/time-stamp
  tools/timer
  tools/seq-play
  tools/sequencer
  tools/robot-simu
  tools/motion-period
  tools/neck-limitation
  tools/mailbox-vector
  tools/kalman
  tools/joint-limitator
  tools/gripper-control
  tools/com-freezer
  tools/clamp-workspace
  tools/binary-int-to-uint
  tools/periodic-call-entity
  tools/joint-trajectory-entity
  tools/latch
  tools/switch
  tools/exp-moving-avg
  tools/gradient-ascent
  tools/parameter-server

  control/control-gr
  control/control-pd

  )

# TODO
IF(WIN32)
  LIST(REMOVE_ITEM plugins
    tools/mailbox-vector
    matrix/binary-op
    matrix/derivator
    matrix/fir-filter
    matrix/integrator-abstract
    matrix/integrator-euler
    )
ENDIF(WIN32)

set(ADDITIONAL_feature-task_LIBS feature-generic task)
set(ADDITIONAL_feature-point6d-relative_LIBS feature-point6d)

set(ADDITIONAL_sot_LIBS constraint task)

set(ADDITIONAL_sequencer_LIBS sot)

set(ADDITIONAL_task-conti_LIBS task)
set(ADDITIONAL_task-pd_LIBS task)
set(ADDITIONAL_task-unilateral_LIBS task)

#Build sot-core
SET(LIBRARY_NAME ${SOTCORE_LIB_NAME})

#sot-core library sources
SET(${LIBRARY_NAME}_SOURCES
  debug/debug.cpp
  debug/contiifstream.cpp

  exception/exception-abstract.cpp
  exception/exception-dynamic.cpp
  exception/exception-factory.cpp
  exception/exception-feature.cpp
  exception/exception-signal.cpp
  exception/exception-task.cpp
  exception/exception-tools.cpp

  signal/signal-cast.cpp

  feature/feature-abstract.cpp

  task/task-abstract.cpp
  task/multi-bound.cpp

  sot/flags.cpp
  sot/memory-task-sot.cpp

  factory/pool.cpp

  tools/utils-windows
  tools/periodic-call
  tools/device
  tools/trajectory
  tools/robot-utils

  matrix/matrix-svd

  filters/causal-filter

  utils/stop-watch

  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES})


target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
  INTERFACE
  $<INSTALL_INTERFACE:include>
)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION})
target_link_libraries(${LIBRARY_NAME} dynamic-graph::dynamic-graph)

#PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)
#IF(BUILD_PYTHON_INTERFACE)
#  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)
#ENDIF(BUILD_PYTHON_INTERFACE)

IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${LIBRARY_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    PUBLIC_HEADER
    INCLUDES DESTINATION include
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})
  #retrieve plugin name
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)

  # only one source file per plugin
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${plugin}.cpp)

  #remove the "lib" prefix from the plugin output name
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    PREFIX "")

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${DYNAMIC_GRAPH_PLUGINDIR})

  # Link with sot-core library
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOTCORE_LIB_NAME})
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dynamic-graph::dynamic-graph)

  ADD_DEPENDENCIES	 (${LIBRARY_NAME} ${SOTCORE_LIB_NAME})

  IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
  ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)

  # Linux dynamic loading library flags
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
  ENDIF(UNIX)

  #PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

  # build python submodule
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    DYNAMIC_GRAPH_PYTHON_MODULE("sot/core/${PYTHON_LIBRARY_NAME}"
      ${LIBRARY_NAME}
      sot-core-${PYTHON_LIBRARY_NAME}-wrap
      )
  ENDIF(BUILD_PYTHON_INTERFACE)

  IF (NOT INSTALL_PYTHON_INTERFACE_ONLY)
    # Install plugins
    INSTALL(TARGETS ${LIBRARY_NAME}
      EXPORT ${TARGETS_EXPORT_NAME}
      PUBLIC_HEADER
      INCLUDES DESTINATION include
      LIBRARY DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR}
      )
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  
ENDFOREACH(plugin)

IF(BUILD_PYTHON_INTERFACE)
  SET(NO_INSTALL_OF_INIT_PY 0)
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/core" ${SOTCORE_LIB_NAME} wrap
    ${NO_INSTALL_OF_INIT_PY})

  # Install empty __init__.py files in intermediate directories.
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/__init__.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/math_small_entities.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/feature_position.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/feature_position_relative.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/matrix_util.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_tasks.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_task_6d.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_tasks_kine.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_tasks_kine_relative.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_task_posture.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_task_visual_point.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/core
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/attime.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/history.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/thread_interruptible_loop.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/viewer_loger.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/utils/viewer_helper.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/core/utils
    )
ENDIF(BUILD_PYTHON_INTERFACE)

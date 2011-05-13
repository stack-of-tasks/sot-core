# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-core. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Verbosity level
IF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  ADD_DEFINITIONS(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
ENDIF (NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

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
	sot/sot-qr
	sot/weighted-sot
	sot/sot-h
	sot/sot

	math/op-point-modifier

	matrix/operator
	matrix/derivator
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
	feature/feature-vector3
	feature/feature-generic
	feature/feature-joint-limits
	feature/feature-1d
	feature/feature-point6d-relative
	feature/feature-visual-point
	feature/feature-task
	feature/feature-line-distance
	feature/feature-posture

	traces/reader

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
set(ADDITIONAL_sot-h_LIBS sot)
set(ADDITIONAL_sot-h_LIBS sot)
set(ADDITIONAL_sot-qr_LIBS sot constraint)
set(ADDITIONAL_weighted-sot_LIBS sot task)

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

	math/matrix-force.cpp
	math/matrix-homogeneous.cpp
	math/matrix-rotation.cpp
	math/matrix-twist.cpp
	math/vector-quaternion.cpp
	math/vector-roll-pitch-yaw.cpp
	math/vector-utheta.cpp

	feature/feature-abstract.cpp

	task/task-abstract.cpp
	task/multi-bound.cpp

	sot/flags.cpp
	sot/memory-task-sot.cpp
	sot/rotation-simple.cpp
	sot/solver-hierarchical-inequalities.cpp

	factory/pool.cpp

	tools/utils-windows
	tools/periodic-call
	tools/device
)

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)

IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)


INSTALL(TARGETS ${LIBRARY_NAME} 
  DESTINATION lib)

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
	  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/plugin)

	# Link with sot-core library
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${SOTCORE_LIB_NAME})
	IF(APPLE)
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
	ENDIF(APPLE)
	ADD_DEPENDENCIES	 (${LIBRARY_NAME} ${SOTCORE_LIB_NAME})

	IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
	  ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
	  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
	ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)	
	
	# Linux dynamic loading library flags
	IF(UNIX)
	  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
	ENDIF(UNIX)

	IF(UNIX AND NOT APPLE)
	  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl pthread gfortran)
	ENDIF(UNIX AND NOT APPLE)

	PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-mal)
	PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)

	# build python submodule
	STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
	FILE(MAKE_DIRECTORY dynamic_graph/sot/core/${PYTHON_LIBRARY_NAME})
	DYNAMIC_GRAPH_PYTHON_MODULE("sot/core/${PYTHON_LIBRARY_NAME}"
	  ${LIBRARY_NAME}
	  sot-core-${PYTHON_LIBRARY_NAME}-wrap
	  )
	# Install plugins
	INSTALL(TARGETS ${LIBRARY_NAME}
	  DESTINATION lib/plugin)
ENDFOREACH(plugin)

# Bindings Python
DYNAMIC_GRAPH_PYTHON_MODULE("sot/core" ${SOTCORE_LIB_NAME} wrap)

# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/se3.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot
  )
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/math_small_entities.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/feature_position.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/core/meta_task_6d.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/core
  )

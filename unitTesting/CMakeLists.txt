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

# Relative plug-in directory (i.e. lib/plugin for instance)
# as exposed by the dynamic-graph pkg-config file.
SET(PLUGINDIR ${DYNAMIC_GRAPH_PLUGINDIRNAME})
SET(FULLPLUGINDIR ${CMAKE_INSTALL_PREFIX}/${PLUGINDIR})
SET(SOT_PLUGIN_OUT_DIR ${CMAKE_BINARY_DIR}/src)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h.cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h)

IF(UNIX)
  ADD_DEFINITIONS(-pthread)
ENDIF(UNIX)

# The sot-core plugin libraries that tests have to be linked against
# the name of the variable for test EXECUTABLE_NAME is
# TEST_${EXECUTABLE_NAME}_LIBS
SET(TEST_tsot_LIBS
	task
	feature-visual-point
	gain-adaptive
)

IF(WIN32)
SET(TEST_traces_EXT_LIBS
	${DYNAMIC_GRAPH_PLUGINDIR}/tracer${CMAKE_LINK_LIBRARY_SUFFIX}
)
ELSE(WIN32)
SET(TEST_traces_EXT_LIBS
	${DYNAMIC_GRAPH_PLUGINDIR}/tracer${CMAKE_SHARED_LIBRARY_SUFFIX}
)
ENDIF(WIN32)

SET(TEST_test_gain_LIBS
	gain-adaptive feature-visual-point
)

SET(TEST_test_task_LIBS
	gain-adaptive feature-visual-point task
)

SET(TEST_test_mailbox_LIBS
	mailbox-vector
)

#test paths and names (without .cpp extension)
SET (tests
	dummy

	signal/test_signal
	signal/test_depend
	signal/test_ptr
	signal/test_dep
	signal/test_ptrcast

	sot/tsot
	sot/test_solverSoth

	traces/files
	traces/traces

	task/test_flags
	task/test_gain
	task/test_multi_bound
	task/test_task

	tools/test_boost
	tools/test_mailbox
	tools/test_matrix
	)

IF(UNIX)
	LINK_DIRECTORIES(${SOT_PLUGIN_OUT_DIR} ${FULLPLUGINDIR})
ENDIF(UNIX)



ADD_DEFINITIONS(-DDEBUG=2)

# Add MatrixAbstractLayer compilation flags
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

FOREACH(test ${tests})
	GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${test} NAME)

	ADD_EXECUTABLE(${EXECUTABLE_NAME} ${test}.cpp)

	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${PROJECT_NAME})
	ADD_DEPENDENCIES     (${EXECUTABLE_NAME} ${PROJECT_NAME})

	IF( TEST_${EXECUTABLE_NAME}_LIBS )
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
		ADD_DEPENDENCIES     (${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
	ENDIF( TEST_${EXECUTABLE_NAME}_LIBS )
	
	IF( TEST_${EXECUTABLE_NAME}_EXT_LIBS )
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_EXT_LIBS})
	ENDIF( TEST_${EXECUTABLE_NAME}_EXT_LIBS )

	IF (UNIX)
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} dl)
	ENDIF(UNIX)

	PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} jrl-mal)
	PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} dynamic-graph)

	ADD_TEST(${test} ${EXECUTABLE_NAME})

	IF (UNIX)
	  SET_PROPERTY(TEST ${test} PROPERTY 
	    ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_BINARY_DIR}/src:${BOOST_ROOT}/lib")
	ENDIF(UNIX)

ENDFOREACH(test)

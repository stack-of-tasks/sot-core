# 
#  Copyright 
# 

# Libraries DEFINITIONS and INCLUDES

# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

#define VP_DEBUG if we're building in debug mode
IF("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)
	ADD_DEFINITIONS(-DDEBUG=2)
ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL DEBUG)

# Add lapack compilation flags and link to library libLapack.so
ADD_DEFINITIONS(${LAPACK_CFLAGS})

# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})

# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})

# provide path to library libMatrixAbstractLayer.so
LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})

# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

IF(WIN32)
foreach(dlink ${MATRIXABSTRACTLAYER_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

IF(WIN32)
foreach(dlink ${DYNAMIC_GRAPH_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# The main library name
SET(SOTCORE_LIB_NAME ${PROJECT_NAME})

#This project will create many plugins as shared libraries, listed here
SET(plugins 
	sot/sot-qr
	sot/weighted-sot
	sot/sot-h
	sot/sot
	
	math/op-point-modifier
	
	matrix/binary-op
	matrix/derivator
	matrix/fir-filter
	matrix/integrator-abstract
	matrix/integrator-euler
	matrix/matrix-constant
	matrix/unary-op
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
	
	traces/reader
	
	tools/time-stamp
	tools/timer
	tools/seq-play
	tools/sequencer
	tools/robot-simu
	tools/periodic-call-entity
	tools/motion-period
	tools/neck-limitation
)

#Build sot-core
SET(LIBRARY_NAME ${SOTCORE_LIB_NAME})

#sot-core library sources
SET(${PROJECT_NAME}_SOURCES
	debug/debug.cpp
	debug/contiifstream.cpp
	
	exception/exception-abstract.cpp
	exception/exception-dynamic.cpp
	exception/exception-factory.cpp
	exception/exception-feature.cpp
	exception/exception-signal.cpp
	exception/exception-task.cpp
	
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
	
	factory/additional-functions.cpp
	factory/factory.cpp
	factory/pool.cpp
	factory/command/import.cpp
	
	tools/utils-windows
	tools/periodic-call
)

ADD_LIBRARY(${LIBRARY_NAME} 
  SHARED
  ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
  PROPERTIES 
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

#SET_TARGET_PROPERTIES(${LIBRARY_NAME}
#  PROPERTIES
  #COMPILE_FLAGS "${${PROJECT_NAME}_src_CXXFLAGS}"
  #LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
#)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${LAPACK_LIBRARIES})
	
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
ENDIF(UNIX)

IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
ENDIF(UNIX)

# Linux dynamic loading library flags
IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl)
ENDIF(UNIX)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

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
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${LAPACK_LIBRARIES})
	# Linux dynamic loading library flags
	IF(UNIX)
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
	ENDIF(UNIX)
	IF(UNIX)
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
	ENDIF(UNIX)
	IF(UNIX)
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl)
	ENDIF(UNIX)
	
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
	
	INSTALL(TARGETS ${LIBRARY_NAME}
	  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/plugin)
ENDFOREACH(plugin)

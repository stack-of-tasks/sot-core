# 
#  Copyright 
# 

#This project will create many shared libraries as plugins, listed here
SET(plugins 
	sot/sot-qr
	sot/weighted-sot
	sot/sot-h
	sot/sot
)

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
	
	matrix/binary-op.cpp
	matrix/derivator.cpp
	matrix/fir-filter.cpp
	matrix/integrator-abstract.cpp
	matrix/integrator-euler.cpp
	matrix/matrix-constant.cpp
	matrix/unary-op.cpp
	matrix/vector-constant.cpp
	matrix/vector-to-rotation.cpp
	
	math/matrix-force.cpp
	math/matrix-homogeneous.cpp
	math/matrix-rotation.cpp
	math/matrix-twist.cpp
	math/op-point-modifier.cpp
	math/vector-quaternion.cpp
	math/vector-roll-pitch-yaw.cpp
	math/vector-utheta.cpp
	
	feature/feature-point6d.cpp
	feature/feature-vector3.cpp
	feature/feature-abstract.cpp
	feature/feature-generic.cpp
	feature/feature-joint-limits.cpp
	feature/feature-1d.cpp
	feature/feature-point6d-relative.cpp
	feature/feature-visual-point.cpp
	feature/feature-task.cpp
	feature/feature-line-distance.cpp
	
	task/gain-adaptative.cpp
	task/task-pd.cpp
	task/constraint.cpp
	task/gain-hyperbolic.cpp
	task/task.cpp
	task/multi-bound.cpp
	task/task-conti.cpp
	task/task-unilateral.cpp
	task/task-abstract.cpp
	
	sot/flags.cpp
	sot/sot-qr.cpp
	sot/memory-task-sot.cpp
	sot/sot.cpp
	sot/rotation-simple.cpp
	sot/weighted-sot.cpp
	sot/solver-hierarchical-inequalities.cpp
	
	sot/sot-h.cpp
	
	factory/additional-functions.cpp
	factory/factory.cpp
	factory/pool.cpp
	factory/command/import.cpp
)

SET(LIBRARY_NAME ${PROJECT_NAME})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libMatrixAbstractLayer.so
LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})

# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

ADD_LIBRARY(${LIBRARY_NAME} 
  SHARED
  ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
  PROPERTIES 
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  COMPILE_FLAGS "${${PROJECT_NAME}_src_CXXFLAGS}"
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

# Add lapack compilation flags and link to library libLapack.so
ADD_DEFINITIONS(${LAPACK_CFLAGS})
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${LAPACK_LIBRARIES})
	
# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${MATRIXABSTRACTLAYER_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${DYNAMIC_GRAPH_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Linux dynamic loading library flags
IF(UNIX)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl)
ENDIF(UNIX)

MESSAGE(STATUS "ldflags ${${PROJECT_NAME}_src_LDFLAGS}" )
SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

#Plugins compilation, link, and installation
FOREACH(plugin ${plugins})
	#retrieve plugin name
	GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
	
	INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
	
	ADD_DEFINITIONS(-DDEBUG=2)
	
	# provide path to library libMatrixAbstractLayer.so
	LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})
	
	# provide path to library libdynamic-graph.so
	LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})
	
	# only one source file per plugin
	ADD_LIBRARY(${LIBRARY_NAME} 
	  SHARED
	  ${plugin}.cpp)
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
	  PROPERTIES 
	  SOVERSION ${PROJECT_VERSION}
	  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/plugin)
	
	# Add lapack compilation flags and link to library libLapack.so
	ADD_DEFINITIONS(${LAPACK_CFLAGS})
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${LAPACK_LIBRARIES})
		
	# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
	ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})
	IF(UNIX)
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
	ENDIF(UNIX)
	
	IF(WIN32)
	foreach(dlink ${MATRIXABSTRACTLAYER_LDFLAGS})
		SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
	endforeach(dlink)
	ENDIF(WIN32)
	
	# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
	ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
	IF(UNIX)
	TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
	ENDIF(UNIX)
	
	IF(WIN32)
	foreach(dlink ${DYNAMIC_GRAPH_LDFLAGS})
		SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
	endforeach(dlink)
	ENDIF(WIN32)
	
	# Linux dynamic loading library flags
	IF(UNIX)
		TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl)
	ENDIF(UNIX)
	
	SET_TARGET_PROPERTIES(${LIBRARY_NAME}
	  PROPERTIES
	  COMPILE_FLAGS "${${PROJECT_NAME}_src_CXXFLAGS}"
	  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
	)
	
	INSTALL(TARGETS ${LIBRARY_NAME}
	  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/plugin)
ENDFOREACH(plugin)

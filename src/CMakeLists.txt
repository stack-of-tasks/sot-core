# 
#  Copyright 
# 

SET(LIBRARY_NAME ${PROJECT_NAME})

#Warning: compile order is VERY important (if you get a segfault, it's probably
#the reason
SET(${LIBRARY_NAME}_SOURCES
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

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libMatrixAbstractLayer.so
LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})


# provide path to library libdynamicsJRLJapan.so
LINK_DIRECTORIES(${DYNAMICSJRLJAPAN_LIBRARY_DIRS})


# provide path to library libhrp2Dynamics.so
LINK_DIRECTORIES(${HRP2DYNAMICS_LIBRARY_DIRS})


# provide path to library libwalkGenJrl.so
LINK_DIRECTORIES(${WALKGENJRL_LIBRARY_DIRS})


# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

ADD_LIBRARY(${LIBRARY_NAME} 
  SHARED
  ${${LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${LIBRARY_NAME} 
  PROPERTIES 
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
  PROPERTIES
  COMPILE_FLAGS ${${PROJECT_NAME}_src_CXXFLAGS}
  LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
)

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

# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMICSJRLJAPAN_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${DYNAMICSJRLJAPAN_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${HRP2DYNAMICS_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${HRP2DYNAMICS_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

# Add walkGenJrl compilation flags and link to library libwalkGenJrl.so
ADD_DEFINITIONS(${WALKGENJRL_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${WALKGENJRL_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${WALKGENJRL_LDFLAGS})
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
  SET(${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS} -ldl")
ENDIF(UNIX)


SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

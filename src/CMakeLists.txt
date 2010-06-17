# 
#  Copyright 
# 

SET(LIBRARY_NAME ${PROJECT_NAME})

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
	exception/exception-traces.cpp
	
	signal/signal-cast.cpp
	
	matrix/sotBinaryOp.cpp
	matrix/sotDerivator.cpp
	matrix/sotFIRFilter.cpp
	matrix/sotIntegratorAbstract.cpp
	matrix/sotIntegratorEuler.cpp
	matrix/sotMatrixConstant.cpp
	matrix/sotUnaryOp.cpp
	matrix/sotVectorConstant.cpp
	matrix/sotVectorToRotation.cpp
	
	factory/sotAdditionalFunctions.cpp
	factory/sotFactory.cpp
	factory/sotPool.cpp
	
	math/sotMatrixForce.cpp
	math/sotMatrixHomogeneous.cpp
	math/sotMatrixRotation.cpp
	math/sotMatrixTwist.cpp
	math/sotOpPointModifior.cpp
	math/sotVectorQuaternion.cpp
	math/sotVectorRollPitchYaw.cpp
	math/sotVectorUTheta.cpp
	
	feature/sotFeaturePoint6d.cpp
	feature/sotFeatureVector3.cpp
	feature/sotFeatureAbstract.cpp
	feature/sotFeatureGeneric.cpp
	feature/sotFeatureJointLimits.cpp
	feature/sotFeature1D.cpp
	feature/sotFeaturePoint6dRelative.cpp
	feature/sotFeatureVisualPoint.cpp
	feature/sotFeatureTask.cpp
	feature/sotFeatureLineDistance.cpp
	
	task/sotGainAdaptative.cpp
	task/sotTaskPD.cpp
	task/sotConstraint.cpp
	task/sotGainHyperbolic.cpp
	task/sotTask.cpp
	task/sotMultiBound.cpp
	task/sotTaskConti.cpp
	task/sotTaskUnilateral.cpp
	task/sotTaskAbstract.cpp
	
	sot/sotFlags.cpp
	sot/sotSOTQr.cpp
	sot/sotMemoryTaskSOT.cpp
	sot/sotSOTH.cpp
	sot/sotSOT.cpp
	sot/sotRotationSimple.cpp
	sot/sotWeightedSOT.cpp
	sot/sotSolverHierarchicalInequalities.cpp
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

# Dynamic loading
IF(UNIX)
  SET(${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS} -ldl")
ENDIF(UNIX)


SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

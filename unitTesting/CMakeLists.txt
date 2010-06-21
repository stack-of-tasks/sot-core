# 
#  Copyright 
# 

SET (tests
	main
	signal/test_signal)

FOREACH(test ${tests})
	GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${test} NAME)
	
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
	
	ADD_EXECUTABLE(${EXECUTABLE_NAME}
	  ${test}.cpp
	  )
	
	INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
	
	LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
	  sot-core)
	
	# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
	ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
	
	# Add dynamicsJRLJapan compilation flags and link to library libdynamicsJRLJapan.so
	ADD_DEFINITIONS(${DYNAMICSJRLJAPAN_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMICSJRLJAPAN_LIBRARIES})
	
	# Add hrp2Dynamics compilation flags and link to library libhrp2Dynamics.so
	ADD_DEFINITIONS(${HRP2DYNAMICS_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${HRP2DYNAMICS_LIBRARIES})
	
	# Add walkGenJrl compilation flags and link to library libwalkGenJrl.so
	ADD_DEFINITIONS(${WALKGENJRL_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${WALKGENJRL_LIBRARIES})
	
	# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
	ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
	
	# FIXME
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
		dl)

ENDFOREACH(test)

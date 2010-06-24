# 
#  Copyright 
# 

SET (tests
	main
	signal/test_signal
	signal/test_array
	signal/test_depend
	signal/test_ptr
	signal/test_dep)

FOREACH(test ${tests})
	GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${test} NAME)
	
	ADD_DEFINITIONS(-DDEBUG=2)
	
	# provide path to library libMatrixAbstractLayer.so
	LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})
	
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
	
	# Add lapack compilation flags and link to library libLapack.so
	ADD_DEFINITIONS(${LAPACK_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${LAPACK_LIBRARIES})

	# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
	ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
	
	# FIXME
	IF (UNIX)
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
		dl)
	ENDIF(UNIX)

ENDFOREACH(test)

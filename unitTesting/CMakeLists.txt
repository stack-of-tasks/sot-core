# 
#  Copyright 
# 

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h.cmake 
			${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h) 

#The sot-core plugin libraries that tests have to be linked against
#the name of the variable for test EXECUTABLE_NAME is TEST_${EXECUTABLE_NAME}_LIBS
SET(TEST_tsot_LIBS 
	task
	feature-visual-point
	gain-adaptive
)

SET(TEST_files_LIBS
	${Boost_LIBRARIES}
)

SET(TEST_traces_LIBS
	${CMAKE_INSTALL_PREFIX}/lib/plugin/tracer.so
)

SET(TEST_test_gain_LIBS
	gain-adaptive feature-visual-point
)

SET(TEST_test_task_LIBS
	gain-adaptive feature-visual-point task
)

#test paths and names (without .cpp extension)
SET (tests
	link
	signal/test_signal
	signal/test_depend
	signal/test_ptr
	signal/test_dep
	sot/tsot
	sot/test_solverSoth
	factory/test_factory
	traces/files
	traces/traces
	task/test_flags
	task/test_gain
	task/test_multi_bound
	task/test_task)

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
	  ${PROJECT_NAME})
	
	# Add MatrixAbstractLayer compilation flags and link to library libMatrixAbstractLayer.so
	ADD_DEFINITIONS(${MATRIXABSTRACTLAYER_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
	
	# Add lapack compilation flags and link to library libLapack.so
	ADD_DEFINITIONS(${LAPACK_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${LAPACK_LIBRARIES})

	# Add dynamic-graph compilation flags and link to library libdynamic-graph.so
	ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
	
	IF (UNIX)
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
		dl)
	ENDIF(UNIX)
	
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
	
	ADD_TEST(${test} ${EXECUTABLE_NAME})

ENDFOREACH(test)

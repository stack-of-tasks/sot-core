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

IF(UNIX)
SET(TEST_traces_LIBS
	${CMAKE_INSTALL_PREFIX}/lib/plugin/tracer.so
)
ENDIF(UNIX)
IF(WIN32)
SET(TEST_traces_LIBS
	${CMAKE_INSTALL_PREFIX}/lib/plugin/tracer.lib
)
ENDIF(WIN32)

SET(TEST_test_gain_LIBS
	gain-adaptive feature-visual-point
)

SET(TEST_test_task_LIBS
	gain-adaptive feature-visual-point task
)

SET(TEST_test_mailbox_LIBS
	mailbox
)

#test paths and names (without .cpp extension)
SET (tests
	dummy
	
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
	task/test_task
	
	tools/test_boost
	tools/test_mailbox
	tools/test_matrix
	)

	
IF(WIN32)
foreach(dlink  ${MATRIXABSTRACTLAYER_LDFLAGS}  ${DYNAMIC_GRAPH_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

IF(UNIX)
	LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src ${CMAKE_INSTALL_PREFIX}/lib/plugin)
ENDIF(UNIX)


	
ADD_DEFINITIONS(-DDEBUG=2)

# provide path to library libMatrixAbstractLayer.so
LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})

# provide path to library libdynamic-graph.so
LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})

# Add MatrixAbstractLayer compilation flags
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
	
# Add lapack compilation flags
ADD_DEFINITIONS(${LAPACK_CFLAGS})

# Add dynamic-graph compilation flags
ADD_DEFINITIONS(${DYNAMIC_GRAPH_CFLAGS})
	
FOREACH(test ${tests})
	GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${test} NAME)

	ADD_EXECUTABLE(${EXECUTABLE_NAME}
	  ${test}.cpp
	  )
		
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
	  ${PROJECT_NAME})
	
	
	# Add link to library libLapack.so
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${LAPACK_LIBRARIES})

	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${Boost_THREAD_LIBRARY})

	IF (UNIX)
		# Add link to library libdynamic-graph.so
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${DYNAMIC_GRAPH_LIBRARIES})
		
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
			dl)
	
		# Add link to library libMatrixAbstractLayer.so
		TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${MATRIXABSTRACTLAYER_LIBRARIES})
	
	ENDIF(UNIX)
	
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_${EXECUTABLE_NAME}_LIBS})
	
	IF(WIN32)
	SET_TARGET_PROPERTIES(${EXECUTABLE_NAME}
	 PROPERTIES
	  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
	)
	ENDIF(WIN32)

	ADD_TEST(${test} ${EXECUTABLE_NAME})

ENDFOREACH(test)

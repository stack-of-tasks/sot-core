# 
#  Copyright 
# 

### tools

#The sot-core plugin libraries that tools have to be linked against
#There is currently no way to define separate libraries for separate tools.
SET(TEST_LINK_LIBRARIES 
	task
	feature-visual-point
	gain-adaptative
)

SET(tools
	""
)

FOREACH(tool_name ${tools})
	GET_FILENAME_COMPONENT(EXECUTABLE_NAME ${tool_name} NAME)
	
	INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
	
	LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
	
	# provide path to library libMatrixAbstractLayer.so
	LINK_DIRECTORIES(${MATRIXABSTRACTLAYER_LIBRARY_DIRS})
	
	# provide path to library libdynamic-graph.so
	LINK_DIRECTORIES(${DYNAMIC_GRAPH_LIBRARY_DIRS})
	
	
	ADD_DEFINITIONS(-DDEBUG=2)

	ADD_EXECUTABLE(${EXECUTABLE_NAME}
	  ${tool_name}.cpp)
	
	
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
	
	# Add tests libraries
	TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${TEST_LINK_LIBRARIES})
	  
	INSTALL(TARGETS ${EXECUTABLE_NAME}
		DESTINATION bin/${PROJECT_NAME}/)
ENDFOREACH(tool_name)
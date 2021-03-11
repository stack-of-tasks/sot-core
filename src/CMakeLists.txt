# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS

# ROS buildfarm uses isolated environments, in which the build rpath is used
set(BUILD_WITH_INSTALL_RPATH TRUE)

SET(plugins
  sot/sot

  math/op-point-modifier

  matrix/operator
  matrix/derivator
  matrix/double-constant
  matrix/fir-filter
  matrix/integrator-abstract
  matrix/integrator-euler
  matrix/matrix-constant
  matrix/vector-constant
  matrix/vector-to-rotation

  task/gain-adaptive
  task/task-pd
  task/gain-hyperbolic
  task/task
  task/task-conti
  task/task-unilateral

  feature/feature-point6d
  feature/feature-pose
  feature/feature-vector3
  feature/feature-generic
  feature/feature-joint-limits
  feature/feature-1d
  feature/feature-point6d-relative
  feature/feature-visual-point
  feature/feature-task
  feature/feature-line-distance
  feature/feature-posture
  feature/visual-point-projecter

  filters/filter-differentiator
  filters/madgwickahrs

  traces/reader

  tools/event
  tools/time-stamp
  tools/timer
  tools/sequencer
  tools/robot-simu
  tools/motion-period
  tools/neck-limitation
  tools/mailbox-vector
  tools/kalman
  tools/joint-limitator
  tools/gripper-control
  tools/com-freezer
  tools/clamp-workspace
  tools/binary-int-to-uint
  tools/periodic-call-entity
  tools/joint-trajectory-entity
  tools/latch
  tools/switch
  tools/exp-moving-avg
  tools/gradient-ascent
  tools/parameter-server
  tools/smooth-reach

  control/control-gr
  control/control-pd
  control/admittance-control-op-point
  )

# TODO
IF(WIN32)
  LIST(REMOVE_ITEM plugins
    tools/mailbox-vector
    matrix/binary-op
    matrix/derivator
    matrix/fir-filter
    matrix/integrator-abstract
    matrix/integrator-euler
    )
ENDIF(WIN32)

set(feature-task_deps feature-generic task)
set(feature-point6d-relative_deps feature-point6d)
set(sot_deps task feature-posture)
set(sequencer_deps sot)
set(task-conti_deps task)
set(task-pd_deps task)
set(task-unilateral_deps task)

FOREACH(plugin ${plugins})
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED ${plugin})
  SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES INSTALL_RPATH $ORIGIN)

  IF(SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF(SUFFIX_SO_VERSION)

  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME} ${${LIBRARY_NAME}_deps})

  IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
    INSTALL(TARGETS ${LIBRARY_NAME} EXPORT ${TARGETS_EXPORT_NAME}
      DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    else()
      DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}"
        ${LIBRARY_NAME} ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
        MODULE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.h")
    endif()
    target_link_libraries(${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap eigenpy::eigenpy)
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(plugin)

IF(BUILD_PYTHON_INTERFACE)
  DYNAMIC_GRAPH_PYTHON_MODULE("${PYTHON_DIR}"
    ${PROJECT_NAME} ${PROJECT_NAME}-wrap
    SOURCE_PYTHON_MODULE "${CMAKE_CURRENT_SOURCE_DIR}/python-module.cc")

  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/__init__.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot)

  SET(${PROJECT_NAME}_PYTHON
    __init__.py
    math_small_entities.py
    feature_position.py
    feature_position_relative.py
    matrix_util.py
    meta_tasks.py
    meta_task_6d.py
    meta_tasks_kine.py
    meta_tasks_kine_relative.py
    meta_task_posture.py
    meta_task_visual_point.py
    )

  FOREACH(file ${${PROJECT_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
  ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})

  SET(${PY_NAME}_PYTHON_UTILS
    __init__.py
    attime.py
    history.py
    thread_interruptible_loop.py
    viewer_loger.py
    viewer_helper.py
    )

  FOREACH(file ${${PY_NAME}_PYTHON_UTILS})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}/utils" ${file})
  ENDFOREACH(file ${${PY_NAME}_PYTHON_UTILS})

  # **********************************
  # Robot_utils_sot_py PYTHON module *
  # **********************************
  ADD_LIBRARY(robot_utils_sot_py SHARED tools/robot-utils-py.cpp)
  TARGET_INCLUDE_DIRECTORIES(robot_utils_sot_py SYSTEM PUBLIC ${PYTHON_INCLUDE_DIR})
  TARGET_LINK_LIBRARIES(robot_utils_sot_py PUBLIC
    ${PYTHON_LIBRARIES} ${PROJECT_NAME})
  TARGET_LINK_BOOST_PYTHON(robot_utils_sot_py PRIVATE)
  SET_TARGET_PROPERTIES(robot_utils_sot_py PROPERTIES PREFIX "")
  INSTALL(TARGETS robot_utils_sot_py DESTINATION "${PYTHON_SITELIB}/dynamic_graph/${PYTHON_DIR}")
ENDIF(BUILD_PYTHON_INTERFACE)

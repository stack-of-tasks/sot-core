# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL,
# CNRS/AIST, LAAS-CNRS

# ROS buildfarm uses isolated environments, in which the build rpath is used
set(BUILD_WITH_INSTALL_RPATH TRUE)

set(plugins
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
    control/admittance-control-op-point)

# TODO
if(WIN32)
  list(
    REMOVE_ITEM
    plugins
    tools/mailbox-vector
    matrix/binary-op
    matrix/derivator
    matrix/fir-filter
    matrix/integrator-abstract
    matrix/integrator-euler)
endif(WIN32)

set(feature-task_deps feature-generic task)
set(feature-point6d-relative_deps feature-point6d)
set(sot_deps task feature-posture)
set(sequencer_deps sot)
set(task-conti_deps task)
set(task-pd_deps task)
set(task-unilateral_deps task)

foreach(plugin ${plugins})
  get_filename_component(LIBRARY_NAME ${plugin} NAME)
  add_library(${LIBRARY_NAME} SHARED "${plugin}.cpp")
  set_target_properties(${LIBRARY_NAME} PROPERTIES INSTALL_RPATH $ORIGIN)

  if(SUFFIX_SO_VERSION)
    set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  target_link_libraries(${LIBRARY_NAME} PUBLIC ${PROJECT_NAME}
                                               ${${LIBRARY_NAME}_deps})

  install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

  string(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
    dynamic_graph_python_module(
      "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
      ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
      "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python-module-py.cc")
  else()
    dynamic_graph_python_module(
      "${PYTHON_DIR}/${PYTHON_LIBRARY_NAME}" ${LIBRARY_NAME}
      ${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap MODULE_HEADER
      "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}-python.h")
  endif()
  target_link_libraries(${PROJECT_NAME}-${PYTHON_LIBRARY_NAME}-wrap
                        PUBLIC eigenpy::eigenpy)
endforeach(plugin)

dynamic_graph_python_module(
  "${PYTHON_DIR}" ${PROJECT_NAME} ${PROJECT_NAME}-wrap SOURCE_PYTHON_MODULE
  "${CMAKE_CURRENT_SOURCE_DIR}/python-module.cc")

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/__init__.py
        DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot)

set(${PROJECT_NAME}_PYTHON
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
    meta_task_visual_point.py)

foreach(file ${${PROJECT_NAME}_PYTHON})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}" ${file})
endforeach(file ${${PROJECT_NAME}_PYTHON})

set(${PY_NAME}_PYTHON_UTILS
    __init__.py attime.py history.py thread_interruptible_loop.py
    viewer_loger.py viewer_helper.py)

foreach(file ${${PY_NAME}_PYTHON_UTILS})
  python_install_on_site("dynamic_graph/${PYTHON_DIR}/utils" ${file})
endforeach(file ${${PY_NAME}_PYTHON_UTILS})

# **********************************
# Robot_utils_sot_py PYTHON module *
# **********************************
add_library(robot_utils_sot_py SHARED tools/robot-utils-py.cpp)
target_link_libraries(robot_utils_sot_py PUBLIC ${PROJECT_NAME})
target_link_boost_python(robot_utils_sot_py PRIVATE)
set_target_properties(robot_utils_sot_py PROPERTIES PREFIX "")
install(TARGETS robot_utils_sot_py
        DESTINATION "${PYTHON_SITELIB}/dynamic_graph/${PYTHON_DIR}")

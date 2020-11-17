# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS

CONFIG_FILES_CMAKE(test-paths.h)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Definition of a macro that corresponds to the suffix for a dynamic lib
ADD_DEFINITIONS(-DTESTS_DYNLIBSUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

# The sot-core plugin libraries that tests have to be linked against
# the name of the variable for test EXECUTABLE_NAME is
# TEST_${EXECUTABLE_NAME}_LIBS
SET(TEST_tsot_LIBS
  task
  feature-visual-point
  gain-adaptive
  )

SET(TEST_test_traces_EXT_LIBS dynamic-graph::tracer)

SET(TEST_test_gain_LIBS
  gain-adaptive feature-visual-point)

SET(TEST_test_task_LIBS
  gain-adaptive feature-visual-point task)

SET(TEST_test_feature_point6d_LIBS
  gain-adaptive feature-point6d task)

SET(TEST_test_feature_generic_LIBS
  gain-adaptive feature-generic task feature-pose)

SET(TEST_test_mailbox_LIBS
  mailbox-vector)

SET(TEST_test_control_pd_LIBS
  control-pd)

SET(TEST_test_control_admittance_LIBS
  admittance-control-op-point)

SET(TEST_test_feature_generic_EXT_LIBS
  pinocchio::pinocchio)

SET(TEST_test_device_EXT_LIBS
  pinocchio::pinocchio)

SET(TEST_test_filter_differentiator_LIBS
  filter-differentiator)

SET(TEST_test_madgwick_ahrs_LIBS
  madgwickahrs)


SET(tests
  dummy

  control/test_control_pd
  control/test_control_admittance

  features/test_feature_point6d
  features/test_feature_generic

  filters/test_filter_differentiator
  filters/test_madgwick_ahrs

  signal/test_signal
  signal/test_depend
  signal/test_ptr
  signal/test_dep
  signal/test_ptrcast

  sot/tsot

  traces/files
  traces/test_traces

  task/test_flags
  task/test_gain
  task/test_multi_bound
  task/test_task

  tools/test_boost
  tools/test_device
  tools/test_mailbox
  tools/test_matrix
  tools/test_robot_utils

  math/matrix-twist
  math/matrix-homogeneous

  matrix/test_operator
  )

# TODO
IF(WIN32)
  LIST(REMOVE_ITEM tests tools/test_mailbox)
ENDIF(WIN32)

IF(UNIX)
  ADD_LIBRARY(pluginabstract SHARED tools/plugin.cc)

  ADD_EXECUTABLE(test_abstract_interface tools/test_abstract_interface.cpp)
  TARGET_LINK_LIBRARIES(test_abstract_interface PRIVATE
    Boost::program_options
    pluginabstract ${CMAKE_DL_LIBS})
ENDIF(UNIX)

FOREACH(path ${tests})
  GET_FILENAME_COMPONENT(test ${path} NAME)
  ADD_UNIT_TEST(${test} ${path}.cpp)

  TARGET_LINK_LIBRARIES(${test} PRIVATE ${PROJECT_NAME}
    Boost::unit_test_framework
    ${TEST_${test}_LIBS} ${TEST_${test}_EXT_LIBS})

  IF(UNIX)
    TARGET_LINK_LIBRARIES(${test} PRIVATE ${CMAKE_DL_LIBS})
  ENDIF(UNIX)
ENDFOREACH(path ${tests})

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)

# Copyright 2010, 2020, François Bleibel, Olivier Stasse, Guilhem Saurel, JRL,
# CNRS/AIST, LAAS-CNRS

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/test-paths.h)

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Definition of a macro that corresponds to the suffix for a dynamic lib
add_definitions(-DTESTS_DYNLIBSUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

# The sot-core plugin libraries that tests have to be linked against the name of
# the variable for test EXECUTABLE_NAME is TEST_${EXECUTABLE_NAME}_LIBS
set(TEST_tsot_LIBS task feature-visual-point gain-adaptive)

set(TEST_test_traces_EXT_LIBS dynamic-graph::tracer)

set(TEST_test_gain_LIBS gain-adaptive feature-visual-point)

set(TEST_test_task_LIBS gain-adaptive feature-visual-point task)

set(TEST_test_feature_point6d_LIBS gain-adaptive feature-point6d task)

set(TEST_test_feature_generic_LIBS gain-adaptive feature-generic task
                                   feature-pose)

set(TEST_test_mailbox_LIBS mailbox-vector)

set(TEST_test_control_pd_LIBS control-pd)

set(TEST_test_control_admittance_LIBS admittance-control-op-point)

set(TEST_test_feature_generic_EXT_LIBS pinocchio::pinocchio)

set(TEST_test_device_EXT_LIBS pinocchio::pinocchio)

set(TEST_test_filter_differentiator_LIBS filter-differentiator)

set(TEST_test_madgwick_ahrs_LIBS madgwickahrs)

set(tests
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
    tools/test_sot_loader
    tools/test_mailbox
    tools/test_matrix
    tools/test_robot_utils
    math/matrix-twist
    math/matrix-homogeneous
    matrix/test_operator)

# TODO
if(WIN32)
  list(REMOVE_ITEM tests tools/test_mailbox)
endif(WIN32)

if(UNIX)
  add_library(pluginabstract SHARED tools/plugin.cc)

  add_executable(test_abstract_interface tools/test_abstract_interface.cpp)
  target_link_libraries(
    test_abstract_interface PRIVATE Boost::program_options pluginabstract
                                    ${CMAKE_DL_LIBS})

  add_library(dummy_sot_external_interface SHARED
              tools/dummy-sot-external-interface.cc)
endif(UNIX)

foreach(path ${tests})
  get_filename_component(test ${path} NAME)
  add_unit_test(${test} ${path}.cpp)

  target_link_libraries(
    ${test} PRIVATE ${PROJECT_NAME} Boost::unit_test_framework
                    ${TEST_${test}_LIBS} ${TEST_${test}_EXT_LIBS})

  # add some preprocessor variable
  target_compile_definitions(
    ${test}
    PUBLIC
      LIB_PLUGIN_ABSTRACT_PATH="${CMAKE_BINARY_DIR}/tests/libdummy_sot_external_interface.so"
      TEST_GOOD_PYTHON_SCRIPT="${PROJECT_SOURCE_DIR}/tests/tools/good_python_script.py"
      TEST_BAD_PYTHON_SCRIPT="${PROJECT_SOURCE_DIR}/tests/tools/bad_python_script"
  )

  if(UNIX)
    target_link_libraries(${test} PRIVATE ${CMAKE_DL_LIBS})
  endif(UNIX)
endforeach(path ${tests})

add_subdirectory(python)

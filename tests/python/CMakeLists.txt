SET(${PROJECT_NAME}_PYTHON_TESTS
  initialize-euler
  matrix-util
  op-point-modifier
  )
FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "tests/python/${TEST}.py")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

set(${PROJECT_NAME}_PYTHON_TESTS initialize-euler matrix-util op-point-modifier
                                 parameter-server smooth-reach imports)
foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  add_python_unit_test("py-${TEST}" "tests/python/test-${TEST}.py")
endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})

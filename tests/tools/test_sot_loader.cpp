/*
 * License BSD3
 * Copyright 2021,
 * Maximilien Naveau,
 *
 * CNRS
 *
 */

#define BOOST_TEST_MODULE test_sot_loader
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>

#include <boost/test/included/unit_test.hpp>
#include <fstream>
#include <iostream>

#include "sot/core/debug.hh"
#include "sot/core/device.hh"
#include "sot/core/sot-loader.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::sot;
namespace dg = dynamicgraph;

class TestDevice : public dg::sot::Device {
 public:
  TestDevice(const std::string &RobotName) : Device(RobotName) {
    timestep_ = 0.001;
  }
  ~TestDevice() {}
};

struct TestFixture {
  TestFixture() {
    asotei_lib_file_ = LIB_PLUGIN_ABSTRACT_PATH;
    good_python_script_ = TEST_GOOD_PYTHON_SCRIPT;
    bad_python_script_ = TEST_BAD_PYTHON_SCRIPT;
  }
  ~TestFixture() {}
  /// @brief Path to the AbstractSotExternalInterface test library.
  std::string asotei_lib_file_;
  /// @brief Path to a python srcipt to parse without error.
  std::string good_python_script_;
  /// @brief Path to a python srcipt to parse with error.
  std::string bad_python_script_;
  /// @brief Sensor values container.
  std::map<std::string, SensorValues> sensors;
  /// @brief Control values container.
  std::map<std::string, ControlValues> controls;
};

BOOST_AUTO_TEST_SUITE(sot_loader_test_suite)

BOOST_FIXTURE_TEST_CASE(test_plugin_existance, TestFixture) {
  std::ifstream file(asotei_lib_file_);
  bool file_exists = false;
  if (file) {
    file_exists = true;
  }
  BOOST_CHECK(file_exists);
  BOOST_CHECK(asotei_lib_file_ != "");
}

BOOST_FIXTURE_TEST_CASE(test_default_behavior, TestFixture) {
  SotLoader sot_loader;
  sot_loader.initialization();
  BOOST_CHECK(sot_loader.isDynamicGraphStopped());
}

BOOST_FIXTURE_TEST_CASE(test_start_stop_dg, TestFixture) {
  SotLoader sot_loader;
  sot_loader.initialization();
  BOOST_CHECK(sot_loader.isDynamicGraphStopped());
  sot_loader.startDG();
  BOOST_CHECK(!sot_loader.isDynamicGraphStopped());
  sot_loader.stopDG();
  BOOST_CHECK(sot_loader.isDynamicGraphStopped());
}

BOOST_FIXTURE_TEST_CASE(test_option_parsing_input_file, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);

  char argv0[] = "test_sot_loader";
  char argv1[] = "--input-file";
  char argv2[] = LIB_PLUGIN_ABSTRACT_PATH;
  char *argv[] = {argv0, argv1, argv2, NULL};
  sot_loader.parseOptions(3, argv);
  BOOST_CHECK(sot_loader.initialization());
}

BOOST_FIXTURE_TEST_CASE(test_option_parsing_sot_dynamic_library, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);

  char argv0[] = "test_sot_loader";
  char argv1[] = "--sot-dynamic-library";
  char argv2[] = LIB_PLUGIN_ABSTRACT_PATH;
  char *argv[] = {argv0, argv1, argv2, NULL};
  sot_loader.parseOptions(3, argv);
  BOOST_CHECK(sot_loader.initialization());
}

BOOST_FIXTURE_TEST_CASE(test_sot_loader_set_dynamic_library_name, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  BOOST_CHECK(sot_loader.initialization());
}

BOOST_FIXTURE_TEST_CASE(test_sot_loader_one_iteration, TestFixture) {
  std::vector<double> controls_values;
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  // Without running the graph:
  sot_loader.oneIteration(sensors, controls);
  BOOST_CHECK(controls.find("ctrl_map_name") == controls.end());
  // With the graph running:
  sot_loader.startDG();
  std::cout << "running the graph" << std::endl;
  sot_loader.oneIteration(sensors, controls);
  std::cout << "running the graph ... done" << std::endl;
  controls_values = controls["ctrl_map_name"].getValues();
  BOOST_CHECK_EQUAL(controls_values.size(), 5);
  for (auto value : controls_values) {
    BOOST_CHECK_EQUAL(value, 3.1415);
  }
}

BOOST_FIXTURE_TEST_CASE(test_cleanup_no_init, TestFixture) {
  SotLoader sot_loader;
  sot_loader.cleanUp();
}

BOOST_FIXTURE_TEST_CASE(test_cleanup_init, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  sot_loader.cleanUp();
}

BOOST_FIXTURE_TEST_CASE(test_run_python_command, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  std::string res, out, err;
  sot_loader.runPythonCommand("1+1", res, out, err);
  BOOST_CHECK_EQUAL(res, "2");
  BOOST_CHECK_EQUAL(out, "");
  BOOST_CHECK_EQUAL(err, "");
}

BOOST_FIXTURE_TEST_CASE(test_run_python_command_error, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  std::string res, out, err;
  sot_loader.runPythonCommand("print(a)", res, out, err);
  std::cout << std::quoted(err) << std::endl;
  BOOST_CHECK_EQUAL(res, "");
  BOOST_CHECK_EQUAL(out, "");
  BOOST_CHECK_EQUAL(err,
                    "Traceback (most recent call last):\n"
                    "  File \"<string>\", line 1, in <module>\n"
                    "NameError: name 'a' is not defined\n");
}

BOOST_FIXTURE_TEST_CASE(test_run_python_scripts, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  std::string err;
  sot_loader.runPythonFile(good_python_script_, err);
  BOOST_CHECK_EQUAL(err, "");
}

BOOST_FIXTURE_TEST_CASE(test_run_python_scripts_failure, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  std::string err;
  sot_loader.runPythonFile(bad_python_script_, err);
  BOOST_CHECK_EQUAL(err,
                    "Traceback (most recent call last):\n"
                    "  File \"" +
                        bad_python_script_ +
                        "\", line 2, in <module>\n"
                        "    print(b)\n"
                        "NameError: name 'b' is not defined\n");
}

BOOST_FIXTURE_TEST_CASE(test_load_device_in_python, TestFixture) {
  SotLoader sot_loader;
  sot_loader.setDynamicLibraryName(asotei_lib_file_);
  sot_loader.initialization();
  Device device("device_name");
  sot_loader.loadDeviceInPython(device.getName());
  std::string res, out, err;
  sot_loader.runPythonCommand("print(device_cpp_object.name)", res, out, err);
  BOOST_CHECK_EQUAL(res, "None");
  BOOST_CHECK_EQUAL(out, "device_name\n");
  BOOST_CHECK_EQUAL(err, "");
}

BOOST_AUTO_TEST_SUITE_END()

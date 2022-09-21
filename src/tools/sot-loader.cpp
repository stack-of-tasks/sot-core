/*
 * Copyright 2016,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

// POSIX.1-2001
#include <dlfcn.h>

// C++ includes
#include <iostream>
#include <sstream>

// Boost includes
#include <boost/program_options.hpp>

// Dynamic Graph includes
#include <dynamic-graph/pool.h>

// Local includes
#include <sot/core/sot-loader.hh>

namespace po = boost::program_options;

namespace dynamicgraph {
namespace sot {

SotLoader::SotLoader() {
  dynamic_graph_stopped_ = true;
  sot_external_interface_ = nullptr;
  sot_dynamic_library_filename_ = "";
  sot_dynamic_library_ = nullptr;
  device_name_ = "";
}

SotLoader::~SotLoader() { cleanUp(); }

int SotLoader::parseOptions(int argc, char *argv[]) {
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "sot-dynamic-library", po::value<std::string>(), "Library to load");
  desc.add_options()("help", "produce help message")(
      "input-file", po::value<std::string>(), "Library to load");

  // Input variable map from the command line (int argc, char* argv[]).
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return -1;
  }
  if (vm.count("sot-dynamic-library")) {
    sot_dynamic_library_filename_ = vm["sot-dynamic-library"].as<std::string>();
  } else if (vm.count("input-file")) {
    sot_dynamic_library_filename_ = vm["input-file"].as<std::string>();
  } else {
    std::cout << "No filename specified\n";
    return -1;
  }
  return 0;
}

bool SotLoader::initialization() {
  // Load the library containing the AbstractSotExternalInterface.
  sot_dynamic_library_ =
      dlopen(sot_dynamic_library_filename_.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!sot_dynamic_library_) {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    return false;
  }

  // reset errors
  dlerror();

  // Load the symbols.
  createSotExternalInterface_t *createSotExternalInterface =
      reinterpret_cast<createSotExternalInterface_t *>(reinterpret_cast<long>(
          dlsym(sot_dynamic_library_, "createSotExternalInterface")));
  const char *dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return false;
  }

  // Create robot-controller
  sot_external_interface_ = createSotExternalInterface();
  assert(sot_external_interface_ && "Fail to create the sotExternalInterface");
  std::cout << "SoT loaded at address [" << &sot_external_interface_
            << "] from " << sot_dynamic_library_filename_ << "." << std::endl;

  // Init the python interpreter.
  std::string result, out, err;
  // Debug print.
  runPythonCommand("print(\"Executing python interpreter prologue...\")",
                   result, out, err);
  // make sure that the current environment variable are setup in the current
  // python interpreter.
  runPythonCommand("import sys, os", result, out, err);
  runPythonCommand("print(\"python version:\", sys.version)", result, out, err);
  runPythonCommand("pythonpath = os.environ.get('PYTHONPATH', '')", result, out,
                   err);
  runPythonCommand("path = []", result, out, err);
  runPythonCommand(
      "for p in pythonpath.split(':'):\n"
      "  if p not in sys.path:\n"
      "    path.append(p)",
      result, out, err);
  runPythonCommand("path.extend(sys.path)", result, out, err);
  runPythonCommand("sys.path = path", result, out, err);
  // used to be able to invoke rospy
  runPythonCommand(
      "if not hasattr(sys, \'argv\'):\n"
      "    sys.argv  = ['sot']",
      result, out, err);
  // help setting signals
  runPythonCommand("import numpy as np", result, out, err);
  // Debug print.
  runPythonCommand("print(\"Executing python interpreter prologue... Done\")",
                   result, out, err);

  return true;
}

void SotLoader::cleanUp() {
  // Unregister the device first if it exists to avoid a double destruction from
  // the pool of entity and the class that handle the Device pointer.
  if (device_name_ != "") {
    PoolStorage::getInstance()->deregisterEntity(device_name_);
  }

  // We do not destroy the FactoryStorage singleton because the module will not
  // be reloaded at next initialization (because Python C API cannot safely
  // unload a module...).
  // SignalCaster singleton could probably be destroyed.
  dynamicgraph::PoolStorage::destroy();

  // Load the symbols.
  if (sot_dynamic_library_ != nullptr && sot_external_interface_ != nullptr) {
    destroySotExternalInterface_t *destroySotExternalInterface =
        reinterpret_cast<destroySotExternalInterface_t *>(
            reinterpret_cast<long>(
                dlsym(sot_dynamic_library_, "destroySotExternalInterface")));
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
      return;
    }

    destroySotExternalInterface(sot_external_interface_);
    sot_external_interface_ = nullptr;

    /// Uncount the number of access to this library.
    dlclose(sot_dynamic_library_);
    sot_dynamic_library_ = nullptr;
  }
}

void SotLoader::runPythonCommand(const std::string &command,
                                 std::string &result, std::string &out,
                                 std::string &err) {
  embeded_python_interpreter_.python(command, result, out, err);
}

void SotLoader::oneIteration(
    std::map<std::string, SensorValues> &sensors_in,
    std::map<std::string, ControlValues> &control_values) {
  if (!dynamic_graph_stopped_) {
    try {
      sot_external_interface_->nominalSetSensors(sensors_in);
      sot_external_interface_->getControl(control_values);
    } catch (std::exception &e) {
      std::cout << "Exception while running the graph:\n"
                << e.what() << std::endl;
      throw e;
    }
  }
}

void SotLoader::loadDeviceInPython(const std::string &device_name) {
  std::string result, out, err;
  // Debug print.
  runPythonCommand("print(\"Load device from C++ to Python...\")", result, out,
                   err);

  // Import the Device entity declaration
  runPythonCommand("from dynamic_graph.sot.core import Device", result, out,
                   err);

  // Get the existing C++ entity pointer in the Python interpreter.
  runPythonCommand("loaded_device_name = \"" + device_name + "\"", result, out,
                   err);
  runPythonCommand("device_cpp_object = Device(loaded_device_name)", result,
                   out, err);

  // Debug print.
  runPythonCommand("print(\"Load device from C++ to Python... Done!!\")",
                   result, out, err);

  // strore the device name to unregister it upon cleanup.
  device_name_ = device_name;
}

} /* namespace sot */
} /* namespace dynamicgraph */

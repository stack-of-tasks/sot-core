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

#ifndef _SOT_LOADER_HH_
#define _SOT_LOADER_HH_

// STL includes
#include <map>

// Dynamic Graph embeded python interpreter.
#include <dynamic-graph/python/interpreter.hh>

// Sot Framework includes
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>
#include <sot/core/device.hh>

namespace dynamicgraph {
namespace sot {

class SotLoader {
protected:
  /// \brief Check if the dynamic graph is running or not.
  bool dynamic_graph_stopped_;

  /// \brief The interface between the device and the robot driver.
  AbstractSotExternalInterface *sot_external_interface_;

  /// \brief Name of the dynamic library containing the
  /// dgs::AbstractSotExternalInterface object.
  std::string sot_dynamic_library_filename_;

  /// \brief Handle on the SoT library.
  void *sot_dynamic_library_;

  /// \brief Embeded python interpreter.
  python::Interpreter embeded_python_interpreter_;

  /// Map of sensor readings
  std::map<std::string, SensorValues> sensors_in_;

  /// Map of control values
  std::map<std::string, ControlValues> control_values_;

public:
  /// \brief Default constructor.
  SotLoader();
  /// \brief Default destructor.
  ~SotLoader();

  /// \brief Read user input to extract the path of the SoT dynamic library.
  int parseOptions(int argc, char *argv[]);

  /// \brief Prepare the SoT framework.
  void initialization();

  /// \brief Unload the library which handles the robot device.
  void cleanUp();

  /// \brief Get Status of dg.
  inline bool isDynamicGraphStopped() { return dynamic_graph_stopped_; }

  /// \brief Get Status of dg.
  inline void startDG() { dynamic_graph_stopped_ = false; }

  /// \brief Get Status of dg.
  inline void stopDG() { dynamic_graph_stopped_ = true; }

  /// \brief Specify the name of the dynamic library.
  inline void setDynamicLibraryName(std::string &afilename) {
    sot_dynamic_library_filename_ = afilename;
  }

  /// \brief Run a python command inside the embeded python interpreter.
  void runPythonCommand(const std::string &command, std::string &result,
                        std::string &out, std::string &err);

  /// \brief Run a python script inside the embeded python interpreter.
  void runPythonFile(const std::string &ifilename);

  /// \brief Initialize the external interface sensor reading.
  void setupSensors();

  /// \brief Compute one iteration of control.
  /// Basically calls fillSensors, the SoT and the readControl.
  void oneIteration();

  /// \brief Load the Device entity in the python global scope.
  void loadDeviceInPython(const Device &device);

  /// \brief Load the Device entity in the python global scope.
  void loadDeviceInPython(const std::string &device_name);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* _SOT_LOADER_HH_ */

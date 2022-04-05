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

/**
 * @brief This class is loading the control part of the Stack-Of-Tasks.
 * - 1/ It loads dynamically the graph interface.
 * - 2/ It loads the python interpreter.
 * - 3/ It loads the Device entity C++ pointer inside the python interpreter.
 * - 4/ It provides the user interface to the graph:
 *    - 4.1/ starts and stop the graph executtion.
 *    - 4.2/ run a python command/script inside the embeded python interpreter.
 *    - 4.3/ execute one iteration of the graph.
 *
 * In order to Use this class you need to provide a dynamic library containing
 * an implementation of the AbstractSotExternalInterface class.
 *
 * Then you can either inherite from this class an initialize and use the
 * sensors_in_ and control_values_ objects.
 * Or you can create you own outside of this class.
 * And then use the oneIteration to execute the graph.
 */
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

  /// \brief Map of sensor readings
  std::map<std::string, SensorValues> sensors_in_;

  /// \brief Map of control values
  std::map<std::string, ControlValues> control_values_;

  /// \brief Device entity created and loaded, so we deregister it as the Pool
  /// is not responsible for it's life time.
  std::string device_name_;

 public:
  /// \brief Default constructor.
  SotLoader();
  /// \brief Default destructor.
  ~SotLoader();

  /// \brief Read user input to extract the path of the SoT dynamic library.
  int parseOptions(int argc, char *argv[]);

  /// \brief Prepare the SoT framework.
  bool initialization();

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
  inline void runPythonFile(std::string ifilename, std::string &err) {
    embeded_python_interpreter_.runPythonFile(ifilename, err);
  }

  /// \brief Run a python script inside the embeded python interpreter.
  inline void runPythonFile(std::string ifilename) {
    embeded_python_interpreter_.runPythonFile(ifilename);
  }

  /// \brief Compute one iteration of control.
  /// Basically executes fillSensors, the SoT and the readControl.
  void oneIteration(std::map<std::string, SensorValues> &sensors_in,
                    std::map<std::string, ControlValues> &control_values);

  /// \brief Load the Device entity in the python global scope.
  void loadDeviceInPython(const std::string &device_name);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* _SOT_LOADER_HH_ */

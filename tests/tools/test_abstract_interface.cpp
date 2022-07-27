/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <iostream>

// POSIX.1-2001
#include <dlfcn.h>

#include <boost/program_options.hpp>
#include <sot/core/debug.hh>

#include "plugin.hh"

using namespace std;
using namespace dynamicgraph::sot;
namespace po = boost::program_options;

class PluginLoader {
 protected:
  PluginAbstract *sotController_;
  po::variables_map vm_;
  std::string dynamicLibraryName_;

 public:
  PluginLoader(){};
  ~PluginLoader(){};

  int parseOptions(int argc, char *argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "input-file", po::value<string>(), "library to load");

    po::store(po::parse_command_line(argc, argv, desc), vm_);
    po::notify(vm_);

    if (vm_.count("help")) {
      cout << desc << "\n";
      return -1;
    }
    if (!vm_.count("input-file")) {
      cout << "No filename specified\n";
      return -1;
    } else
      dynamicLibraryName_ = vm_["input-file"].as<string>();

    Initialization();
    return 0;
  }

  void Initialization() {
    // Load the SotRobotBipedController library.
    void *SotRobotControllerLibrary =
        dlopen("libpluginabstract.so", RTLD_LAZY | RTLD_LOCAL);
    if (!SotRobotControllerLibrary) {
      std::cerr << "Cannot load library: " << dlerror() << '\n';
      return;
    }

    // reset errors
    dlerror();

    // Load the symbols.
    createPlugin_t *createPlugin =
        (createPlugin_t *)dlsym(SotRobotControllerLibrary, "createPlugin");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
      return;
    }

    // Create robot-controller
    sotController_ = createPlugin();
    //    std::string s="libsot-hrp2-14-controller.so";
    sotController_->Initialization(dynamicLibraryName_);
    cout << "Went out from Initialization." << endl;
  }
};

int main(int argc, char *argv[]) {
  PluginLoader aPluginLoader;
  aPluginLoader.parseOptions(argc, argv);
}

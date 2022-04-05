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

#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>

#include "plugin.hh"

using namespace std;
using namespace dynamicgraph::sot;

class Plugin : public PluginAbstract {
 protected:
  AbstractSotExternalInterface *sotController_;

 public:
  Plugin(){};
  ~Plugin(){};

  void Initialization(std::string &dynamicLibraryName) {
    // Load the SotRobotBipedController library.
    void *SotRobotControllerLibrary =
        dlopen(dynamicLibraryName.c_str(), RTLD_GLOBAL | RTLD_NOW);
    if (!SotRobotControllerLibrary) {
      std::cerr << "Cannot load library: " << dlerror() << '\n';
      return;
    }

    // reset errors
    dlerror();

    // Load the symbols.
    createSotExternalInterface_t *createRobotController =
        (createSotExternalInterface_t *)dlsym(SotRobotControllerLibrary,
                                              "createSotExternalInterface");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
      return;
    }

    /*
    destroySotExternalInterface_t * destroyRobotController =
      (destroySotExternalInterface_t *) dlsym(SotRobotControllerLibrary,
                                              "destroySotExternalInterface");
    */
    dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
      return;
    }

    // Create robot-controller
    sotController_ = createRobotController();
    cout << "Went out from Initialization." << endl;
  }
};

extern "C" {
PluginAbstract *createPlugin() { return new Plugin; }
}

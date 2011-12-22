/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <iostream>

// POSIX.1-2001
#include <dlfcn.h>

#include "plugin.hh"
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot; 

class PluginLoader {

protected:
  PluginAbstract * sotController_;
  
public:
  PluginLoader() {};
  ~PluginLoader() {};
  
  void Initialization(std::string &dynamicLibraryName)
  {
    // Load the SotRobotBipedController library.
    void * SotRobotControllerLibrary = dlopen(dynamicLibraryName.c_str(),
					      RTLD_LAZY | RTLD_LOCAL );
    if (!SotRobotControllerLibrary) {
      std::cerr << "Cannot load library: " << dlerror() << '\n';
      return ;
    }
    
    // reset errors
    dlerror();
    
    // Load the symbols.
    createPlugin_t * createPlugin =
      (createPlugin_t *) dlsym(SotRobotControllerLibrary, 
			       "createPlugin");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
      return ;
    }
    
    // Create robot-controller
    sotController_ = createPlugin();
    std::string s="libsot-hrp2-14-controller.so";
    sotController_->Initialization(s);
    cout <<"Went out from Initialization." << endl;
  }
};

int main(void)
{
  PluginLoader aPluginLoader;
  string soPluginAbstract("libpluginabstract.so");
  aPluginLoader.Initialization(soPluginAbstract);
}

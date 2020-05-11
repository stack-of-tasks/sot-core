/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>
#include <string>

#include "../test-paths.h"
#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/factory.hh>
#include <sot/core/feature-visual-point.hh>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dg;

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef WIN32
typedef HMODULE sotPluginKey;
#else
typedef void *sotPluginKey;
#endif

class TestFeature : public FeatureAbstract {
public:
  TestFeature(void) : FeatureAbstract("") {}
  virtual ~TestFeature(void) {}
  virtual unsigned int &getDimension(unsigned int &res, int /*time*/) {
    return res;
  }

  virtual dg::Vector &computeError(dg::Vector &res, int /*time*/) {
    return res;
  }
  virtual dg::Matrix &computeJacobian(dg::Matrix &res, int /*time*/) {
    return res;
  }
  virtual dg::Vector &computeActivation(dg::Vector &res, int /*time*/) {
    return res;
  }
};

int main() {

  sotDEBUG(0) << "# In {" << endl;
  //   Entity test("");
  //   ExceptionFeature t2(ExceptionFeature::BAD_INIT);
  //   ExceptionSignal t4(ExceptionSignal::COPY_NOT_INITIALIZED);
  //   Flags t3;
  //   TestFeature t5;

#ifdef WIN32
  sotPluginKey dlib = LoadLibrary(PLUGIN_LIB_INSTALL_PATH
                                  "/feature-visual-point" TESTS_DYNLIBSUFFIX);
#else
  sotPluginKey dlib =
      dlopen(PLUGIN_LIB_INSTALL_PATH "/feature-visual-point" TESTS_DYNLIBSUFFIX,
             RTLD_NOW);
#endif

  if (NULL == dlib) {
    cerr << " Error dl" << endl;
#ifndef WIN32
    cerr << dlerror() << endl;
#else
    // Retrieve the system error message for the last-error code
    LPTSTR pszMessage;
    DWORD dwLastError = GetLastError();
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL, dwLastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR)&pszMessage, 0, NULL);

    cerr << pszMessage << endl;
    LocalFree(pszMessage);
#endif

    exit(1);
  }

#ifdef WIN32
  dlib =
      LoadLibrary(PLUGIN_LIB_INSTALL_PATH "/gain-adaptive" TESTS_DYNLIBSUFFIX);
#else
  dlib = dlopen(PLUGIN_LIB_INSTALL_PATH "/gain-adaptive" TESTS_DYNLIBSUFFIX,
                RTLD_NOW);
#endif

  if (NULL == dlib) {
    cerr << " Error dl" << endl;
#ifndef WIN32
    cerr << dlerror() << endl;
#else
    // Retrieve the system error message for the last-error code
    LPTSTR pszMessage;
    DWORD dwLastError = GetLastError();
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL, dwLastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR)&pszMessage, 0, NULL);

    cerr << pszMessage << endl;
    LocalFree(pszMessage);
#endif
    exit(1);
  }

  Entity *gain =
      FactoryStorage::getInstance()->newEntity("GainAdaptive", "Gain");
  FeatureAbstract *point =
      sotFactory.newFeature("FeatureVisualPoint", "DynamicTest.");

  try {
    gain->display(cout);
    cout << endl;
    cout << gain->getClassName();
    cout << endl;

    point->display(cout);
    cout << endl;
    cout << point->getClassName();
    cout << endl;
  } catch (ExceptionSignal e) {
    cout << "Exception caught! " << e << endl;
  }

  sotDEBUG(0) << "# Out }" << endl;
}

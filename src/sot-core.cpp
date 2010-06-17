/*
 *  Copyright
 */

#include <iostream>
#include "sot-core/sot-core.hh" 

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cerr << "sot-core:" << x << std::endl
#define ODEBUG1(x) std::cerr << "sot-core:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "sot-core:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

using namespace sot;

Core::Core()
{
  ODEBUG2("message of level 2");
  ODEBUG1("message of level 1");
}

Core::~Core()
{
}

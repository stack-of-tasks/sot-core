/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* SOT */
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-base.h>

#include <sot/core/exception-factory.hh>
#include <sot/core/pool.hh>

#include "sot/core/api.hh"

/* --- STD --- */
#include <map>
#include <sstream>
#include <string>

/* --- BOOST --- */
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace dynamicgraph {
namespace sot {

/*! @ingroup factory
  \brief This helper class dynamically overloads the "new" shell command
  to allow creation of tasks and features as well as entities.
 */
class AdditionalFunctions {
 public:
  AdditionalFunctions();
  ~AdditionalFunctions();
  static void cmdFlagSet(const std::string &cmd, std::istringstream &args,
                         std::ostream &os);
};

} /* namespace sot */
} /* namespace dynamicgraph */

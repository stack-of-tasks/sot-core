/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
 *
 * CNRS
 *
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include "sot/core/robot-simu.hh"

namespace dynamicgraph {
namespace sot {
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RobotSimu, "RobotSimu");

RobotSimu::RobotSimu(const std::string& inName) :
  Device(inName) {
  using namespace dynamicgraph::command;
  std::string docstring;
  /* Command increment. */
  addCommand("increment",
             command::makeCommandVoid0((Device&)*this,
                                       &Device::increment,
                                       command::docCommandVoid0("Increment the dynamic: check the control and run synchronous commands")));

  /* Set Time step. */
  docstring =
    "\n"
    "    Set the time step provided\n"
    "\n"
    "    take one floating point number as input\n"
    "\n";
  addCommand("setTimeStep",
             makeDirectSetter (*this, &this->timestep_,
                               docstring));


}
} // namespace sot
} // namespace dynamicgraph

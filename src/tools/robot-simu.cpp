/*
 * Copyright 2010,
 * Nicolas Mansard, Olivier Stasse, François Bleibel, Florent Lamiraux
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

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include "sot/core/robot-simu.hh"

namespace dynamicgraph {
  namespace sot {
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RobotSimu,"RobotSimu");

    RobotSimu::RobotSimu(const std::string& inName) :
      Device(inName)
    {
      using namespace dynamicgraph::command;
      std::string docstring;
      /* Command increment. */
      docstring =
	"\n"
	"    Integrate dynamics for time step provided as input\n"
	"\n"
	"      take one floating point number as input\n"
	"\n";
      addCommand("increment",
		 command::makeCommandVoid1((Device&)*this,
					   &Device::increment, docstring));
      
    }
  } // namespace sot
} // namespace dynamicgraph

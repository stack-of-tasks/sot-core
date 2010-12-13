/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS/AIST
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

#ifndef ROBOT_SIMU_COMMAND_H
 #define ROBOT_SIMU_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace sot {
  namespace command {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;
  
    // Command Increment
    class Increment : public Command
    {
    public:
      virtual ~Increment() {}
      /// Create command and store it in Entity
      /// \param entity instance of Entity owning this command
      /// \param docstring documentation of the command
      Increment(RobotSimu& entity, const std::string& docstring) :
	command(entity, boost::assign::list_of(Value::DOUBLE), docstring)
      {
      }
      virtual Value doExecute()
      {
	RobotSimu& rs = static_cast<RobotSimu&>(owner());
	std::vector<Value> values = getParameterValues();
	double timeStep = values[0].value();
	rs.increment(timeStep);
	// return void
	return Value();
      }
    }; // class Increment
  } // namespace command
} //namespace sot

#endif //ROBOT_SIMU_COMMAND_H

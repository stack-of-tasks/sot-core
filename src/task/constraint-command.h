/*
 * Copyright 2010,
 * Florent Lamiraux
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

#ifndef CONSTRAINT_COMMAND_H
 #define CONSTRAINT_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace dynamicgraph {
  namespace sot {
    namespace command {
      namespace constraint {
	using ::dynamicgraph::command::Command;
	using ::dynamicgraph::command::Value;
	
	// Command AddJacobian
	class AddJacobian : public Command
	{
	public:
	  virtual ~AddJacobian() {}
	  /// Create command and store it in Entity
	  /// \param entity instance of Entity owning this command
	  /// \param docstring documentation of the command
	AddJacobian(Constraint& entity, const std::string& docstring) :
	  Command(entity, boost::assign::list_of(Value::STRING), docstring)
	    {
	    }
	  virtual Value doExecute()
	  {
	    Constraint& constraint = static_cast<Constraint&>(owner());
	    std::vector<Value> values = getParameterValues();
	    std::string signalName = values[0].value();
	    std::istringstream iss(signalName);
	    SignalBase<int>& signal = g_pool.getSignal(iss);
	    try {
	      Signal< ml::Matrix,int >& matrixSignal
		= dynamic_cast< Signal<ml::Matrix,int>& >( signal );
	      constraint.addJacobian(matrixSignal);
	    } catch (const std::bad_cast& exc) {
	      std::string msg(signalName + " is not of type matrix.");
	      SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST, msg);
	    }
	    // return void
	    return Value();
	  }
	}; // class AddJacobian
      } // namespace constraint
    } // namespace command
  } // namespace sot
} // namespace dynamicgraph

#endif //CONSTRAINT_COMMAND_H

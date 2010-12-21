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

#ifndef SOT_COMMAND_H
 #define SOT_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace sot {
  namespace command {
    namespace classSot {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;
      
      // Command AddConstraint
      class AddConstraint : public Command
      {
      public:
	virtual ~AddConstraint() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      AddConstraint(Sot& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  Sot& sot = static_cast<Sot&>(owner());
	  std::vector<Value> values = getParameterValues();
	  std::string constraintName = values[0].value();

	  Constraint& constraint =
	    dynamic_cast<Constraint&>(sotPool.getTask(constraintName));
	  sot.addConstraint(constraint);
	  sot.constraintSOUT.setReady();
	  // return void
	  return Value();
	}
      }; // class AddConstraint

      // Command Push
      class Push : public Command
      {
      public:
	virtual ~Push() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Push(Sot& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  Sot& sot = static_cast<Sot&>(owner());
	  std::vector<Value> values = getParameterValues();
	  std::string taskName = values[0].value();

	  TaskAbstract& task = sotPool.getTask(taskName);
	  sot.push(task);
	  // return void
	  return Value();
	}
      }; // class Push
    } // namespace classSot
  } // namespace command
} //namespace sot

#endif //SOT_COMMAND_H

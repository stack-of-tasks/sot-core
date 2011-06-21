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

namespace dynamicgraph { namespace sot {
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
	    dynamic_cast<Constraint&>(PoolStorage::getInstance()->
				      getTask(constraintName));
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

	  TaskAbstract& task = PoolStorage::getInstance()->getTask(taskName);
	  sot.push(task);
	  // return void
	  return Value();
	}
      }; // class Push

      // Command Remove
      class Remove : public Command
      {
      public:
	virtual ~Remove() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Remove(Sot& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  Sot& sot = static_cast<Sot&>(owner());
	  std::vector<Value> values = getParameterValues();
	  std::string taskName = values[0].value();

	  TaskAbstract& task = PoolStorage::getInstance()->getTask(taskName);
	  sot.remove(task);
	  // return void
	  return Value();
	}
      }; // class Remove

      // Command Up
      class Up : public Command
      {
      public:
	virtual ~Up() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Up(Sot& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  Sot& sot = static_cast<Sot&>(owner());
	  std::vector<Value> values = getParameterValues();
	  std::string taskName = values[0].value();

	  TaskAbstract& task = PoolStorage::getInstance()->getTask(taskName);
	  sot.up(task);
	  // return void
	  return Value();
	}
      }; // class Remove

      // Command Down
      class Down : public Command
      {
      public:
	virtual ~Down() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Down(Sot& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::STRING), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  Sot& sot = static_cast<Sot&>(owner());
	  std::vector<Value> values = getParameterValues();
	  std::string taskName = values[0].value();

	  TaskAbstract& task = PoolStorage::getInstance()->getTask(taskName);
	  sot.down(task);
	  // return void
	  return Value();
	}
      }; // class Down

      // Command Display
      class Display : public Command
      {
      public:
	virtual ~Display() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Display(Sot& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type> (), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  std::stringstream returnString;
	  Sot& sot = static_cast<Sot&>(owner());
	  sot.display (returnString);

	  // return the stack
	  return Value(returnString.str());
	}
      }; // class Display

      // Command Clear
      class Clear : public Command
      {
      public:
	virtual ~Clear() {}
	/// Clear the stack
	/// \param docstring documentation of the command
      Clear(Sot& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type> (), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  std::stringstream returnString;
	  Sot& sot = static_cast<Sot&>(owner());
          sot.clear();
	  // return the stack
	  return Value();
	}
      }; // class Clear


    } // namespace classSot
  } // namespace command
} /* namespace sot */} /* namespace dynamicgraph */

#endif //SOT_COMMAND_H

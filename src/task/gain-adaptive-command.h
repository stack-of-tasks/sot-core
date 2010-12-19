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

#ifndef GAIN_ADAPTIVE_COMMAND_H
 #define GAIN_ADAPTIVE_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace sot {
  namespace command {
    namespace gainAdaptive {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;
      
      // Command SetConstant
      class SetConstant : public Command
      {
      public:
	virtual ~SetConstant() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      SetConstant(GainAdaptive& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::DOUBLE), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  GainAdaptive& gainAdaptive = static_cast<GainAdaptive&>(owner());
	  std::vector<Value> values = getParameterValues();
	  double gain = values[0].value();
	  gainAdaptive.init(gain);
	  // return void
	  return Value();
	}
      }; // class SetConstant

      // Command Set
      class Set : public Command
      {
      public:
	virtual ~Set() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Set(GainAdaptive& entity, const std::string& docstring) :
	Command(entity, boost::assign::list_of(Value::DOUBLE)(Value::DOUBLE)
		(Value::DOUBLE), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  GainAdaptive& gainAdaptive = static_cast<GainAdaptive&>(owner());
	  std::vector<Value> values = getParameterValues();
	  double c0 = values[0].value();
	  double cinf = values[1].value();
	  double p0 = values[2].value();
	  gainAdaptive.init(c0, cinf, p0);
	  // return void
	  return Value();
	}
      }; // class Set
    } // namespace gainAdaptive
  } // namespace command
} //namespace sot

#endif //GAIN_ADAPTIVE_COMMAND_H

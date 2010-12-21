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

#ifndef FEATURE_JOINT_LIMITS_COMMAND_H
 #define FEATURE_JOINT_LIMITS_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace sot {
  namespace command {
    namespace featureJointLimits {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;
      
      // Command Actuate
      class Actuate : public Command
      {
      public:
	virtual ~Actuate() {}
	/// Create command and store it in Entity
	/// \param entity instance of Entity owning this command
	/// \param docstring documentation of the command
      Actuate(FeatureJointLimits& entity, const std::string& docstring) :
	Command(entity, std::vector<Value::Type>(), docstring)
	  {
	  }
	virtual Value doExecute()
	{
	  FeatureJointLimits& fjl = static_cast<FeatureJointLimits&>(owner());
	  Flags fl( 63 ); //0x0000003f = 00000000000000000000000000111111
	  fjl.selectionSIN =  (! fl);
	  // return void
	  return Value();
	}
      }; // class Actuate
    } // namespace featureJointLimits
  } // namespace command
} //namespace sot

#endif //FEATURE_JOINT_LIMITS_COMMAND_H

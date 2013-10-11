//
// Copyright 2013,
// Olivier Stasse
//
// CNRS
//
// This file is part of sot-core.
// sot-core is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-core.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef JOINT_TRAJECTORY_COMMAND_H_
#define JOINT_TRAJECTORY_COMMAND_H_

#include <boost/assign/list_of.hpp>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

#include <sot/core/joint-trajectory-entity.hh>

namespace dynamicgraph{ namespace sot {
    namespace command { namespace classSot {
        using ::dynamicgraph::command::Command;
        using ::dynamicgraph::command::Value;

        class SetInitTrajectory : public Command 
        {
        public:
          virtual ~SetInitTrajectory() {}

          /// Set the initial trajectory.
          SetInitTrajectory(SotJointTrajectoryEntity & entity,
                            const std::string &docstring) :
            Command(entity, 
                    boost::assign::list_of(Value::STRING),
                    docstring)
          {
          }

          virtual Value doExecute()
          {

            // return void
            return Value();
          }
        };
      } // namespace classSot
    } // namespace command
  } // namespace sot 
} // namespace dynamicgraph
#endif /* JOINT_TRAJECTORY_COMMAND_H_ */

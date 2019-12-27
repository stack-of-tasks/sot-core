//
// Copyright 2013,
// Olivier Stasse
//
// CNRS
//

#ifndef JOINT_TRAJECTORY_COMMAND_H_
#define JOINT_TRAJECTORY_COMMAND_H_

#include <boost/assign/list_of.hpp>

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <sot/core/joint-trajectory-entity.hh>

namespace dynamicgraph {
namespace sot {
namespace command {
namespace classSot {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

class SetInitTrajectory : public Command {
public:
  virtual ~SetInitTrajectory() {}

  /// Set the initial trajectory.
  SetInitTrajectory(SotJointTrajectoryEntity &entity,
                    const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::STRING), docstring) {}

  virtual Value doExecute() {

    // return void
    return Value();
  }
};
} // namespace classSot
} // namespace command
} // namespace sot
} // namespace dynamicgraph
#endif /* JOINT_TRAJECTORY_COMMAND_H_ */

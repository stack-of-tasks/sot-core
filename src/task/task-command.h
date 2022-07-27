/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS
 *
 */

#ifndef TASK_COMMAND_H
#define TASK_COMMAND_H

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
namespace sot {
namespace command {
namespace task {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command ListFeatures
class ListFeatures : public Command {
 public:
  virtual ~ListFeatures() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  ListFeatures(Task &entity, const std::string &docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    typedef Task::FeatureList_t FeatureList_t;
    Task &task = static_cast<Task &>(owner());
    const FeatureList_t &fl = task.getFeatureList();
    std::string result("[");
    for (FeatureList_t::const_iterator it = fl.begin(); it != fl.end(); it++) {
      result += "'" + (*it)->getName() + "',";
    }
    result += "]";
    return Value(result);
  }
};  // class ListFeatures
}  // namespace task
}  // namespace command
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // TASK_COMMAND_H

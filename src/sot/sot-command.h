/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS
 *
 */

#ifndef SOT_COMMAND_H
#define SOT_COMMAND_H

#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
namespace sot {
namespace command {
namespace classSot {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;

// Command Push
class Push : public Command {
 public:
  virtual ~Push() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Push(Sot &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::STRING), docstring) {}
  virtual Value doExecute() {
    Sot &sot = static_cast<Sot &>(owner());
    std::vector<Value> values = getParameterValues();
    std::string taskName = values[0].value();

    TaskAbstract &task = PoolStorage::getInstance()->getTask(taskName);
    sot.push(task);
    // return void
    return Value();
  }
};  // class Push

// Command Remove
class Remove : public Command {
 public:
  virtual ~Remove() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Remove(Sot &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::STRING), docstring) {}
  virtual Value doExecute() {
    Sot &sot = static_cast<Sot &>(owner());
    std::vector<Value> values = getParameterValues();
    std::string taskName = values[0].value();

    TaskAbstract &task = PoolStorage::getInstance()->getTask(taskName);
    sot.remove(task);
    // return void
    return Value();
  }
};  // class Remove

// Command Up
class Up : public Command {
 public:
  virtual ~Up() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Up(Sot &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::STRING), docstring) {}
  virtual Value doExecute() {
    Sot &sot = static_cast<Sot &>(owner());
    std::vector<Value> values = getParameterValues();
    std::string taskName = values[0].value();

    TaskAbstract &task = PoolStorage::getInstance()->getTask(taskName);
    sot.up(task);
    // return void
    return Value();
  }
};  // class Remove

// Command Down
class Down : public Command {
 public:
  virtual ~Down() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Down(Sot &entity, const std::string &docstring)
      : Command(entity, boost::assign::list_of(Value::STRING), docstring) {}
  virtual Value doExecute() {
    Sot &sot = static_cast<Sot &>(owner());
    std::vector<Value> values = getParameterValues();
    std::string taskName = values[0].value();

    TaskAbstract &task = PoolStorage::getInstance()->getTask(taskName);
    sot.down(task);
    // return void
    return Value();
  }
};  // class Down

// Command Display
class Display : public Command {
 public:
  virtual ~Display() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  Display(Sot &entity, const std::string &docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    std::stringstream returnString;
    Sot &sot = static_cast<Sot &>(owner());
    sot.display(returnString);

    // return the stack
    return Value(returnString.str());
  }
};  // class Display

// Command List
class List : public Command {
 public:
  virtual ~List() {}
  /// Create command and store it in Entity
  /// \param entity instance of Entity owning this command
  /// \param docstring documentation of the command
  List(Sot &entity, const std::string &docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    Sot &sot = static_cast<Sot &>(owner());
    typedef Sot::StackType StackType;
    const StackType &stack = sot.tasks();

    std::stringstream returnString;
    returnString << "( ";
    for (StackType::const_iterator it = stack.begin(); it != stack.end();
         ++it) {
      returnString << '"' << (*it)->getName() << "\", ";
    }
    returnString << ")";

    // return the stack
    return Value(returnString.str());
  }
};  // class List

// Command Clear
class Clear : public Command {
 public:
  virtual ~Clear() {}
  /// Clear the stack
  /// \param docstring documentation of the command
  Clear(Sot &entity, const std::string &docstring)
      : Command(entity, std::vector<Value::Type>(), docstring) {}
  virtual Value doExecute() {
    std::stringstream returnString;
    Sot &sot = static_cast<Sot &>(owner());
    sot.clear();
    // return the stack
    return Value();
  }
};  // class Clear

}  // namespace classSot
}  // namespace command
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // SOT_COMMAND_H

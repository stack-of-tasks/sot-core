/*
 * Copyright 2011,
 * Olivier Stasse, CNRS
 *
 * CNRS
 *
 */

#ifndef ABSTRACT_SOT_EXTERNAL_INTERFACE_HH
#define ABSTRACT_SOT_EXTERNAL_INTERFACE_HH

#include <map>
#include <sot/core/api.hh>
#include <sot/core/fwd.hh>
#include <string>
#include <vector>

namespace dynamicgraph {
namespace sot {

class SOT_CORE_EXPORT NamedVector {
 private:
  std::string name_;
  std::vector<double> values_;

 public:
  NamedVector() {}
  ~NamedVector() {}

  const std::string &getName() const { return name_; }

  void setName(const std::string &aname) { name_ = aname; }

  const std::vector<double> &getValues() const { return values_; }

  void setValues(const std::vector<double> &values) { values_ = values; }
};
typedef NamedVector SensorValues;
typedef NamedVector ControlValues;

class SOT_CORE_EXPORT AbstractSotExternalInterface {
 public:
  AbstractSotExternalInterface() {}

  virtual ~AbstractSotExternalInterface() {}

  virtual void setupSetSensors(
      std::map<std::string, SensorValues> &sensorsIn) = 0;

  virtual void nominalSetSensors(
      std::map<std::string, SensorValues> &sensorsIn) = 0;

  virtual void cleanupSetSensors(
      std::map<std::string, SensorValues> &sensorsIn) = 0;

  // Get control vector
  // \param map map string to vector of doubles. This method is expected to
  //        fill in entry "control"
  // \param period time since last call.
  virtual void getControl(std::map<std::string, ControlValues> &,
                          const double &period = 0) = 0;
  virtual void setSecondOrderIntegration(void) = 0;
  virtual void setNoIntegration(void) = 0;
  // Set the number of joints that are controlled
  virtual void setControlSize(const size_type &) = 0;
};
}  // namespace sot
}  // namespace dynamicgraph

typedef dynamicgraph::sot::AbstractSotExternalInterface *
createSotExternalInterface_t();
typedef void destroySotExternalInterface_t(
    dynamicgraph::sot::AbstractSotExternalInterface *);

#endif  // ABSTRACT_SOT_EXTERNAL_INTERFACE_HH

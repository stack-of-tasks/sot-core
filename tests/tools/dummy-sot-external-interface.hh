/**
 * @file abstract-sot-external-interface-tester.hh
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-11-17
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef ABSTRACT_SOT_EXTERNAL_INTERFACE_TESTER_HH
#define ABSTRACT_SOT_EXTERNAL_INTERFACE_TESTER_HH

#include <sot/core/abstract-sot-external-interface.hh>

class DummySotExternalInterface
    : public dynamicgraph::sot::AbstractSotExternalInterface {
 public:
  DummySotExternalInterface(){};

  virtual ~DummySotExternalInterface(){};

  virtual void setupSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &sensorsIn);

  virtual void nominalSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &sensorsIn);

  virtual void cleanupSetSensors(
      std::map<std::string, dynamicgraph::sot::SensorValues> &sensorsIn);

  virtual void getControl(
      std::map<std::string, dynamicgraph::sot::ControlValues> &controlOut);
  virtual void setSecondOrderIntegration(void);
  virtual void setNoIntegration(void);

 public:
  bool second_integration_;
};

#endif /* ABSTRACT_SOT_EXTERNAL_INTERFACE_TESTER_HH */

/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include "dummy-sot-external-interface.hh"

#include <iostream>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot;

void DummySotExternalInterface::setupSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> &sensorsIn) {
  nominalSetSensors(sensorsIn);
  return;
}

void DummySotExternalInterface::nominalSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> & /*sensorsIn*/) {
  return;
}

void DummySotExternalInterface::cleanupSetSensors(
    std::map<std::string, dynamicgraph::sot::SensorValues> &sensorsIn) {
  nominalSetSensors(sensorsIn);
  return;
}

void DummySotExternalInterface::getControl(
    std::map<std::string, dynamicgraph::sot::ControlValues> &controlOut) {
  controlOut["ctrl_map_name"] = dynamicgraph::sot::ControlValues();
  controlOut["ctrl_map_name"].setName("ctrl_value_name");
  controlOut["ctrl_map_name"].setValues(std::vector<double>(5, 3.1415));
  return;
}

void DummySotExternalInterface::setSecondOrderIntegration(void) {
  second_integration_ = true;
  return;
}

void DummySotExternalInterface::setNoIntegration(void) {
  second_integration_ = false;
  return;
}

extern "C" {
dynamicgraph::sot::AbstractSotExternalInterface *createSotExternalInterface() {
  return new DummySotExternalInterface();
}
}

extern "C" {
void destroySotExternalInterface(
    dynamicgraph::sot::AbstractSotExternalInterface *p) {
  delete p;
}
}

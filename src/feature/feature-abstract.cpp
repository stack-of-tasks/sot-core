/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <iostream>

#include "sot/core/debug.hh"
#include "sot/core/exception-feature.hh"
#include <dynamic-graph/all-commands.h>
#include <sot/core/feature-abstract.hh>
#include <sot/core/pool.hh>

using namespace dynamicgraph::sot;
using dynamicgraph::sot::ExceptionFeature;

const std::string FeatureAbstract::CLASS_NAME = "FeatureAbstract";

FeatureAbstract::FeatureAbstract(const std::string &name)
    : Entity(name), selectionSIN(NULL, "sotFeatureAbstract(" + name +
                                           ")::input(flag)::selec"),
      errordotSIN(NULL, "sotFeatureAbstract(" + name +
                            ")::input(vector)::errordotIN"),
      errorSOUT(boost::bind(&FeatureAbstract::computeError, this, _1, _2),
                selectionSIN,
                "sotFeatureAbstract(" + name + ")::output(vector)::error"),
      errordotSOUT(boost::bind(&FeatureAbstract::computeErrorDot, this, _1, _2),
                   selectionSIN << errordotSIN,
                   "sotFeatureAbstract(" + name + ")::output(vector)::errordot")

      ,
      jacobianSOUT(boost::bind(&FeatureAbstract::computeJacobian, this, _1, _2),
                   selectionSIN,
                   "sotFeatureAbstract(" + name +
                       ")::output(matrix)::jacobian"),
      dimensionSOUT(boost::bind(&FeatureAbstract::getDimension, this, _1, _2),
                    selectionSIN,
                    "sotFeatureAbstract(" + name + ")::output(uint)::dim") {
  selectionSIN = true;
  signalRegistration(selectionSIN << errorSOUT << jacobianSOUT
                                  << dimensionSOUT);
  featureRegistration();
  initCommands();
}

void FeatureAbstract::initCommands(void) {
  using namespace command;
  addCommand("setReference",
             new dynamicgraph::command::Setter<FeatureAbstract, std::string>(
                 *this, &FeatureAbstract::setReferenceByName,
                 "Give the name of the reference feature.\nInput: a string "
                 "(feature name)."));
  addCommand("getReference",
             new dynamicgraph::command::Getter<FeatureAbstract, std::string>(
                 *this, &FeatureAbstract::getReferenceByName,
                 "Get the name of the reference feature.\nOutput: a string "
                 "(feature name)."));
}

void FeatureAbstract::featureRegistration(void) {
  PoolStorage::getInstance()->registerFeature(name, this);
}

std::ostream &FeatureAbstract::writeGraph(std::ostream &os) const {
  Entity::writeGraph(os);

  if (isReferenceSet()) {
    const FeatureAbstract *asotFA = getReferenceAbstract();
    os << "\t\"" << asotFA->getName() << "\" -> \"" << getName() << "\""
       << "[ color=darkseagreen4 ]" << std::endl;
  } else
    std::cout << "asotFAT : 0" << std::endl;

  return os;
}

void FeatureAbstract::setReferenceByName(const std::string &name) {
  setReference(
      &dynamicgraph::sot::PoolStorage::getInstance()->getFeature(name));
}

std::string FeatureAbstract::getReferenceByName() const {
  if (isReferenceSet())
    return getReferenceAbstract()->getName();
  else
    return "none";
}

dynamicgraph::Vector &
FeatureAbstract::computeErrorDot(dynamicgraph::Vector &res, int time) {
  const Flags &fl = selectionSIN.access(time);
  const int &dim = dimensionSOUT(time);

  unsigned int curr = 0;
  res.resize(dim);

  sotDEBUG(25) << "Dim = " << dim << std::endl;

  if (isReferenceSet() && getReferenceAbstract()->errordotSIN.isPlugged()) {
    const dynamicgraph::Vector &errdotDes =
        getReferenceAbstract()->errordotSIN(time);
    sotDEBUG(15) << "Err* = " << errdotDes;
    if (errdotDes.size() < dim) {
      SOT_THROW ExceptionFeature(
          ExceptionFeature::UNCOMPATIBLE_SIZE,
          "Error: dimension uncompatible with des->errorIN size."
          " (while considering feature <%s>).",
          getName().c_str());
    }

    for (int i = 0; i < errdotDes.size(); ++i)
      if (fl(i))
        res(curr++) = -errdotDes(i);
  } else
    res.setZero();

  return res;
}

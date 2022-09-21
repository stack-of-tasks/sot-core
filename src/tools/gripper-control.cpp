/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#define ENABLE_RT_LOG

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/real-time-logger.h>

#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/gripper-control.hh>
#include <sot/core/macros-signal.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GripperControlPlugin, "GripperControl");

/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */
/* --- PLUGIN --------------------------------------------------------------- */

#define SOT_FULL_TO_REDUCED(sotName)                                          \
  sotName##FullSizeSIN(NULL, "GripperControl(" + name +                       \
                                 ")::input(vector)::" + #sotName + "FullIN"), \
      sotName##ReduceSOUT(                                                    \
          SOT_INIT_SIGNAL_2(GripperControlPlugin::selector,                   \
                            sotName##FullSizeSIN, dynamicgraph::Vector,       \
                            selectionSIN, Flags),                             \
          "GripperControl(" + name + ")::input(vector)::" + #sotName +        \
              "ReducedOUT")

const double GripperControl::OFFSET_DEFAULT = 0.9;

// TODO: hard coded
const double DT = 0.005;

GripperControl::GripperControl(void)
    : offset(GripperControl::OFFSET_DEFAULT), factor() {}

GripperControlPlugin::GripperControlPlugin(const std::string &name)
    : Entity(name),
      calibrationStarted(false),
      positionSIN(NULL,
                  "GripperControl(" + name + ")::input(vector)::position"),
      positionDesSIN(
          NULL, "GripperControl(" + name + ")::input(vector)::positionDes"),
      torqueSIN(NULL, "GripperControl(" + name + ")::input(vector)::torque"),
      torqueLimitSIN(
          NULL, "GripperControl(" + name + ")::input(vector)::torqueLimit"),
      selectionSIN(NULL, "GripperControl(" + name + ")::input(vector)::selec")

      ,
      SOT_FULL_TO_REDUCED(position),
      SOT_FULL_TO_REDUCED(torque),
      SOT_FULL_TO_REDUCED(torqueLimit),
      desiredPositionSOUT(
          SOT_MEMBER_SIGNAL_4(GripperControl::computeDesiredPosition,
                              positionSIN, dynamicgraph::Vector, positionDesSIN,
                              dynamicgraph::Vector, torqueSIN,
                              dynamicgraph::Vector, torqueLimitSIN,
                              dynamicgraph::Vector),
          "GripperControl(" + name + ")::output(vector)::reference") {
  sotDEBUGIN(5);

  positionSIN.plug(&positionReduceSOUT);
  torqueSIN.plug(&torqueReduceSOUT);
  torqueLimitSIN.plug(&torqueLimitReduceSOUT);

  signalRegistration(
      positionSIN << positionDesSIN << torqueSIN << torqueLimitSIN
                  << selectionSIN << desiredPositionSOUT << positionFullSizeSIN
                  << torqueFullSizeSIN << torqueLimitFullSizeSIN);
  sotDEBUGOUT(5);

  initCommands();
}

GripperControlPlugin::~GripperControlPlugin(void) {}

std::string GripperControlPlugin::getDocString() const {
  std::string docstring = "Control of gripper.";
  return docstring;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

void GripperControl::computeIncrement(
    const dynamicgraph::Vector &torques,
    const dynamicgraph::Vector &torqueLimits,
    const dynamicgraph::Vector &currentNormVel) {
  const dynamicgraph::Vector::Index SIZE = currentNormVel.size();

  // initialize factor, if needed.
  if (factor.size() != SIZE) {
    factor.resize(SIZE);
    factor.fill(1.);
  }

  // Torque not provided?
  if (torques.size() == 0) {
    dgRTLOG() << "torque is not provided " << std::endl;
    return;
  }

  for (int i = 0; i < SIZE; ++i) {
    // apply a reduction factor if the torque limits are exceeded
    // and the velocity goes in the same way
    if ((torques(i) > torqueLimits(i)) && (currentNormVel(i) > 0)) {
      factor(i) *= offset;
    } else if ((torques(i) < -torqueLimits(i)) && (currentNormVel(i) < 0)) {
      factor(i) *= offset;
    }
    // otherwise, release smoothly the reduction if possible/needed
    else {
      factor(i) /= offset;
    }

    // ensure factor is in )0,1(
    factor(i) = std::min(1., std::max(factor(i), 0.));
  }
}

dynamicgraph::Vector &GripperControl::computeDesiredPosition(
    const dynamicgraph::Vector &currentPos,
    const dynamicgraph::Vector &desiredPos, const dynamicgraph::Vector &torques,
    const dynamicgraph::Vector &torqueLimits,
    dynamicgraph::Vector &referencePos) {
  const dynamicgraph::Vector::Index SIZE = currentPos.size();
  //  if( (SIZE==torques.size()) )
  //    { /* ERROR ... */ }

  // compute the desired velocity
  dynamicgraph::Vector velocity = (desiredPos - currentPos) * (1. / DT);

  computeIncrement(torques, torqueLimits, velocity);

  sotDEBUG(25) << " velocity " << velocity << std::endl;
  sotDEBUG(25) << " factor " << factor << std::endl;

  // multiply the velocity elmt per elmt
  dynamicgraph::Vector weightedVel(SIZE);
  weightedVel = velocity * factor;
  sotDEBUG(25) << " weightedVel " << weightedVel << std::endl;

  // integrate the desired velocity
  referencePos.resize(SIZE);
  referencePos = currentPos + weightedVel * DT;
  return referencePos;
}

dynamicgraph::Vector &GripperControl::selector(
    const dynamicgraph::Vector &fullsize, const Flags &selec,
    dynamicgraph::Vector &desPos) {
  int size = 0;
  for (int i = 0; i < fullsize.size(); ++i) {
    if (selec(i)) size++;
  }

  int curs = 0;
  desPos.resize(size);
  for (int i = 0; i < fullsize.size(); ++i) {
    if (selec(i)) desPos(curs++) = fullsize(i);
  }

  return desPos;
}

/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */
/* --- COMMANDLINE ---------------------------------------------------------- */
void GripperControlPlugin::initCommands() {
  namespace dc = ::dynamicgraph::command;
  addCommand("offset",
             dc::makeCommandVoid1(*this, &GripperControlPlugin::setOffset,
                                  "set the offset (should be in )0, 1( )."));
}

void GripperControlPlugin::setOffset(const double &value) {
  if ((value > 0) && (value < 1))
    offset = value;
  else
    throw std::invalid_argument("The offset should be in )0, 1(.");
}

/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * Noëlie Ramuzat
 * This file is part of sot-core.
 * See license file.
 */

#include "sot/core/admittance-control-op-point.hh"

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace core {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace pinocchio;
using namespace dg::command;

#define PROFILE_ADMITTANCECONTROLOPPOINT_WFORCE_COMPUTATION \
  "AdmittanceControlOpPoint: w_force computation   "

#define PROFILE_ADMITTANCECONTROLOPPOINT_WDQ_COMPUTATION \
  "AdmittanceControlOpPoint: w_dq computation      "

#define PROFILE_ADMITTANCECONTROLOPPOINT_DQ_COMPUTATION \
  "AdmittanceControlOpPoint: dq computation        "

#define INPUT_SIGNALS                                                      \
  m_KpSIN << m_KdSIN << m_dqSaturationSIN << m_forceSIN << m_w_forceDesSIN \
          << m_opPoseSIN << m_sensorPoseSIN

#define INNER_SIGNALS m_w_forceSINNER << m_w_dqSINNER

#define OUTPUT_SIGNALS m_dqSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef AdmittanceControlOpPoint EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AdmittanceControlOpPoint,
                                   "AdmittanceControlOpPoint");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
AdmittanceControlOpPoint::AdmittanceControlOpPoint(const std::string &name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(Kp, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(Kd, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(dqSaturation, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(force, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(w_forceDes, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(opPose, dynamicgraph::sot::MatrixHomogeneous),
      CONSTRUCT_SIGNAL_IN(sensorPose, dynamicgraph::sot::MatrixHomogeneous),
      CONSTRUCT_SIGNAL_INNER(w_force, dynamicgraph::Vector, m_forceSIN),
      CONSTRUCT_SIGNAL_INNER(w_dq, dynamicgraph::Vector,
                             INPUT_SIGNALS << m_w_forceSINNER),
      CONSTRUCT_SIGNAL_OUT(dq, dynamicgraph::Vector, m_w_dqSINNER),
      m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << INNER_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid1(*this, &AdmittanceControlOpPoint::init,
                                      docCommandVoid1("Initialize the entity.",
                                                      "time step")));
  addCommand("resetDq",
             makeCommandVoid0(*this, &AdmittanceControlOpPoint::resetDq,
                              docCommandVoid0("resetDq")));
}

void AdmittanceControlOpPoint::init(const double &dt) {
  if (!m_dqSaturationSIN.isPlugged())
    return SEND_MSG("Init failed: signal dqSaturation is not plugged",
                    MSG_TYPE_ERROR);
  if (!m_KpSIN.isPlugged())
    return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
  if (!m_KdSIN.isPlugged())
    return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
  if (!m_forceSIN.isPlugged())
    return SEND_MSG("Init failed: signal force is not plugged", MSG_TYPE_ERROR);
  if (!m_w_forceDesSIN.isPlugged())
    return SEND_MSG("Init failed: signal w_forceDes is not plugged",
                    MSG_TYPE_ERROR);
  if (!m_opPoseSIN.isPlugged())
    return SEND_MSG("Init failed: signal opPose is not plugged",
                    MSG_TYPE_ERROR);
  if (!m_sensorPoseSIN.isPlugged())
    return SEND_MSG("Init failed: signal sensorPose is not plugged",
                    MSG_TYPE_ERROR);

  m_n = 6;
  m_dt = dt;
  m_w_dq.setZero(m_n);
  m_initSucceeded = true;
}

void AdmittanceControlOpPoint::resetDq() {
  m_w_dq.setZero(m_n);
  return;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */
DEFINE_SIGNAL_INNER_FUNCTION(w_force, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot compute signal w_force before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLOPPOINT_WFORCE_COMPUTATION);

  const Vector &force = m_forceSIN(iter);
  const MatrixHomogeneous &sensorPose = m_sensorPoseSIN(iter);
  assert(force.size() == m_n && "Unexpected size of signal force");
  pinocchio::SE3 sensorPlacement(
      sensorPose.matrix());  // homogeneous matrix to SE3
  s = sensorPlacement.act(pinocchio::Force(force)).toVector();

  getProfiler().stop(PROFILE_ADMITTANCECONTROLOPPOINT_WFORCE_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_INNER_FUNCTION(w_dq, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot compute signal w_dq before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLOPPOINT_WDQ_COMPUTATION);

  const Vector &w_forceDes = m_w_forceDesSIN(iter);
  const Vector &w_force = m_w_forceSINNER(iter);
  const Vector &Kp = m_KpSIN(iter);
  const Vector &Kd = m_KdSIN(iter);
  const Vector &dqSaturation = m_dqSaturationSIN(iter);
  assert(w_force.size() == m_n && "Unexpected size of signal force");
  assert(w_forceDes.size() == m_n && "Unexpected size of signal w_forceDes");
  assert(Kp.size() == m_n && "Unexpected size of signal Kp");
  assert(Kd.size() == m_n && "Unexpected size of signal Kd");
  assert(dqSaturation.size() == m_n &&
         "Unexpected size of signal dqSaturation");

  m_w_dq = m_w_dq + m_dt * (Kp.cwiseProduct(w_forceDes - w_force)) -
           Kd.cwiseProduct(m_w_dq);

  for (int i = 0; i < m_n; i++) {
    if (m_w_dq[i] > dqSaturation[i]) m_w_dq[i] = dqSaturation[i];
    if (m_w_dq[i] < -dqSaturation[i]) m_w_dq[i] = -dqSaturation[i];
  }

  s = m_w_dq;

  getProfiler().stop(PROFILE_ADMITTANCECONTROLOPPOINT_WDQ_COMPUTATION);

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dq, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute signal dq before initialization!");
    return s;
  }
  if (s.size() != 6) s.resize(6);

  getProfiler().start(PROFILE_ADMITTANCECONTROLOPPOINT_DQ_COMPUTATION);

  const Vector &w_dq = m_w_dqSINNER(iter);
  const MatrixHomogeneous &opPose = m_opPoseSIN(iter);
  assert(w_dq.size() == m_n && "Unexpected size of signal w_dq");
  pinocchio::SE3 opPointPlacement(
      opPose.matrix());  // homogeneous matrix to SE3
  s = opPointPlacement.actInv(pinocchio::Motion(w_dq)).toVector();

  getProfiler().stop(PROFILE_ADMITTANCECONTROLOPPOINT_DQ_COMPUTATION);

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */
void AdmittanceControlOpPoint::display(std::ostream &os) const {
  os << "AdmittanceControlOpPoint " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
}  // namespace core
}  // namespace sot
}  // namespace dynamicgraph

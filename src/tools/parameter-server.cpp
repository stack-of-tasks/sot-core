/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <pinocchio/fwd.hpp>
// keep pinocchio before boost

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <boost/property_tree/ptree.hpp>
#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <sot/core/parameter-server.hh>

namespace dynamicgraph {
namespace sot {
namespace dynamicgraph = ::dynamicgraph;
using namespace dynamicgraph;
using namespace dynamicgraph::command;
using namespace std;

// Size to be aligned "-------------------------------------------------------"
#define PROFILE_PWM_DESIRED_COMPUTATION \
  "Control manager                                        "
#define PROFILE_DYNAMIC_GRAPH_PERIOD \
  "Control period                                         "

#define INPUT_SIGNALS
#define OUTPUT_SIGNALS

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef ParameterServer EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ParameterServer, "ParameterServer");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
ParameterServer::ParameterServer(const std::string &name)
    : Entity(name),
      m_robot_util(RefVoidRobotUtil()),
      m_initSucceeded(false),
      m_emergency_stop_triggered(false),
      m_is_first_iter(true),
      m_iter(0),
      m_sleep_time(0.0) {
  //~ Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init",
             makeCommandVoid3(*this, &ParameterServer::init,
                              docCommandVoid3("Initialize the entity.",
                                              "Time period in seconds (double)",
                                              "URDF file path (string)",
                                              "Robot reference (string)")));
  addCommand(
      "init_simple",
      makeCommandVoid1(*this, &ParameterServer::init_simple,
                       docCommandVoid1("Initialize the entity.",
                                       "Time period in seconds (double)")));

  addCommand("setNameToId",
             makeCommandVoid2(
                 *this, &ParameterServer::setNameToId,
                 docCommandVoid2("Set map for a name to an Id",
                                 "(string) joint name", "(double) joint id")));

  addCommand(
      "setForceNameToForceId",
      makeCommandVoid2(
          *this, &ParameterServer::setForceNameToForceId,
          docCommandVoid2(
              "Set map from a force sensor name to a force sensor Id",
              "(string) force sensor name", "(double) force sensor id")));

  addCommand("setJointLimitsFromId",
             makeCommandVoid3(
                 *this, &ParameterServer::setJointLimitsFromId,
                 docCommandVoid3("Set the joint limits for a given joint ID",
                                 "(double) joint id", "(double) lower limit",
                                 "(double) uppper limit")));

  addCommand(
      "setForceLimitsFromId",
      makeCommandVoid3(
          *this, &ParameterServer::setForceLimitsFromId,
          docCommandVoid3("Set the force limits for a given force sensor ID",
                          "(double) force sensor id", "(double) lower limit",
                          "(double) uppper limit")));

  addCommand(
      "setJointsUrdfToSot",
      makeCommandVoid1(*this, &ParameterServer::setJoints,
                       docCommandVoid1("Map Joints From URDF to SoT.",
                                       "Vector of integer for mapping")));

  addCommand(
      "setRightFootSoleXYZ",
      makeCommandVoid1(*this, &ParameterServer::setRightFootSoleXYZ,
                       docCommandVoid1("Set the right foot sole 3d position.",
                                       "Vector of 3 doubles")));
  addCommand(
      "setRightFootForceSensorXYZ",
      makeCommandVoid1(*this, &ParameterServer::setRightFootForceSensorXYZ,
                       docCommandVoid1("Set the right foot sensor 3d position.",
                                       "Vector of 3 doubles")));

  addCommand("setFootFrameName",
             makeCommandVoid2(
                 *this, &ParameterServer::setFootFrameName,
                 docCommandVoid2("Set the Frame Name for the Foot Name.",
                                 "(string) Foot name", "(string) Frame name")));
  addCommand("setHandFrameName",
             makeCommandVoid2(
                 *this, &ParameterServer::setHandFrameName,
                 docCommandVoid2("Set the Frame Name for the Hand Name.",
                                 "(string) Hand name", "(string) Frame name")));
  addCommand("setImuJointName",
             makeCommandVoid1(
                 *this, &ParameterServer::setImuJointName,
                 docCommandVoid1("Set the Joint Name wich IMU is attached to.",
                                 "(string) Joint name")));
  addCommand("displayRobotUtil",
             makeCommandVoid0(
                 *this, &ParameterServer::displayRobotUtil,
                 docCommandVoid0("Display the current robot util data set.")));

  addCommand(
      "setParameterBool",
      makeCommandVoid2(
          *this, &ParameterServer::setParameter<bool>,
          docCommandVoid2("Set a parameter named ParameterName to value "
                          "ParameterValue (string format).",
                          "(string) ParameterName", "(bool) ParameterValue")));
  addCommand(
      "setParameterInt",
      makeCommandVoid2(
          *this, &ParameterServer::setParameter<int>,
          docCommandVoid2("Set a parameter named ParameterName to value "
                          "ParameterValue (string format).",
                          "(string) ParameterName", "(int) ParameterValue")));
  addCommand("setParameterDbl",
             makeCommandVoid2(
                 *this, &ParameterServer::setParameter<double>,
                 docCommandVoid2("Set a parameter named ParameterName to value "
                                 "ParameterValue (string format).",
                                 "(string) ParameterName",
                                 "(double) ParameterValue")));

  addCommand("setParameter",
             makeCommandVoid2(
                 *this, &ParameterServer::setParameter<std::string>,
                 docCommandVoid2("Set a parameter named ParameterName to value "
                                 "ParameterValue (string format).",
                                 "(string) ParameterName",
                                 "(string) ParameterValue")));

  addCommand(
      "getParameter",
      makeCommandReturnType1(*this, &ParameterServer::getParameter<std::string>,
                             docCommandReturnType1<std::string>(
                                 "Return the parameter value for parameter"
                                 " named ParameterName.",
                                 "(string) ParameterName")));

  addCommand(
      "getParameterInt",
      makeCommandReturnType1(
          *this, &ParameterServer::getParameter<int>,
          docCommandReturnType1<int>("Return the parameter value for parameter"
                                     " named ParameterName.",
                                     "(int) ParameterName")));

  addCommand(
      "getParameterDbl",
      makeCommandReturnType1(*this, &ParameterServer::getParameter<double>,
                             docCommandReturnType1<double>(
                                 "Return the parameter value for parameter"
                                 " named ParameterName.",
                                 "(double) ParameterName")));

  addCommand(
      "getParameterBool",
      makeCommandReturnType1(
          *this, &ParameterServer::getParameter<bool>,
          docCommandReturnType1<bool>("Return the parameter value for parameter"
                                      " named ParameterName.",
                                      "(string) ParameterName")));
}

void ParameterServer::init_simple(const double &dt) {
  if (dt <= 0.0) return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);

  m_dt = dt;

  m_emergency_stop_triggered = false;
  m_initSucceeded = true;

  std::string localName("robot");
  std::shared_ptr<std::vector<std::string> > listOfRobots =
      sot::getListOfRobots();

  if (listOfRobots->size() == 1)
    localName = (*listOfRobots)[0];
  else {
    std::ostringstream oss;
    oss << "No robot registered in the parameter server list";
    oss << " listOfRobots->size: " << listOfRobots->size();
    throw ExceptionTools(ExceptionTools::ErrorCodeEnum::PARAMETER_SERVER,
                         oss.str());
  }

  if (!isNameInRobotUtil(localName)) {
    m_robot_util = createRobotUtil(localName);
  } else {
    m_robot_util = getRobotUtil(localName);
  }

  addCommand(
      "getJointsUrdfToSot",
      makeDirectGetter(*this, &m_robot_util->m_dgv_urdf_to_sot,
                       docDirectSetter("Display map Joints From URDF to SoT.",
                                       "Vector of integer for mapping")));
}

void ParameterServer::init(const double &dt, const std::string &urdfFile,
                           const std::string &robotRef) {
  if (dt <= 0.0) return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
  m_dt = dt;
  m_emergency_stop_triggered = false;
  m_initSucceeded = true;

  std::string localName(robotRef);
  if (!isNameInRobotUtil(localName)) {
    m_robot_util = createRobotUtil(localName);
  } else {
    m_robot_util = getRobotUtil(localName);
  }

  m_robot_util->m_urdf_filename = urdfFile;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

/* --- COMMANDS ---------------------------------------------------------- */

void ParameterServer::setNameToId(const std::string &jointName,
                                  const double &jointId) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set joint name from joint id  before initialization!");
    return;
  }
  m_robot_util->set_name_to_id(jointName, static_cast<Index>(jointId));
}

void ParameterServer::setJointLimitsFromId(const double &jointId,
                                           const double &lq, const double &uq) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set joints limits from joint id  before initialization!");
    return;
  }

  m_robot_util->set_joint_limits_for_id((Index)jointId, lq, uq);
}

void ParameterServer::setForceLimitsFromId(const double &jointId,
                                           const dynamicgraph::Vector &lq,
                                           const dynamicgraph::Vector &uq) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set force limits from force id  before initialization!");
    return;
  }

  m_robot_util->m_force_util.set_force_id_to_limits((Index)jointId, lq, uq);
}

void ParameterServer::setForceNameToForceId(const std::string &forceName,
                                            const double &forceId) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set force sensor name from force sensor id "
        " before initialization!");
    return;
  }

  m_robot_util->m_force_util.set_name_to_force_id(forceName,
                                                  static_cast<Index>(forceId));
}

void ParameterServer::setJoints(const dynamicgraph::Vector &urdf_to_sot) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set mapping to sot before initialization!");
    return;
  }
  m_robot_util->set_urdf_to_sot(urdf_to_sot);
}

void ParameterServer::setRightFootSoleXYZ(const dynamicgraph::Vector &xyz) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set right foot sole XYZ before initialization!");
    return;
  }

  m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ = xyz;
}

void ParameterServer::setRightFootForceSensorXYZ(
    const dynamicgraph::Vector &xyz) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot set right foot force sensor XYZ before initialization!");
    return;
  }

  m_robot_util->m_foot_util.m_Right_Foot_Force_Sensor_XYZ = xyz;
}

void ParameterServer::setFootFrameName(const std::string &FootName,
                                       const std::string &FrameName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set foot frame name!");
    return;
  }
  if (FootName == "Left")
    m_robot_util->m_foot_util.m_Left_Foot_Frame_Name = FrameName;
  else if (FootName == "Right")
    m_robot_util->m_foot_util.m_Right_Foot_Frame_Name = FrameName;
  else
    SEND_WARNING_STREAM_MSG("Did not understand the foot name !" + FootName);
}

void ParameterServer::setHandFrameName(const std::string &HandName,
                                       const std::string &FrameName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set hand frame name!");
    return;
  }
  if (HandName == "Left")
    m_robot_util->m_hand_util.m_Left_Hand_Frame_Name = FrameName;
  else if (HandName == "Right")
    m_robot_util->m_hand_util.m_Right_Hand_Frame_Name = FrameName;
  else
    SEND_WARNING_STREAM_MSG(
        "Available hand names are 'Left' and 'Right', not '" + HandName +
        "' !");
}

void ParameterServer::setImuJointName(const std::string &JointName) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot set IMU joint name!");
    return;
  }
  m_robot_util->m_imu_joint_name = JointName;
}

void ParameterServer::displayRobotUtil() { m_robot_util->display(std::cout); }

/* --- PROTECTED MEMBER METHODS
 * ---------------------------------------------------------- */

bool ParameterServer::convertJointNameToJointId(const std::string &name,
                                                unsigned int &id) {
  // Check if the joint name exists
  sot::Index jid = m_robot_util->get_id_from_name(name);
  if (jid < 0) {
    SEND_MSG("The specified joint name does not exist: " + name,
             MSG_TYPE_ERROR);
    std::stringstream ss;
    for (long unsigned int it = 0; it < m_robot_util->m_nbJoints; it++)
      ss << m_robot_util->get_name_from_id(it) << ", ";
    SEND_MSG("Possible joint names are: " + ss.str(), MSG_TYPE_INFO);
    return false;
  }
  id = (unsigned int)jid;
  return true;
}

bool ParameterServer::isJointInRange(unsigned int id, double q) {
  const JointLimits &JL = m_robot_util->get_joint_limits_from_id((Index)id);

  double jl = JL.lower;
  if (q < jl) {
    SEND_MSG("Desired joint angle " + toString(q) +
                 " is smaller than lower limit: " + toString(jl),
             MSG_TYPE_ERROR);
    return false;
  }
  double ju = JL.upper;
  if (q > ju) {
    SEND_MSG("Desired joint angle " + toString(q) +
                 " is larger than upper limit: " + toString(ju),
             MSG_TYPE_ERROR);
    return false;
  }
  return true;
}

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void ParameterServer::display(std::ostream &os) const {
  os << "ParameterServer " << getName();
}
}  // namespace sot
}  // namespace dynamicgraph

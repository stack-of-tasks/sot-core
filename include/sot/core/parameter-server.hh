/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_parameter_server_H__
#define __sot_torque_control_parameter_server_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(__sot_torque_parameter_server_H__)
#define SOTParameterServer_EXPORT __declspec(dllexport)
#else
#define SOTParameterServer_EXPORT __declspec(dllimport)
#endif
#else
#define SOTParameterServer_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>

#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/// Number of time step to transition from one ctrl mode to another
#define CTRL_MODE_TRANSITION_TIME_STEP 1000.0

class SOTParameterServer_EXPORT ParameterServer
    : public ::dynamicgraph::Entity {
  typedef ParameterServer EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  ParameterServer(const std::string &name);

  ~ParameterServer(){};

  /// Initialize
  /// @param dt: control interval
  /// @param urdfFile: path to the URDF model of the robot
  void init(const double &dt, const std::string &urdfFile,
            const std::string &robotRef);

  /// Initialize
  /// @param dt: control interval provided by the device.
  /// The urdf model is found by reading /robot_description
  /// The robot name is found using the name inside robot_description
  void init_simple(const double &dt);
  /* --- SIGNALS --- */

  /* --- COMMANDS --- */

  /// Commands related to joint name and joint id
  void setNameToId(const std::string &jointName, const double &jointId);
  void setJointLimitsFromId(const double &jointId, const double &lq,
                            const double &uq);

  /// Command related to ForceUtil
  void setForceLimitsFromId(const double &jointId,
                            const dynamicgraph::Vector &lq,
                            const dynamicgraph::Vector &uq);
  void setForceNameToForceId(const std::string &forceName,
                             const double &forceId);

  /// \name  Commands related to FootUtil
  /// @{
  void setRightFootSoleXYZ(const dynamicgraph::Vector &);
  void setRightFootForceSensorXYZ(const dynamicgraph::Vector &);
  void setFootFrameName(const std::string &, const std::string &);
  void setHandFrameName(const std::string &, const std::string &);
  void setImuJointName(const std::string &);
  void displayRobotUtil();
  /// @}
  /// \name Commands related to the model
  /// @{
  template <typename Type>
  void setParameter(const std::string &ParameterName,
                    const Type &ParameterValue) {
    if (!m_initSucceeded) {
      DYNAMIC_GRAPH_ENTITY_WARNING(*this)
          << "Cannot set parameter " << ParameterName << " to "
          << ParameterValue << " before initialization!\n";
      return;
    }

    m_robot_util->set_parameter<Type>(ParameterName, ParameterValue);
  }

  template <typename Type>
  Type getParameter(const std::string &ParameterName) {
    if (!m_initSucceeded) {
      DYNAMIC_GRAPH_ENTITY_WARNING(*this)
          << "Cannot get parameter " << ParameterName
          << " before initialization!\n";
      throw std::runtime_error("Cannot get parameter before initialization");
    }
    return m_robot_util->get_parameter<Type>(ParameterName);
  }

  /// @}
  /// Set the mapping between urdf and sot.
  void setJoints(const dynamicgraph::Vector &);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

 protected:
  RobotUtilShrPtr m_robot_util;
  bool
      m_initSucceeded;  /// true if the entity has been successfully initialized
  double m_dt;          /// control loop time period
  bool m_emergency_stop_triggered;  /// true if an emergency condition as been
                                    /// triggered either by an other entity, or
                                    /// by control limit violation
  bool m_is_first_iter;  /// true at the first iteration, false otherwise
  int m_iter;
  double m_sleep_time;  /// time to sleep at every iteration (to slow down
                        /// simulation)

  bool convertJointNameToJointId(const std::string &name, unsigned int &id);
  bool isJointInRange(unsigned int id, double q);
  void updateJointCtrlModesOutputSignal();

};  // class ParameterServer

}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_control_manager_H__

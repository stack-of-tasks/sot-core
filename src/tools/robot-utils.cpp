/*
 * Copyright 2017, 2019
 * LAAS-CNRS
 * A. Del Prete, T. Flayols, O. Stasse, F. Bailly
 *
 */

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic-graph/factory.h>
#include <iostream>
#include <sot/core/debug.hh>
#include <sot/core/robot-utils.hh>

namespace dynamicgraph {
namespace sot {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

RobotUtil VoidRobotUtil;
ForceLimits VoidForceLimits;
JointLimits VoidJointLimits;
Index VoidIndex(-1);

RobotUtilShrPtr RefVoidRobotUtil() {
  return boost::make_shared<RobotUtil>(VoidRobotUtil);
}

void ForceLimits::display(std::ostream &os) const {
  os << "Lower limits:" << std::endl;
  os << lower << std::endl;
  os << "Upper Limits:" << std::endl;
  os << upper << std::endl;
}

/******************** FootUtil ***************************/

void FootUtil::display(std::ostream &os) const {
  os << "Right Foot Sole XYZ " << std::endl;
  os << m_Right_Foot_Sole_XYZ << std::endl;
  os << "Left Foot Frame Name:" << m_Left_Foot_Frame_Name << std::endl;
  os << "Right Foot Frame Name:" << m_Right_Foot_Frame_Name << std::endl;
}

/******************** HandUtil ***************************/

void HandUtil::display(std::ostream &os) const {
  os << "Left Hand Frame Name:" << m_Left_Hand_Frame_Name << std::endl;
  os << "Right Hand Frame Name:" << m_Right_Hand_Frame_Name << std::endl;
}

/******************** ForceUtil ***************************/

void ForceUtil::set_name_to_force_id(const std::string &name,
                                     const Index &force_id) {
  m_name_to_force_id[name] = (Index)force_id;
  create_force_id_to_name_map();
  if (name == "rf")
    set_force_id_right_foot(m_name_to_force_id[name]);
  else if (name == "lf")
    set_force_id_left_foot(m_name_to_force_id[name]);
  else if (name == "lh")
    set_force_id_left_hand(m_name_to_force_id[name]);
  else if (name == "rh")
    set_force_id_right_hand(m_name_to_force_id[name]);
}

void ForceUtil::set_force_id_to_limits(const Index &force_id,
                                       const dg::Vector &lf,
                                       const dg::Vector &uf) {
  m_force_id_to_limits[(Index)force_id].lower = lf;
  m_force_id_to_limits[(Index)force_id].upper = uf;
}

Index ForceUtil::get_id_from_name(const std::string &name) {
  std::map<std::string, Index>::const_iterator it;
  it = m_name_to_force_id.find(name);
  if (it != m_name_to_force_id.end())
    return it->second;
  return -1;
}

std::string force_default_rtn("Force name not found");
std::string joint_default_rtn("Joint name not found");

const std::string &ForceUtil::get_name_from_id(Index idx) {
  std::map<Index, std::string>::const_iterator it;
  it = m_force_id_to_name.find(idx);
  if (it != m_force_id_to_name.end())
    return it->second;
  return force_default_rtn;
}

std::string ForceUtil::cp_get_name_from_id(Index idx) {
  const std::string &default_rtn = get_name_from_id(idx);
  return default_rtn;
}
void ForceUtil::create_force_id_to_name_map() {
  std::map<std::string, Index>::const_iterator it;
  for (it = m_name_to_force_id.begin(); it != m_name_to_force_id.end(); it++)
    m_force_id_to_name[it->second] = it->first;
}

const ForceLimits &ForceUtil::get_limits_from_id(Index force_id) {
  std::map<Index, ForceLimits>::const_iterator iter =
      m_force_id_to_limits.find(force_id);
  if (iter == m_force_id_to_limits.end())
    return VoidForceLimits; // Returns void instance
  return iter->second;
}

ForceLimits ForceUtil::cp_get_limits_from_id(Index force_id) {
  std::map<Index, ForceLimits>::const_iterator iter =
      m_force_id_to_limits.find(force_id);
  if (iter == m_force_id_to_limits.end())
    return VoidForceLimits; // Returns void instance
  return iter->second;
}

void ForceUtil::display(std::ostream &os) const {
  os << "Force Id to limits " << std::endl;
  for (std::map<Index, ForceLimits>::const_iterator it =
           m_force_id_to_limits.begin();
       it != m_force_id_to_limits.end(); ++it) {
    it->second.display(os);
  }

  os << "Name to force id:" << std::endl;
  for (std::map<std::string, Index>::const_iterator it =
           m_name_to_force_id.begin();
       it != m_name_to_force_id.end(); ++it) {
    os << "(" << it->first << "," << it->second << ") ";
  }
  os << std::endl;

  os << "Force id to Name:" << std::endl;
  for (std::map<Index, std::string>::const_iterator it =
           m_force_id_to_name.begin();
       it != m_force_id_to_name.end(); ++it) {
    os << "(" << it->first << "," << it->second << ") ";
  }
  os << std::endl;

  os << "Index for force sensors:" << std::endl;
  os << "Left Hand (" << m_Force_Id_Left_Hand << ") ,"
     << "Right Hand (" << m_Force_Id_Right_Hand << ") ,"
     << "Left Foot (" << m_Force_Id_Left_Foot << ") ,"
     << "Right Foot (" << m_Force_Id_Right_Foot << ") " << std::endl;
}

/**************** FromURDFToSot *************************/
RobotUtil::RobotUtil() {}

void RobotUtil::set_joint_limits_for_id(const Index &idx, const double &lq,
                                        const double &uq) {
  m_limits_map[(Index)idx] = JointLimits(lq, uq);
}

const JointLimits &RobotUtil::get_joint_limits_from_id(Index id) {
  std::map<Index, JointLimits>::const_iterator iter = m_limits_map.find(id);
  if (iter == m_limits_map.end())
    return VoidJointLimits;
  return iter->second;
}
JointLimits RobotUtil::cp_get_joint_limits_from_id(Index id) {
  const JointLimits &rtn = get_joint_limits_from_id(id);
  return rtn;
}

void RobotUtil::set_name_to_id(const std::string &jointName,
                               const Index &jointId) {
  m_name_to_id[jointName] = (Index)jointId;
  create_id_to_name_map();
}

void RobotUtil::create_id_to_name_map() {
  std::map<std::string, Index>::const_iterator it;
  for (it = m_name_to_id.begin(); it != m_name_to_id.end(); it++)
    m_id_to_name[it->second] = it->first;
}

const Index &RobotUtil::get_id_from_name(const std::string &name) {
  std::map<std::string, Index>::const_iterator it = m_name_to_id.find(name);
  if (it == m_name_to_id.end())
    return VoidIndex;
  return it->second;
}

const std::string &RobotUtil::get_name_from_id(Index id) {
  std::map<Index, std::string>::const_iterator iter = m_id_to_name.find(id);
  if (iter == m_id_to_name.end())
    return joint_default_rtn;
  return iter->second;
}

void RobotUtil::set_urdf_to_sot(const std::vector<Index> &urdf_to_sot) {
  m_nbJoints = urdf_to_sot.size();
  m_urdf_to_sot.resize(urdf_to_sot.size());
  m_dgv_urdf_to_sot.resize(urdf_to_sot.size());
  for (std::size_t idx = 0; idx < urdf_to_sot.size(); idx++) {
    m_urdf_to_sot[(Index)idx] = urdf_to_sot[(Index)idx];
    m_dgv_urdf_to_sot[(Index)idx] =
        static_cast<double>(urdf_to_sot[(Index)idx]);
  }
}

void RobotUtil::set_urdf_to_sot(const dg::Vector &urdf_to_sot) {
  m_nbJoints = urdf_to_sot.size();
  m_urdf_to_sot.resize(urdf_to_sot.size());
  for (unsigned int idx = 0; idx < urdf_to_sot.size(); idx++) {
    m_urdf_to_sot[idx] = (unsigned int)urdf_to_sot[idx];
  }
  m_dgv_urdf_to_sot = urdf_to_sot;
}

bool RobotUtil::joints_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot) {
  if (m_nbJoints == 0) {
    SEND_MSG("set_urdf_to_sot should be called", MSG_TYPE_ERROR);
    return false;
  }
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints));
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints));

  for (unsigned int idx = 0; idx < m_nbJoints; idx++)
    q_sot[m_urdf_to_sot[idx]] = q_urdf[idx];
  return true;
}

bool RobotUtil::joints_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf) {
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints));
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints));

  if (m_nbJoints == 0) {
    SEND_MSG("set_urdf_to_sot should be called", MSG_TYPE_ERROR);
    return false;
  }

  for (unsigned int idx = 0; idx < m_nbJoints; idx++)
    q_urdf[idx] = q_sot[m_urdf_to_sot[idx]];
  return true;
}

bool RobotUtil::velocity_urdf_to_sot(ConstRefVector q_urdf,
                                     ConstRefVector v_urdf, RefVector v_sot) {
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 7));
  assert(v_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));
  assert(v_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));

  if (m_nbJoints == 0) {
    SEND_MSG("velocity_urdf_to_sot should be called", MSG_TYPE_ERROR);
    return false;
  }
  const Eigen::Quaterniond q(q_urdf(6), q_urdf(3), q_urdf(4), q_urdf(5));
  Eigen::Matrix3d oRb = q.toRotationMatrix();
  v_sot.head<3>() = oRb * v_urdf.head<3>();
  v_sot.segment<3>(3) = oRb * v_urdf.segment<3>(3);
  //        v_sot.head<6>() = v_urdf.head<6>();
  joints_urdf_to_sot(v_urdf.tail(m_nbJoints), v_sot.tail(m_nbJoints));
  return true;
}

bool RobotUtil::velocity_sot_to_urdf(ConstRefVector q_urdf,
                                     ConstRefVector v_sot, RefVector v_urdf) {
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 7));
  assert(v_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));
  assert(v_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));

  if (m_nbJoints == 0) {
    SEND_MSG("velocity_sot_to_urdf should be called", MSG_TYPE_ERROR);
    return false;
  }
  // compute rotation from world to base frame
  const Eigen::Quaterniond q(q_urdf(6), q_urdf(3), q_urdf(4), q_urdf(5));
  Eigen::Matrix3d oRb = q.toRotationMatrix();
  v_urdf.head<3>() = oRb.transpose() * v_sot.head<3>();
  v_urdf.segment<3>(3) = oRb.transpose() * v_sot.segment<3>(3);
  //        v_urdf.head<6>() = v_sot.head<6>();
  joints_sot_to_urdf(v_sot.tail(m_nbJoints), v_urdf.tail(m_nbJoints));
  return true;
}

bool RobotUtil::base_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);

  // ********* Quat to RPY *********
  const double W = q_urdf[6];
  const double X = q_urdf[3];
  const double Y = q_urdf[4];
  const double Z = q_urdf[5];
  const Eigen::Matrix3d R = Eigen::Quaterniond(W, X, Y, Z).toRotationMatrix();
  return base_se3_to_sot(q_urdf.head<3>(), R, q_sot);
}

bool RobotUtil::base_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);
  // *********  RPY to Quat *********
  const double r = q_sot[3];
  const double p = q_sot[4];
  const double y = q_sot[5];
  const Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
  const Eigen::Quaternion<double> quat = yawAngle * pitchAngle * rollAngle;

  q_urdf[0] = q_sot[0]; // BASE
  q_urdf[1] = q_sot[1];
  q_urdf[2] = q_sot[2];
  q_urdf[3] = quat.x();
  q_urdf[4] = quat.y();
  q_urdf[5] = quat.z();
  q_urdf[6] = quat.w();

  return true;
}

bool RobotUtil::config_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot) {
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 7));
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));

  base_urdf_to_sot(q_urdf.head<7>(), q_sot.head<6>());
  joints_urdf_to_sot(q_urdf.tail(m_nbJoints), q_sot.tail(m_nbJoints));

  return true;
}

bool RobotUtil::config_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf) {
  assert(q_urdf.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 7));
  assert(q_sot.size() == static_cast<Eigen::VectorXd::Index>(m_nbJoints + 6));
  base_sot_to_urdf(q_sot.head<6>(), q_urdf.head<7>());
  joints_sot_to_urdf(q_sot.tail(m_nbJoints), q_urdf.tail(m_nbJoints));
  return true;
}
void RobotUtil::display(std::ostream &os) const {
  m_force_util.display(os);
  m_foot_util.display(os);
  m_hand_util.display(os);
  os << "Nb of joints: " << m_nbJoints << std::endl;
  os << "Urdf file name: " << m_urdf_filename << std::endl;

  // Display m_urdf_to_sot
  os << "Map from urdf index to the Sot Index " << std::endl;
  for (unsigned int i = 0; i < m_urdf_to_sot.size(); i++)
    os << "(" << i << " : " << m_urdf_to_sot[i] << ") ";
  os << std::endl;

  os << "Joint name to joint id:" << std::endl;
  for (std::map<std::string, Index>::const_iterator it = m_name_to_id.begin();
       it != m_name_to_id.end(); ++it) {
    os << "(" << it->first << "," << it->second << ") ";
  }
  os << std::endl;

  os << "Joint id to joint Name:" << std::endl;
  for (std::map<Index, std::string>::const_iterator it = m_id_to_name.begin();
       it != m_id_to_name.end(); ++it) {
    os << "(" << it->first << "," << it->second << ") ";
  }
  os << std::endl;
}
void RobotUtil::sendMsg(const std::string &msg, MsgType t, const char *file,
                        int line) {
  logger_.sendMsg("[RobotUtil]" + msg, t, file, line);
}
bool base_se3_to_sot(ConstRefVector pos, ConstRefMatrix R, RefVector q_sot) {
  assert(q_sot.size() == 6);
  assert(pos.size() == 3);
  assert(R.rows() == 3);
  assert(R.cols() == 3);
  // ********* Quat to RPY *********
  double r, p, y, m;
  m = sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));
  p = atan2(-R(2, 0), m);
  if (fabs(fabs(p) - M_PI / 2) < 0.001) {
    r = 0.0;
    y = -atan2(R(0, 1), R(1, 1));
  } else {
    y = atan2(R(1, 0), R(0, 0));
    r = atan2(R(2, 1), R(2, 2));
  }
  // *********************************
  q_sot[0] = pos[0];
  q_sot[1] = pos[1];
  q_sot[2] = pos[2];
  q_sot[3] = r;
  q_sot[4] = p;
  q_sot[5] = y;
  return true;
}

bool base_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);
  // ********* Quat to RPY *********
  const double W = q_urdf[6];
  const double X = q_urdf[3];
  const double Y = q_urdf[4];
  const double Z = q_urdf[5];
  const Eigen::Matrix3d R = Eigen::Quaterniond(W, X, Y, Z).toRotationMatrix();
  return base_se3_to_sot(q_urdf.head<3>(), R, q_sot);
}

bool base_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);
  // *********  RPY to Quat *********
  const double r = q_sot[3];
  const double p = q_sot[4];
  const double y = q_sot[5];
  const Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
  const Eigen::Quaternion<double> quat = yawAngle * pitchAngle * rollAngle;

  q_urdf[0] = q_sot[0]; // BASE
  q_urdf[1] = q_sot[1];
  q_urdf[2] = q_sot[2];
  q_urdf[3] = quat.x();
  q_urdf[4] = quat.y();
  q_urdf[5] = quat.z();
  q_urdf[6] = quat.w();

  return true;
}

std::map<std::string, RobotUtilShrPtr> sgl_map_name_to_robot_util;

RobotUtilShrPtr getRobotUtil(std::string &robotName) {
  std::map<std::string, RobotUtilShrPtr>::iterator it =
      sgl_map_name_to_robot_util.find(robotName);
  if (it != sgl_map_name_to_robot_util.end())
    return it->second;
  return RefVoidRobotUtil();
}

bool isNameInRobotUtil(std::string &robotName) {
  std::map<std::string, RobotUtilShrPtr>::iterator it =
      sgl_map_name_to_robot_util.find(robotName);
  if (it != sgl_map_name_to_robot_util.end())
    return true;
  return false;
}
RobotUtilShrPtr createRobotUtil(std::string &robotName) {
  std::map<std::string, RobotUtilShrPtr>::iterator it =
      sgl_map_name_to_robot_util.find(robotName);
  if (it == sgl_map_name_to_robot_util.end()) {
    sgl_map_name_to_robot_util[robotName] =
        boost::shared_ptr<RobotUtil>(new RobotUtil);
    it = sgl_map_name_to_robot_util.find(robotName);
    return it->second;
  }
  std::cout << "Another robot is already in the map for " << robotName
            << std::endl;
  return RefVoidRobotUtil();
}
} // namespace sot
} // namespace dynamicgraph

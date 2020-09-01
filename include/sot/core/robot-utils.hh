/*
 * Copyright 2017, 2019
 * LAAS-CNRS
 * A. Del Prete, T. Flayols, O. Stasse, F. Bailly
 *
 */

#ifndef __sot_core_robot_utils_H__
#define __sot_core_robot_utils_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <map>

#include <pinocchio/fwd.hpp>

#include <boost/assign.hpp>
#include <boost/property_tree/ptree.hpp>

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/logger.h>
#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
namespace sot {

struct SOT_CORE_EXPORT JointLimits {
  double upper;
  double lower;

  JointLimits() : upper(0.0), lower(0.0) {}

  JointLimits(double l, double u) : upper(u), lower(l) {}
};

typedef Eigen::VectorXd::Index Index;

class SOT_CORE_EXPORT ExtractJointMimics {

public:
  /// Constructor
  ExtractJointMimics(std::string &robot_model);

  /// Get mimic joints.
  const std::vector<std::string> &get_mimic_joints();

private:
  void go_through(boost::property_tree::ptree &pt, int level, int stage);

  // Create empty property tree object
  boost::property_tree::ptree tree_;
  std::vector<std::string> mimic_joints_;
  std::string current_joint_name_;
  void go_through_full();
};

struct SOT_CORE_EXPORT ForceLimits {
  Eigen::VectorXd upper;
  Eigen::VectorXd lower;

  ForceLimits() : upper(Vector6d::Zero()), lower(Vector6d::Zero()) {}

  ForceLimits(const Eigen::VectorXd &l, const Eigen::VectorXd &u)
      : upper(u), lower(l) {}

  void display(std::ostream &os) const;
};

struct SOT_CORE_EXPORT ForceUtil {
  std::map<Index, ForceLimits> m_force_id_to_limits;
  std::map<std::string, Index> m_name_to_force_id;
  std::map<Index, std::string> m_force_id_to_name;

  Index m_Force_Id_Left_Hand, m_Force_Id_Right_Hand, m_Force_Id_Left_Foot,
      m_Force_Id_Right_Foot;

  void set_name_to_force_id(const std::string &name, const Index &force_id);

  void set_force_id_to_limits(const Index &force_id,
                              const dynamicgraph::Vector &lf,
                              const dynamicgraph::Vector &uf);

  void create_force_id_to_name_map();

  Index get_id_from_name(const std::string &name);

  const std::string &get_name_from_id(Index idx);
  std::string cp_get_name_from_id(Index idx);

  const ForceLimits &get_limits_from_id(Index force_id);
  ForceLimits cp_get_limits_from_id(Index force_id);

  Index get_force_id_left_hand() { return m_Force_Id_Left_Hand; }

  void set_force_id_left_hand(Index anId) { m_Force_Id_Left_Hand = anId; }

  Index get_force_id_right_hand() { return m_Force_Id_Right_Hand; }

  void set_force_id_right_hand(Index anId) { m_Force_Id_Right_Hand = anId; }

  Index get_force_id_left_foot() { return m_Force_Id_Left_Foot; }

  void set_force_id_left_foot(Index anId) { m_Force_Id_Left_Foot = anId; }

  Index get_force_id_right_foot() { return m_Force_Id_Right_Foot; }

  void set_force_id_right_foot(Index anId) { m_Force_Id_Right_Foot = anId; }

  void display(std::ostream &out) const;

}; // struct ForceUtil

struct SOT_CORE_EXPORT FootUtil {
  /// Position of the foot soles w.r.t. the frame of the foot
  dynamicgraph::Vector m_Right_Foot_Sole_XYZ;

  /// Position of the force/torque sensors w.r.t. the frame of the hosting link
  dynamicgraph::Vector m_Right_Foot_Force_Sensor_XYZ;

  std::string m_Left_Foot_Frame_Name;
  std::string m_Right_Foot_Frame_Name;
  void display(std::ostream &os) const;
};

struct SOT_CORE_EXPORT HandUtil {
  std::string m_Left_Hand_Frame_Name;
  std::string m_Right_Hand_Frame_Name;
  void display(std::ostream &os) const;
};

struct SOT_CORE_EXPORT RobotUtil {
public:
  RobotUtil();

  /// Forces data
  ForceUtil m_force_util;

  /// Foot information
  FootUtil m_foot_util;

  /// Hand information
  HandUtil m_hand_util;

  /// Map from the urdf index to the SoT index.
  std::vector<Index> m_urdf_to_sot;

  /// Nb of Dofs for the robot.
  std::size_t m_nbJoints;

  /// Map from the name to the id.
  std::map<std::string, Index> m_name_to_id;

  /// The map between id and name
  std::map<Index, std::string> m_id_to_name;

  /// The joint limits map.
  std::map<Index, JointLimits> m_limits_map;

  /// The name of the joint IMU is attached to
  std::string m_imu_joint_name;

  /// This method creates the map between id and name.
  /// It is called each time a new link between id and name is inserted
  /// (i.e. when set_name_to_id is called).
  void create_id_to_name_map();

  /// URDF file path
  std::string m_urdf_filename;

  dynamicgraph::Vector m_dgv_urdf_to_sot;

  /** Given a joint name it finds the associated joint id.
   * If the specified joint name is not found it returns -1;
   * @param name Name of the joint to find.
   * @return The id of the specified joint, -1 if not found. */
  const Index &get_id_from_name(const std::string &name);

  /** Given a joint id it finds the associated joint name.
   * If the specified joint is not found it returns "Joint name not found";
   * @param id Id of the joint to find.
   * @return The name of the specified joint, "Joint name not found" if not
   * found. */

  /// Get the joint name from its index
  const std::string &get_name_from_id(Index id);

  /// Set relation between the name and the SoT id
  void set_name_to_id(const std::string &jointName, const Index &jointId);

  /// Set the map between urdf index and sot index
  void set_urdf_to_sot(const std::vector<Index> &urdf_to_sot);
  void set_urdf_to_sot(const dynamicgraph::Vector &urdf_to_sot);

  /// Set the limits (lq,uq) for joint idx
  void set_joint_limits_for_id(const Index &idx, const double &lq,
                               const double &uq);

  bool joints_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot);

  bool joints_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf);

  bool velocity_urdf_to_sot(ConstRefVector q_urdf, ConstRefVector v_urdf,
                            RefVector v_sot);

  bool velocity_sot_to_urdf(ConstRefVector q_urdf, ConstRefVector v_sot,
                            RefVector v_urdf);

  bool config_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot);
  bool config_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf);

  bool base_urdf_to_sot(ConstRefVector q_urdf, RefVector q_sot);
  bool base_sot_to_urdf(ConstRefVector q_sot, RefVector q_urdf);

  /** Given a joint id it finds the associated joint limits.
   * If the specified joint is not found it returns JointLimits(0,0).
   * @param id Id of the joint to find.
   * @return The limits of the specified joint, JointLimits(0,0) if not found.
   */
  const JointLimits &get_joint_limits_from_id(Index id);
  JointLimits cp_get_joint_limits_from_id(Index id);

  /** \name Logger related methods */
  /** \{*/
  /// \brief Send messages \c msg with level \c t. Add string \c file and \c
  /// line to message.
  void sendMsg(const std::string &msg, MsgType t = MSG_TYPE_INFO,
               const std::string &lineId = "");

  /// \brief Specify the verbosity level of the logger.
  void setLoggerVerbosityLevel(LoggerVerbosity lv) { logger_.setVerbosity(lv); }

  /// \brief Get the logger's verbosity level.
  LoggerVerbosity getLoggerVerbosityLevel() { return logger_.getVerbosity(); };

  void display(std::ostream &os) const;

  /**{ \name Handling general parameters */
  /** \brief Set a parameter of type string.
      If parameter_name already exists the value is overwritten.
      If not it is inserted.
   */
  template <typename Type>
  void set_parameter(const std::string &parameter_name,
                     const Type &parameter_value) {
    try {
      typedef boost::property_tree::ptree::path_type path;
      path apath(parameter_name, '/');
      property_tree_.put<Type>(apath, parameter_value);
    } catch (const boost::property_tree::ptree_error &e) {
      std::ostringstream oss;
      oss << "Robot utils: parameter path is invalid " << '\n'
          << " for set_parameter(" << parameter_name << ")\n"
          << e.what() << std::endl;
      sendMsg(oss.str(), MSG_TYPE_ERROR);
      return;
    }
  }

  /** \brief Get a parameter of type string.
      If parameter_name already exists the value is overwritten.
      If not it is inserted.
      @param parameter_name: Name of the parameter
      Return false if the parameter is not found.
   */
  template <typename Type>
  Type get_parameter(const std::string &parameter_name) {
    try {
      boost::property_tree::ptree::path_type apath(parameter_name, '/');
      const Type &res = property_tree_.get<Type>(apath);

      return res;
    } catch (const boost::property_tree::ptree_error &e) {
      std::ostringstream oss;
      oss << "Robot utils: parameter path is invalid " << '\n'
          << " for get_parameter(" << parameter_name << ")\n"
          << e.what() << std::endl;
      sendMsg(oss.str(), MSG_TYPE_ERROR);
    }
  }
  /** @} */

  /** Access to property tree directly */
  boost::property_tree::ptree &get_property_tree();

protected:
  Logger logger_;

  /** \brief Map of the parameters: map of strings. */
  std::map<std::string, std::string> parameters_strings_;

  /** \brief Property tree */
  boost::property_tree::ptree property_tree_;
}; // struct RobotUtil

/// Accessors - This should be changed to RobotUtilPtrShared
typedef std::shared_ptr<RobotUtil> RobotUtilShrPtr;

RobotUtilShrPtr RefVoidRobotUtil();
RobotUtilShrPtr getRobotUtil(std::string &robotName);
bool isNameInRobotUtil(std::string &robotName);
RobotUtilShrPtr createRobotUtil(std::string &robotName);
std::shared_ptr<std::vector<std::string> > getListOfRobots();

bool base_se3_to_sot(ConstRefVector pos, ConstRefMatrix R, RefVector q_sot);

} // namespace sot
} // namespace dynamicgraph

#endif // sot_torque_control_common_h_

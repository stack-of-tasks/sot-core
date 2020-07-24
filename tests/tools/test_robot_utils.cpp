/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * François Bailly
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <dynamic-graph/factory.h>
#include <iostream>
#include <sot/core/debug.hh>
#include <sot/core/robot-utils.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

std::string localName("robot_test");
RobotUtilShrPtr robot_util;
int main(void) {
  robot_util = createRobotUtil(localName);
  /*Test set and get joint_limits_for_id */
  const double upper_lim(1);
  const double lower_lim(2);
  robot_util->set_joint_limits_for_id(1, lower_lim, upper_lim);
  if (robot_util->get_joint_limits_from_id(1).upper == upper_lim &&
      robot_util->get_joint_limits_from_id(1).lower == lower_lim) {
    std::cout << "joint_limits_for_id works !" << std::endl;
  } else {
    std::cout << "ERROR: joint_limits_for_id does not work !" << std::endl;
  }
  if (robot_util->cp_get_joint_limits_from_id(1).upper == upper_lim &&
      robot_util->cp_get_joint_limits_from_id(1).lower == lower_lim) {
    std::cout << "cp_get_joint_limits_for_id works !" << std::endl;
  } else {
    std::cout << "ERROR: cp_get_joint_limits_for_id does not work !"
              << std::endl;
  }

  /*Test set and get name_to_id */
  const std::string joint_name("test_joint");
  const Index joint_id(10);
  robot_util->set_name_to_id(joint_name, joint_id);
  if (robot_util->get_id_from_name(joint_name) == joint_id &&
      robot_util->get_name_from_id(joint_id) == joint_name) {
    std::cout << "name_to_id works !" << std::endl;
  } else {
    std::cout << "ERROR: name_to_id does not work !" << std::endl;
  }

  /*Test create_id_to_name_map */
  robot_util->create_id_to_name_map();

  /*Test set urdf_to_sot */

  dynamicgraph::Vector urdf_to_sot(3);
  urdf_to_sot << 0, 2, 1;
  robot_util->set_urdf_to_sot(urdf_to_sot);
  if (urdf_to_sot == robot_util->m_dgv_urdf_to_sot) {
    std::cout << "urdf_to_sot works !" << std::endl;
  } else {
    std::cout << "ERROR: urdf_to_sot does not work !" << std::endl;
  }
  /*Test joints_urdf_to_sot and joints_sot_to_urdf */
  dynamicgraph::Vector q_urdf(3);
  q_urdf << 10, 20, 30;
  dynamicgraph::Vector q_sot(3);
  dynamicgraph::Vector q_test_urdf(3);
  robot_util->joints_urdf_to_sot(q_urdf, q_sot);
  robot_util->joints_sot_to_urdf(q_sot, q_test_urdf);
  if (q_urdf == q_test_urdf) {
    std::cout << "joints_urdf_to_sot and joints_sot_to_urdf work !"
              << std::endl;
  } else {
    std::cout << "ERROR: joints_urdf_to_sot or joints_sot_to_urdf "
                 "do not work !"
              << std::endl;
  }

  /*Test velocity_sot_to_urdf and velocity_urdf_to_sot */
  dynamicgraph::Vector q2_urdf(10);
  dynamicgraph::Vector v_urdf(9);
  dynamicgraph::Vector v_sot(9);
  robot_util->velocity_urdf_to_sot(q2_urdf, v_urdf, v_sot);
  robot_util->velocity_sot_to_urdf(q2_urdf, v_sot, v_urdf);
  std::cout << "velocity_sot_to_urdf and velocity_urdf_to_sot work !"
            << std::endl;

  /*Test base_urdf_to_sot and base_sot_to_urdf */
  dynamicgraph::Vector base_q_urdf(7);
  dynamicgraph::Vector base_q_sot(6);
  robot_util->base_urdf_to_sot(base_q_urdf, base_q_sot);
  robot_util->base_sot_to_urdf(base_q_sot, base_q_urdf);
  std::cout << "base_urdf_to_sot and base_sot_to_urdf work !" << std::endl;

  /*Test config_urdf_to_sot and config_sot_to_urdf */
  dynamicgraph::Vector q2_sot(9);
  robot_util->config_urdf_to_sot(q2_urdf, q2_sot);
  robot_util->config_sot_to_urdf(q2_sot, q2_urdf);
  std::cout << "config_urdf_to_sot and config_sot_to_urdf work !" << std::endl;

  robot_util->display(std::cout);
  robot_util->sendMsg("test", MSG_TYPE_ERROR_STREAM);

  /*Test set_name_to_force_id of forceutil */
  const std::string rf("rf");
  const std::string lf("lf");
  const std::string lh("lh");
  const std::string rh("rh");

  robot_util->m_force_util.set_name_to_force_id(rf, 0);
  robot_util->m_force_util.set_name_to_force_id(lf, 1);
  robot_util->m_force_util.set_name_to_force_id(lh, 2);
  robot_util->m_force_util.set_name_to_force_id(rh, 3);

  dynamicgraph::Vector lf_lim(6);
  lf_lim << 1, 2, 3, 4, 5, 6;
  dynamicgraph::Vector uf_lim(6);
  uf_lim << 10, 20, 30, 40, 50, 60;
  robot_util->m_force_util.set_force_id_to_limits(1, lf_lim, uf_lim);
  if (robot_util->m_force_util.get_id_from_name(rf) == 0 &&
      robot_util->m_force_util.get_id_from_name(lf) == 1 &&
      robot_util->m_force_util.get_id_from_name(lh) == 2 &&
      robot_util->m_force_util.get_id_from_name(rh) == 3) {
    std::cout << "force_util set and get id_from_name work !" << std::endl;
  } else {
    std::cout << "ERROR: force_util set and get id_from_name do not work !"
              << std::endl;
  }
  if (robot_util->m_force_util.get_name_from_id(0) == rf &&
      robot_util->m_force_util.get_name_from_id(1) == lf &&
      robot_util->m_force_util.get_name_from_id(2) == lh &&
      robot_util->m_force_util.get_name_from_id(3) == rh) {
    std::cout << "force_util get name_from_id works !" << std::endl;
  } else {
    std::cout << "ERROR: force_util get name_from_id does not work !"
              << std::endl;
  }
  if (robot_util->m_force_util.get_limits_from_id(1).upper == uf_lim &&
      robot_util->m_force_util.get_limits_from_id(1).lower == lf_lim) {
    std::cout << "force_util set and get id to limits work !" << std::endl;
  } else {
    std::cout << "ERROR: force_util set and get id to "
                 "limits works do not work !"
              << std::endl;
  }
  robot_util->m_force_util.display(std::cout);
  robot_util->m_foot_util.display(std::cout);
  return 0;
}

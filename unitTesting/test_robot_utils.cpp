/*
 * Copyright 2019
 *
 * LAAS-CNRS
 *
 * François Bailly 
 * This file is part of sot-core.
 * See license file.
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>
#include <sot/core/robot-utils.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

std::string localName("robot_test");
RobotUtil * robot_util;
int main( void )
{
  robot_util = createRobotUtil(localName);
  /*Test set and get joint_limits_for_id */
  const double upper_lim(1);
  const double lower_lim(2);
  robot_util->set_joint_limits_for_id(1.,lower_lim,upper_lim);
  if (robot_util->get_joint_limits_from_id(1).upper == upper_lim && robot_util->get_joint_limits_from_id(1).lower == lower_lim)
  {
      std::cout << "joint_limits_for_id works !" << std::endl;  }
  else 
  {
      std::cout << "ERROR: joint_limits_for_id does not work !" << std::endl;
  }

  /*Test set and get name_to_id */
  const std::string joint_name("test_joint");
  const double joint_id(10);  
  robot_util->set_name_to_id(joint_name,joint_id);
  if (robot_util->get_id_from_name(joint_name) == joint_id && robot_util->get_name_from_id(joint_id) == joint_name)
  {
      std::cout << "name_to_id works !" << std::endl;  
  }
  else 
  {
      std::cout << "ERROR: name_to_id does not work !" << std::endl;
  }

  /*Test set urdf_to_sot */

  dg::Vector urdf_to_sot(3);
  urdf_to_sot << 1,3,2;
  robot_util->set_urdf_to_sot(urdf_to_sot);
  if (urdf_to_sot == robot_util->m_dgv_urdf_to_sot )
  {
      std::cout << "urdf_to_sot works !" << std::endl; 
  }
  else 
  {
      std::cout << "ERROR: urdf_to_sot does not work !" << std::endl;
  }
  return 0;
}
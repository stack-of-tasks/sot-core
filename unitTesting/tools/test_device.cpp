/*
 * Copyright 2018,
 * Olivier Stasse,
 *
 * CNRS
 * See LICENSE.txt
 *
 */

#include <iostream>
#include <sot/core/debug.hh>
#include <yaml-cpp/yaml.h>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot/core/device.hh>

#include <sstream>
#include <fstream>


#define BOOST_TEST_MODULE test-device

void CreateYAMLFILE() {
  YAML::Emitter yaml_out;
  YAML::Node aNode, yn_map_hw_sot_c, yn_map_sensors;
  yn_map_hw_sot_c = aNode["map_hardware_sot_control"];
  yn_map_sensors = aNode["sensors"];
  /*
  yn_map_hw_sot_c["waist"];
  yn_map_hw_sot_c["waist"]["hw"] = "POSITION";
  yn_map_hw_sot_c["waist"]["sot"] = "POSITION";
  yn_map_hw_sot_c["waist"]["controlPos"] = 0;
  yn_map_hw_sot_c["waist"]["sensors"] = "";
  */
  yn_map_hw_sot_c["LLEG_HIP_P"];
  yn_map_hw_sot_c["LLEG_HIP_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_P"]["controlPos"] = 6;
  yn_map_hw_sot_c["LLEG_HIP_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LLEG_HIP_R"];
  yn_map_hw_sot_c["LLEG_HIP_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_R"]["controlPos"] = 7;
  yn_map_hw_sot_c["LLEG_HIP_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LLEG_HIP_Y"];
  yn_map_hw_sot_c["LLEG_HIP_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_HIP_Y"]["controlPos"] = 8;
  yn_map_hw_sot_c["LLEG_HIP_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LLEG_KNEE"];
  yn_map_hw_sot_c["LLEG_KNEE"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_KNEE"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_KNEE"]["controlPos"] = 9;
  yn_map_hw_sot_c["LLEG_KNEE"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LLEG_ANKLE_P"];
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["controlPos"] = 10;
  yn_map_hw_sot_c["LLEG_ANKLE_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LLEG_ANKLE_R"];
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["controlPos"] = 11;
  yn_map_hw_sot_c["LLEG_ANKLE_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_HIP_P"];
  yn_map_hw_sot_c["RLEG_HIP_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_P"]["controlPos"] = 12;
  yn_map_hw_sot_c["RLEG_HIP_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_HIP_R"];
  yn_map_hw_sot_c["RLEG_HIP_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_R"]["controlPos"] = 13;
  yn_map_hw_sot_c["RLEG_HIP_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_HIP_Y"];
  yn_map_hw_sot_c["RLEG_HIP_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_HIP_Y"]["controlPos"] = 14;
  yn_map_hw_sot_c["RLEG_HIP_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_KNEE"];
  yn_map_hw_sot_c["RLEG_KNEE"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_KNEE"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_KNEE"]["controlPos"] = 15;
  yn_map_hw_sot_c["RLEG_KNEE"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_ANKLE_P"];
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["controlPos"] = 16;
  yn_map_hw_sot_c["RLEG_ANKLE_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RLEG_ANKLE_R"];
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["controlPos"] = 17;
  yn_map_hw_sot_c["RLEG_ANKLE_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["WAIST_P"];
  yn_map_hw_sot_c["WAIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["WAIST_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["WAIST_P"]["controlPos"] = 18;
  yn_map_hw_sot_c["WAIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["WAIST_R"];
  yn_map_hw_sot_c["WAIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["WAIST_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["WAIST_R"]["controlPos"] = 19;
  yn_map_hw_sot_c["WAIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["CHEST"];
  yn_map_hw_sot_c["CHEST"]["hw"] = "POSITION";
  yn_map_hw_sot_c["CHEST"]["sot"] = "POSITION";
  yn_map_hw_sot_c["CHEST"]["controlPos"] = 20;
  yn_map_hw_sot_c["CHEST"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_SHOULDER_P"];
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["controlPos"] = 21;
  yn_map_hw_sot_c["RARM_SHOULDER_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_SHOULDER_R"];
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["controlPos"] = 22;
  yn_map_hw_sot_c["RARM_SHOULDER_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_SHOULDER_Y"];
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["controlPos"] = 23;
  yn_map_hw_sot_c["RARM_SHOULDER_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_ELBOW"];
  yn_map_hw_sot_c["RARM_ELBOW"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_ELBOW"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_ELBOW"]["controlPos"] = 24;
  yn_map_hw_sot_c["RARM_ELBOW"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_WRIST_Y"];
  yn_map_hw_sot_c["RARM_WRIST_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_Y"]["controlPos"] = 25;
  yn_map_hw_sot_c["RARM_WRIST_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_WRIST_P"];
  yn_map_hw_sot_c["RARM_WRIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_P"]["controlPos"] = 26;
  yn_map_hw_sot_c["RARM_WRIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["RARM_WRIST_R"];
  yn_map_hw_sot_c["RARM_WRIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["RARM_WRIST_R"]["controlPos"] = 27;
  yn_map_hw_sot_c["RARM_WRIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current]";

  yn_map_hw_sot_c["LARM_SHOULDER_P"];
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["controlPos"] = 28;
  yn_map_hw_sot_c["LARM_SHOULDER_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_SHOULDER_R"];
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["controlPos"] = 29;
  yn_map_hw_sot_c["LARM_SHOULDER_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_SHOULDER_Y"];
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["controlPos"] = 30;
  yn_map_hw_sot_c["LARM_SHOULDER_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_ELBOW"];
  yn_map_hw_sot_c["LARM_ELBOW"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_ELBOW"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_ELBOW"]["controlPos"] = 31;
  yn_map_hw_sot_c["LARM_ELBOW"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_WRIST_Y"];
  yn_map_hw_sot_c["LARM_WRIST_Y"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_Y"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_Y"]["controlPos"] = 32;
  yn_map_hw_sot_c["LARM_WRIST_Y"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_WRIST_P"];
  yn_map_hw_sot_c["LARM_WRIST_P"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_P"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_P"]["controlPos"] = 33;
  yn_map_hw_sot_c["LARM_WRIST_P"]["sensors"] = "[motor_angle,joint_angle,temperature,current,torque]";

  yn_map_hw_sot_c["LARM_WRIST_R"];
  yn_map_hw_sot_c["LARM_WRIST_R"]["hw"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_R"]["sot"] = "POSITION";
  yn_map_hw_sot_c["LARM_WRIST_R"]["controlPos"] = 34;
  yn_map_hw_sot_c["LARM_WRIST_R"]["sensors"] = "[motor_angle,joint_angle,temperature,current]";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["left_ankle_ft"];
  yn_map_sensors["force_torque"]["left_ankle_ft"]["sensor_joint"] = "LLEG_ANKLE_R";
  yn_map_sensors["force_torque"]["left_ankle_ft"]["frame"] = "LLEG_LINK_6";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["right_ankle_ft"];
  yn_map_sensors["force_torque"]["right_ankle_ft"]["sensor_joint"] = "RLEG_ANKLE_R";
  yn_map_sensors["force_torque"]["right_ankle_ft"]["frame"] = "RLEG_LINK_6";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["left_wrist_ft"];
  yn_map_sensors["force_torque"]["left_wrist_ft"]["sensor_joint"] = "LLEG_WRIST_R";
  yn_map_sensors["force_torque"]["left_wrist_ft"]["frame"] = "LARM_LINK_7";

  yn_map_sensors["force_torque"];
  yn_map_sensors["force_torque"]["right_wrist_ft"];
  yn_map_sensors["force_torque"]["right_wrist_ft"]["sensor_joint"] = "RLEG_WRIST_R";
  yn_map_sensors["force_torque"]["right_wrist_ft"]["frame"] = "RARM_LINK_7";


  YAML::Node yn_imu = yn_map_sensors["imu"];
  yn_imu["base_imu"];
  yn_imu["base_imu"]["frame"] = "LARM_LINK6";
  yn_imu["base_imu"]["gazebo_sensor_J"] = "LARM_LINK6";

  ofstream of;
  of.open("map_hs_sot_gen.yaml", ios::out);
  if (of.is_open()) {
    of << aNode;
  }
  of.close();
}

int ReadYAMLFILE(dg::sot::Device &aDevice, unsigned int debug_mode) {
  std::string yaml_file = "map_hs_sot_gen.yaml";
  YAML::Node map_hs_sot = YAML::LoadFile(yaml_file);

  if (map_hs_sot.IsNull()) {
    std::cerr << "Unable to read " << yaml_file << std::endl;
    return -1;
  }
  if (debug_mode > 0)
    std::cout << "Reading file : " << yaml_file << std::endl;
  YAML::Node map_hs_control = map_hs_sot["map_hardware_sot_control"];
  if (debug_mode > 1) {
    std::cout << "map_hs_control.size(): "
              << map_hs_control.size() << std::endl;
    std::cout << map_hs_control << std::endl;
  }
  unsigned int i = 0;
  for (YAML::const_iterator it = map_hs_control.begin();
       it != map_hs_control.end();
       it++) {
    if (debug_mode > 1) {
      std::cout << i << " " << std::endl;
      std::cout << "key:" << it->first.as<string>() << std::endl;
    }
    std::string jointName = it->first.as<string>();

    YAML::Node aNode = it->second;
    if (debug_mode > 1)
      std::cout << "Type of value: " << aNode.Type() << std::endl;

    for (YAML::const_iterator it2 = aNode.begin();
         it2 != aNode.end();
         it2++)

    {
      std::string aKey = it2->first.as<string>();
      if (debug_mode > 1)
        std::cout << "-- key:" << aKey << std::endl;

      if (aKey == "hw") {
        std::string value = it2->second.as<string>();
        if (debug_mode > 1)
          std::cout << "-- Value: " << value << std::endl;
        aDevice.setHWControlType(jointName, value);
      } else if (aKey == "sot") {
        std::string value = it2->second.as<string>();
        if (debug_mode > 1)
          std::cout << "-- Value: " << value << std::endl;
        aDevice.setSoTControlType(jointName, value);
      } else if (aKey == "controlPos") {
        unsigned int index = it2->second.as<int>();
        if (debug_mode > 1)
          std::cout << "-- index: " << index << std::endl;
        aDevice.setControlPos(jointName, index);
      }
    }
    i++;
  }
  return 0;
}

namespace dg = dynamicgraph;
int main(int, char **) {
  unsigned int debug_mode = 5;

  std::string robot_description;
  ifstream urdfFile;
  std::string filename = "/opt/openrobots/share/simple_humanoid_description/urdf/simple_humanoid.urdf";
  urdfFile.open(filename.c_str());
  if (!urdfFile.is_open()) {
    std::cerr << "Unable to open " << filename << std::endl;
    return -1;
  }
  stringstream strStream;
  strStream << urdfFile.rdbuf();
  robot_description = strStream.str();

  /// Test reading the URDF file.
  dg::sot::Device aDevice(std::string("simple_humanoid"));

  aDevice.setURDFModel(robot_description);
  CreateYAMLFILE();

  if (ReadYAMLFILE(aDevice, debug_mode) < 0)
    return -1;

  dg::Vector aState(29);
  for (unsigned j = 0; j < aState.size(); j++)
    aState(j) = 0.0;
  aDevice.setState(aState);

  /// Fix constant vector for the control entry
  dg::Vector aControlVector(35);
  double dt = 0.005;
  for (unsigned int i = 0; i < 35; i++)
    aControlVector[i] = -0.5;
  aDevice.controlSIN.setConstant(aControlVector);

  for (unsigned int i = 0; i < 2000; i++)
    aDevice.increment(dt);

  const se3::Model & aModel = aDevice.getModel();

  const dg::Vector & aPosition = aDevice.stateSOUT_(2001);
  double diff = 0, ldiff;

  dgsot::JointSHWControlType_iterator it_control_type;
  for (it_control_type  = aDevice.jointDevices_.begin();
       it_control_type != aDevice.jointDevices_.end();
       it_control_type++) 
  {
    int lctl_index = it_control_type->second.control_index;
    if (it_control_type->second.HWcontrol == dgsot::POSITION) 
    {
      int u_index = it_control_type->second.urdf_index;
      if (u_index != -1) 
      {
        std::vector< ::urdf::LinkSharedPtr > urdf_links;
        vector< ::urdf::JointSharedPtr > urdf_joints;
        aModel.getLinks(urdf_links);
        for (unsigned j=0; j<urdf_links.size(); j++)
        {
          std::vector<urdf::JointSharedPtr> child_joints = urdf_links[j]->child_joints;
          urdf_joints.insert(urdf_joints.end(), boost::make_move_iterator(child_joints.begin()), 
                             boost::make_move_iterator(child_joints.end()));
        }

        double lowerLim = urdf_joints[u_index]->limits->lower;
        ldiff = (aPosition[lpos_index] - lowerLim);
        diff += ldiff;
        std::cout << ldiff << " " << urdf_joints[u_index]->name << " "
                  << aPosition[lpos_index]  << " "
                  << lowerLim << " "
                  << -urdf_joints[u_index]->limits->velocity
                  << std::endl;
      }
    }
  }

  if (diff > 1e-3)
    return -1;
  return 0;
}

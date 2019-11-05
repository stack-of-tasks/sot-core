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

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot/core/generic-device.hh>

#include <sstream>
#include <fstream>


#define BOOST_TEST_MODULE test-device

void CreateYAMLFILE() {
  YAML::Emitter yaml_out;
  YAML::Node aNode, yn_map_sot_controller, yn_map_rc_to_sot_device, yn_map_joint_names, yn_control_mode;
  yn_map_sot_controller = aNode["sot_controller"];
  yn_map_rc_to_sot_device = yn_map_sot_controller["map_rc_to_sot_device"];
  yn_map_joint_names = yn_map_sot_controller["joint_names"];
  yn_control_mode = yn_map_sot_controller["control_mode"];

  yn_map_rc_to_sot_device["motor-angles"] = "motor-angles";
  yn_map_rc_to_sot_device["joint-angles"] = "joint-angles";
  yn_map_rc_to_sot_device["velocities"] = "velocities";
  yn_map_rc_to_sot_device["forces"] = "forces";
  yn_map_rc_to_sot_device["currents"] = "currents";
  yn_map_rc_to_sot_device["torques"] = "torques";
  yn_map_rc_to_sot_device["accelerometer_0"] = "accelerometer_0";
  yn_map_rc_to_sot_device["gyrometer_0"] = "gyrometer_0";
  yn_map_rc_to_sot_device["control"] = "control";

  yn_map_joint_names.push_back("waist");
  yn_map_joint_names.push_back("LLEG_HIP_P");
  yn_map_joint_names.push_back("LLEG_HIP_R");
  yn_map_joint_names.push_back("LLEG_HIP_Y");
  yn_map_joint_names.push_back("LLEG_KNEE");
  yn_map_joint_names.push_back("LLEG_ANKLE_P");
  yn_map_joint_names.push_back("LLEG_ANKLE_R");
  yn_map_joint_names.push_back("RLEG_HIP_P");
  yn_map_joint_names.push_back("RLEG_HIP_R");
  yn_map_joint_names.push_back("RLEG_HIP_Y");
  yn_map_joint_names.push_back("RLEG_KNEE");
  yn_map_joint_names.push_back("RLEG_ANKLE_P");
  yn_map_joint_names.push_back("RLEG_ANKLE_R");
  yn_map_joint_names.push_back("WAIST_P");
  yn_map_joint_names.push_back("WAIST_R");
  yn_map_joint_names.push_back("CHEST");
  yn_map_joint_names.push_back("RARM_SHOULDER_P");
  yn_map_joint_names.push_back("RARM_SHOULDER_R");
  yn_map_joint_names.push_back("RARM_SHOULDER_Y");
  yn_map_joint_names.push_back("RARM_ELBOW");
  yn_map_joint_names.push_back("RARM_WRIST_Y");
  yn_map_joint_names.push_back("RARM_WRIST_P");
  yn_map_joint_names.push_back("RARM_WRIST_R");
  yn_map_joint_names.push_back("LARM_SHOULDER_P");
  yn_map_joint_names.push_back("LARM_SHOULDER_R");
  yn_map_joint_names.push_back("LARM_SHOULDER_Y");
  yn_map_joint_names.push_back("LARM_ELBOW");
  yn_map_joint_names.push_back("LARM_WRIST_Y");
  yn_map_joint_names.push_back("LARM_WRIST_P");
  yn_map_joint_names.push_back("LARM_WRIST_R");

  yn_control_mode["waist"];
  yn_control_mode["waist"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_HIP_P"];
  yn_control_mode["LLEG_HIP_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_HIP_R"];
  yn_control_mode["LLEG_HIP_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_HIP_Y"];
  yn_control_mode["LLEG_HIP_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_KNEE"];
  yn_control_mode["LLEG_KNEE"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_ANKLE_P"];
  yn_control_mode["LLEG_ANKLE_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LLEG_ANKLE_R"];
  yn_control_mode["LLEG_ANKLE_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_HIP_P"];
  yn_control_mode["RLEG_HIP_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_HIP_R"];
  yn_control_mode["RLEG_HIP_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_HIP_Y"];
  yn_control_mode["RLEG_HIP_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_KNEE"];
  yn_control_mode["RLEG_KNEE"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_ANKLE_P"];
  yn_control_mode["RLEG_ANKLE_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RLEG_ANKLE_R"];
  yn_control_mode["RLEG_ANKLE_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["WAIST_P"];
  yn_control_mode["WAIST_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["WAIST_R"];
  yn_control_mode["WAIST_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["CHEST"];
  yn_control_mode["CHEST"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_SHOULDER_P"];
  yn_control_mode["RARM_SHOULDER_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_SHOULDER_R"];
  yn_control_mode["RARM_SHOULDER_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_SHOULDER_Y"];
  yn_control_mode["RARM_SHOULDER_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_ELBOW"];
  yn_control_mode["RARM_ELBOW"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_WRIST_Y"];
  yn_control_mode["RARM_WRIST_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_WRIST_P"];
  yn_control_mode["RARM_WRIST_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["RARM_WRIST_R"];
  yn_control_mode["RARM_WRIST_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_SHOULDER_P"];
  yn_control_mode["LARM_SHOULDER_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_SHOULDER_R"];
  yn_control_mode["LARM_SHOULDER_R"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_SHOULDER_Y"];
  yn_control_mode["LARM_SHOULDER_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_ELBOW"];
  yn_control_mode["LARM_ELBOW"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_WRIST_Y"];
  yn_control_mode["LARM_WRIST_Y"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_WRIST_P"];
  yn_control_mode["LARM_WRIST_P"]["ros_control_mode"] = "POSITION";

  yn_control_mode["LARM_WRIST_R"];
  yn_control_mode["LARM_WRIST_R"]["ros_control_mode"] = "POSITION";

  yn_map_sot_controller["left_ft_sensor"] = "left_ankle_ft";

  yn_map_sot_controller["right_ft_sensor"] = "right_ankle_ft";

  yn_map_sot_controller["left_wrist_ft_sensor"] = "left_wrist_ft";

  yn_map_sot_controller["right_ft_wrist_sensor"] = "right_wrist_ft";

  yn_map_sot_controller["base_imu_sensor"] = "base_imu";

  ofstream of;
  of.open("map_hs_sot_gen.yaml", ios::out);
  if (of.is_open()) {
    of << aNode;
  }
  of.close();
}

int ReadYAMLFILE(dg::sot::GenericDevice &aDevice) {
  // Reflect how the data are splitted in two yaml files in the sot
  // Comment and use the commented code to use the above yaml file
  std::ifstream yaml_file_controller("../../unitTesting/tools/sot_controller.yaml");
  std::string yaml_string_controller;
  yaml_string_controller.assign((std::istreambuf_iterator<char>(yaml_file_controller) ),
                                (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_controller);

  std::ifstream yaml_file_params("../../unitTesting/tools/sot_params.yaml");
  std::string yaml_string_params;
  yaml_string_params.assign((std::istreambuf_iterator<char>(yaml_file_params) ),
                            (std::istreambuf_iterator<char>()    ) );
  aDevice.ParseYAMLString(yaml_string_params);

  // Uncomment if you want to use the above yaml file
  // All the data are in one file, which does not reflect reality

  // std::ifstream yaml_file("map_hs_sot_gen.yaml");
  // std::string yaml_string;
  // yaml_string.assign((std::istreambuf_iterator<char>(yaml_file) ),
  //                               (std::istreambuf_iterator<char>()    ) );
  // aDevice.ParseYAMLString(yaml_string);
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
  dg::sot::GenericDevice aDevice(std::string("simple_humanoid"));
  aDevice.setDebugMode(debug_mode);
  aDevice.setURDFModel(robot_description);

  // Uncomment if you want to create and use the above yaml file
  // All the data are in one file, which does not reflect reality
  // CreateYAMLFILE();

  if (ReadYAMLFILE(aDevice) < 0)
    return -1;

  /// Fix constant vector for the control entry in position
  dg::Vector aStateVector(30);
  for (unsigned int i = 0; i < 30; i++) {
    aStateVector[i] = -0.5;
  }
  aDevice.stateSIN.setConstant(aStateVector); // entry signal in position

  for (unsigned int i = 0; i < 2000; i++)
    aDevice.motorcontrolSOUT_.recompute(i);

  const urdf::ModelInterfaceSharedPtr aModel = aDevice.getModel();

  const dg::Vector & aControl = aDevice.motorcontrolSOUT_(2001);
  double diff = 0, ldiff;

  vector< ::urdf::JointSharedPtr > urdf_joints = aDevice.getURDFJoints();

  dgsot::JointSHWControlType_iterator it_control_type;
  for (it_control_type  = aDevice.jointDevices_.begin();
       it_control_type != aDevice.jointDevices_.end();
       it_control_type++) {
    int lctl_index = it_control_type->second.control_index;
    int u_index = it_control_type->second.urdf_index;
    std::cout << "\n ########### \n " << std::endl;
    std::cout << "urdf_joints: " << urdf_joints[u_index]->name << std::endl;

    if (it_control_type->second.SoTcontrol == dgsot::POSITION) {
      if (u_index != -1 && (urdf_joints[u_index]->limits)) {
        double lowerLim = urdf_joints[u_index]->limits->lower;
        ldiff = (aControl[lctl_index] - lowerLim);
        diff += ldiff;
        std::cout << "Position lowerLim: " << lowerLim << "\n"
                  << "motorcontrolSOUT: " << aControl[lctl_index]  << " -- "
                  << "diff: " << ldiff << "\n"
                  << "Velocity limit: " << urdf_joints[u_index]->limits->velocity
                  << std::endl;
      }
    } else if (it_control_type->second.SoTcontrol == dgsot::TORQUE) {
      if (u_index != -1 && (urdf_joints[u_index]->limits)) {
        double lim = urdf_joints[u_index]->limits->effort;
        ldiff = (aControl[lctl_index] - lim);
        diff += ldiff;
        std::cout << "Torque Lim: " << lim << "\n"
                  << "motorcontrolSOUT: " << aControl[lctl_index]  << " -- "
                  << "diff: " << ldiff << "\n"
                  << std::endl;
      }
    } else {
      std::cout << "motorcontrolSOUT: " << aControl[lctl_index] << std::endl;
    }
  }
  std::cout << "\n ########### \n " << std::endl;
  std::cout << "totalDiff: " << diff << std::endl;

  return 0;
}

/*
 * Copyright 2010-2018, CNRS
 * Florent Lamiraux
 * Olivier Stasse
 *
 * CNRS
 *
 * See LICENSE.txt
 */

#ifndef SOT_DEVICE_HH
#define SOT_DEVICE_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <vector>
#include <iterator>

/// URDF DOM
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>

/// YAML CPP
#include <yaml-cpp/yaml.h>

/* -- MaaL --- */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;
/* SOT */
/// dg
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
/// sot-core
#include "sot/core/periodic-call.hh"
#include <sot/core/matrix-geometry.hh>
#include "sot/core/api.hh"
#include <sot/core/abstract-sot-external-interface.hh>

namespace dgsot = dynamicgraph::sot;
namespace dg = dynamicgraph;

namespace dynamicgraph {
namespace sot {

/// Specifies the nature of one joint control
/// It is used for both the SoT side and the hardware side.
enum ControlType {
  POSITION = 0,
  TORQUE = 1,
  VELOCITY = 2
};

const std::string ControlType_s[] = {
  "POSITION", "TORQUE", "VELOCITY"
};

//@}

/// \brief Store information on each joint.
struct JointSoTHWControlType {
  /// Defines the control from the Stack-of-Tasks side (for instance, position)
  ControlType SoTcontrol;
  /// Defines the control from the hardware side.
  ControlType HWcontrol;
  /// Position of the joint in the control vector.
  int control_index;
  /// Position of the joint in the URDF index.
  int urdf_index;

  /// Various indexes for the sensor signals.
  /// This may vary if some joints does not support this feature.
  ///@{
  /// Position of the joint in the temperature vector
  int temperature_index;

  /// Position of the joint in the velocity vector
  int velocity_index;

  /// Position of the joint in the current vector
  int current_index;

  /// Position of the joint in the torque vector
  int torque_index;

  /// Position of the joint in the force vector
  int force_index;

  /// Position of the joint in the joint-angles vector
  int joint_angle_index;

  /// Position of the joint in the motor-angles vector
  int motor_angle_index;

  ///@}
  JointSoTHWControlType();
};

struct IMUSOUT {
  std::string imu_sensor_name;
  dg::Signal<MatrixRotation, int> attitudeSOUT;
  dg::Signal<dg::Vector, int> accelerometerSOUT;
  dg::Signal<dg::Vector, int> gyrometerSOUT;
  IMUSOUT(const std::string &limu_sensor_name,
          const std::string &device_name):
    imu_sensor_name(limu_sensor_name)
    , attitudeSOUT("Device(" + device_name +
                   ")::output(vector6)::" + imu_sensor_name + "_attitudeSOUT")
    , accelerometerSOUT("Device(" + device_name +
                        ")::output(vector3)::" + imu_sensor_name + "_accelerometerSOUT")
    , gyrometerSOUT("Device(" + device_name +
                    ")::output(vector3)::" + imu_sensor_name + "_gyrometerSOUT")
  {}
};

typedef std::map<std::string, JointSoTHWControlType>::iterator
JointSHWControlType_iterator;
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOT_CORE_EXPORT Device: public Entity 
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName(void) const {
    return CLASS_NAME;
  }
  static const double TIMESTEP_DEFAULT;

  /// Maps of joint devices.
  std::map<std::string, dgsot::JointSoTHWControlType> jointDevices_;

  /// Set integration time.
  void timeStep(double ts) { timestep_ = ts;}
  /// Set debug mode.
  void setDebugMode(int mode) { debug_mode_ = mode;}

 protected:
  /// \brief Current integration step.
  double timestep_;

  /// \name Vectors related to the state.
  ///@{

  /// Control vector of the robot wrt urdf file.
  Eigen::VectorXd control_;

  ///@}


  bool sanityCheck_;

  /// Specifies the control input by each element of the state vector.
  std::map<std::string, ControlType> sotControlType_;
  std::map<std::string, ControlType> hwControlType_;

  ///
  PeriodicCall periodicCallBefore_;
  PeriodicCall periodicCallAfter_;

 public:

  /* --- CONSTRUCTION --- */
  Device(const std::string& name);
  /* --- DESTRUCTION --- */
  virtual ~Device();

  virtual void setControl(const dg::Vector& cont);

  /// Set control input type.
  virtual void setSoTControlType(const std::string &jointNames,
                                 const std::string &sotCtrlType);
  virtual void setHWControlType(const std::string &jointNames,
                                const std::string &hwCtrlType);
  virtual void increment();
  /// Read directly the URDF model
  void setURDFModel(const std::string &aURDFModel);

  /// \name Sanity check parameterization
  /// \{
  void setSanityCheck   (const bool & enableCheck);
  /// \}

  /// \name Set index in vector (position, velocity, acceleration, control)
  /// \{
  void setControlPos(const std::string &jointName,
                     const unsigned & index);
  /// \}
 public: /* --- DISPLAY --- */
  virtual void display(std::ostream& os) const;
  SOT_CORE_EXPORT friend std::ostream&
  operator<<(std::ostream& os, const Device& r) {
    r.display(os); return os;
  }

 public: /* --- SIGNALS --- */

  /// Input signal handling the control vector
  /// This entity needs a control vector to be send to the hardware.
  /// The control vector can be position and effort.
  /// It depends on each of the actuator
  dg::SignalPtr<dg::Vector, int> controlSIN;


  /// \name Device current state.
  /// \{
  /// \brief Output attitude provided by the hardware
  /*! \brief The current state of the robot from the command viewpoint. */
  dg::Signal<dg::Vector, int> motorcontrolSOUT_;
  /// \}

  /// \name Real robot current state
  /// This corresponds to the real encoders values and take into
  /// account the stabilization step. Therefore, this usually
  /// does *not* match the state control input signal.
  /// \{
  /// Motor positions
  dg::Signal<dg::Vector, int> robotState_;
  /// Motor velocities
  dg::Signal<dg::Vector, int> robotVelocity_;
  /// The force torque sensors
  std::vector<dg::Signal<dg::Vector, int>*> forcesSOUT_;
  /// The imu sensors
  std::vector<IMUSOUT *> imuSOUT_;
  /// Motor or pseudo torques (estimated or build from PID)
  dg::Signal<dg::Vector, int> * pseudoTorqueSOUT_;
  /// Temperature signal
  dg::Signal<dg::Vector, int> * temperatureSOUT_;
  /// Current signal
  dg::Signal<dg::Vector, int> * currentsSOUT_;
  /// Motor angles signal
  dg::Signal<dg::Vector, int> * motor_anglesSOUT_;
  /// Joint angles signal
  dg::Signal<dg::Vector, int> * joint_anglesSOUT_;
  /// P gains signal
  dg::Signal<dg::Vector, int> * p_gainsSOUT_;
  /// D gains signal
  dg::Signal<dg::Vector, int> * d_gainsSOUT_;
  /// \}

  /// Parse a YAML string for configuration.
  int ParseYAMLString(const std::string &aYamlString);

  /// \name Robot Side
  ///@{

  /// \brief Allows the robot to fill in the internal variables of the device
  /// to publish data on the signals.
  void setSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);
  void setupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);
  void nominalSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);
  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  /// \brief Provides to the robot the control information.
  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);
  ///@}
  
 protected:

  void setControlType(const std::string &strCtrlType,
                      ControlType &aCtrlType);

  /// \brief Compute the new control, from the given one.
  /// When the control is in position, checks that the position is within bounds.
  /// When the control is in torque, checks that the torque is within bounds.
  virtual void updateControl(const Vector & controlIN);

  /// \name Signals related methods
  ///@{
  /// \brief Creates a signal called Device(DeviceName)::output(vector6)::force_sensor_name
  void CreateAForceSignal(const std::string & force_sensor_name);
  /// \brief Creates a signal called Device(DeviceName)::output(vector6)::imu_sensor_name
  void CreateAnImuSignal(const std::string & imu_sensor_name);

  /// \name YAML related methods
  /// @{
  /// Parse YAML for mapping joint names between hardware and sot
  /// starting from a YAML-cpp node.
  int ParseYAMLMapHardwareJointNames(YAML::Node & map_joint_name);

  /// Parse YAML for mapping control modes between hardware and sot
  /// starting from a YAML-cpp node.
  int ParseYAMLMapHardwareControlMode(YAML::Node & map_control_mode);

  /// Parse YAML for sensors from a YAML-cpp node.
  int ParseYAMLSensors(YAML::Node &map_sensors);

  /// Parse YAML for joint sensors from YAML-cpp node.
  int ParseYAMLJointSensor(YAML::Node &aJointSensors);
  /// @}
  
  /// \brief Creates signals based on the joints information parsed by the YAML string.
  int UpdateSignals();

  ///@}
  /// Get freeflyer pose
  const MatrixHomogeneous& freeFlyerPose() const;

  /// Protected methods for internal variables filling
  void setSensorsForce(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsIMU(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsEncoders(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsVelocities(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);
  void setSensorsTorquesCurrents(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);

  void setSensorsGains(std::map<std::string, dgsot::SensorValues> &SensorsIn, int t);

 private:

  // URDF Model of the robot
  ::urdf::ModelInterfaceSharedPtr model_;
  std::vector< ::urdf::JointSharedPtr > urdf_joints_;

  // Debug mode
  int debug_mode_;

  // Intermediate index when parsing YAML file.
  int temperature_index_, velocity_index_,
      current_index_, torque_index_,
      force_index_, joint_angle_index_,
      motor_angle_index_
      ;

 public:
  const ::urdf::ModelInterfaceSharedPtr & getModel() {
    return model_;
  }

  const std::vector< ::urdf::JointSharedPtr > & getURDFJoints() {
    return urdf_joints_;
  }
};
} // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef SOT_DEVICE_HH */


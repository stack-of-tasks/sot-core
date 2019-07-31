/*
 * Copyright 2019,  CNRS
 * Author: Olivier Stasse
 *
 * Please check LICENSE.txt for licensing
 * 
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#define ENABLE_RT_LOG

#include "sot/core/device.hh"
#include <sot/core/debug.hh>
using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-geometry.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#define DBGFILE "/tmp/device.txt"

#if 0
#define RESETDEBUG5() { std::ofstream DebugFile;  \
    DebugFile.open(DBGFILE,std::ofstream::out);   \
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
    DebugFile << __FILE__ << ":"      \
        << __FUNCTION__ << "(#"     \
        << __LINE__ << "):" << x << std::endl;  \
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;    \
    DebugFile.close();}

#else
// Void the macro
#define RESETDEBUG5()
#define ODEBUG5FULL(x)
#define ODEBUG5(x)
#endif

const std::string Device::CLASS_NAME = "Device";
const double Device::TIMESTEP_DEFAULT = 0.001;


JointSoTHWControlType::JointSoTHWControlType():
  control_index(-1)
  ,urdf_index(-1)
  ,temperature_index(-1)
  ,velocity_index(-1)
  ,current_index(-1)
  ,torque_index(-1)
  ,joint_angle_index(-1)
  ,motor_angle_index(-1)
{
}
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Device::~Device( )
{
  for( unsigned int i=0; i<forcesSOUT_.size(); ++i ) {
    delete forcesSOUT_[i];
  }

  for( unsigned int i=0; i<imuSOUT_.size(); ++i ) {
    delete imuSOUT_[i];
  }

}


Device::Device( const std::string& n )
  :Entity(n)
  ,timestep_(TIMESTEP_DEFAULT)
  ,control_(6)
  ,sanityCheck_(true)
  ,controlSIN( NULL,"Device("+n+")::input(double)::control" )   
  ,stateSOUT_( "Device("+n+")::output(vector)::state" )   
  ,motorcontrolSOUT_   ( "Device("+n+")::output(vector)::motorcontrol" )
  ,previousControlSOUT_( "Device("+n+")::output(vector)::previousControl" )
  ,robotState_     ("Device("+n+")::output(vector)::robotState")
  ,robotVelocity_  ("Device("+n+")::output(vector)::robotVelocity")
  ,forcesSOUT_(0)
  ,imuSOUT_(0)
  ,pseudoTorqueSOUT_(0)
  ,temperatureSOUT_(0)
  ,currentsSOUT_(0)
  ,motor_anglesSOUT_(0)
  ,joint_anglesSOUT_(0)
  ,forceZero6 (6)
  ,debug_mode_(5)
  ,temperature_index_(0)
  ,velocity_index_(0)
  ,current_index_(0)
  ,torque_index_(0)
  ,joint_angle_index_(0)
  ,motor_angle_index_(0)
   
{
  forceZero6.fill (0);

  signalRegistration( controlSIN
          << stateSOUT_
          << robotState_
          << robotVelocity_
          << previousControlSOUT_
          << motorcontrolSOUT_);
  control_.fill(.0); stateSOUT_.setConstant( control_ );


  /* --- Commands --- */
  {
    std::string docstring;
    docstring =
        "\n"
        "    Set state vector value\n"
        "\n";
    addCommand("set",
               new command::Setter<Device, Vector>
               (*this, &Device::setState, docstring));

    void(Device::*setRootPtr)(const Matrix&) = &Device::setRoot;
    docstring = command::docCommandVoid1("Set the root position.",
                                         "matrix homogeneous");
    addCommand("setRoot",
               command::makeCommandVoid1(*this,setRootPtr,
                                         docstring));


    /* SET of SoT control input type per joint */
    addCommand("setSoTControlType",
         command::makeCommandVoid2(*this,&Device::setSoTControlType,
                 command::docCommandVoid2 ("Set the type of control input per joint on the SoT side",
             "Joint name",
             "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));


    /* SET of HW control input type per joint */
    addCommand("setHWControlType",
         command::makeCommandVoid2(*this,&Device::setHWControlType,
                 command::docCommandVoid2 ("Set HW control input type per joint",
             "Joint name",
             "Control type: [TORQUE|ACCELERATION|VELOCITY|POSITION]")
                 ));
    
    docstring =
        "\n"
        "    Enable/Disable sanity checks\n"
        "\n";
    addCommand("setSanityCheck",
               new command::Setter<Device, bool>
               (*this, &Device::setSanityCheck, docstring));


    // Handle commands and signals called in a synchronous way.
    periodicCallBefore_.addSpecificCommands(*this, commandMap, "before.");
    periodicCallAfter_.addSpecificCommands(*this, commandMap, "after.");

  }
}

void Device::setState( const Vector& st )
{
  updateControl(st);
  stateSOUT_ .setConstant( control_ );
  motorcontrolSOUT_ .setConstant( control_ );
}


void Device::setRoot( const Matrix & root )
{
  Eigen::Matrix4d _matrix4d(root);
  MatrixHomogeneous _root(_matrix4d);
  setRoot( _root );
}

void Device::setRoot( const MatrixHomogeneous & worldMwaist )
{
  VectorRollPitchYaw r = (worldMwaist.linear().eulerAngles(2,1,0)).reverse();
  for( unsigned int i=0;i<3;++i )
  {
    control_(i) = worldMwaist.translation()(i);
    control_(i+3) = r(i);
  } 
}


void Device::setSanityCheck(const bool & enableCheck)
{
  sanityCheck_ = enableCheck;
}

void Device::setControlType(const std::string &strCtrlType, ControlType &aCtrlType)
{
  for(int j=0;j<4;j++)
  {
    if (strCtrlType==ControlType_s[j]){
      aCtrlType = (ControlType)j;
    } 
  }
}

void Device::setSoTControlType(const std::string &jointNames, const std::string &strCtrlType)
{
  setControlType(strCtrlType, jointDevices_[jointNames].SoTcontrol);
}

void Device::setHWControlType(const std::string &jointNames, const std::string &strCtrlType)
{
  setControlType(strCtrlType, jointDevices_[jointNames].HWcontrol);
}

void Device::setControlPos(const std::string &jointName, const unsigned & index)
{
  jointDevices_[jointName].control_index = index;
}


void Device::setURDFModel(const std::string &aURDFModel)
{
  model_ = ::urdf::parseURDF(aURDFModel);      
  // if (urdfTree)
  // {
  //   model_.name = urdfTree->getName();
  //   model_.initTree(urdfTree);
  //   // details::parseRootTree(urdfTree->getRoot(),model_,false);
  // }
  if (!model_)
  {
    dgRTLOG()
    << "The XML stream does not contain a valid URDF model." << std::endl;
    return;
  }

  /// Build the map between urdf file and the alphabetical order.
  std::vector< ::urdf::LinkSharedPtr > urdf_links;
  model_->getLinks(urdf_links);
  for (unsigned j=0; j<urdf_links.size(); j++)
  {
    std::vector<urdf::JointSharedPtr> child_joints = urdf_links[j]->child_joints;
    urdf_joints_.insert(urdf_joints_.end(), boost::make_move_iterator(child_joints.begin()), 
                        boost::make_move_iterator(child_joints.end()));
  }

  std::cout << "urdf_joints_.size(): " << urdf_joints_.size() << std::endl;
  for(unsigned int i=0;i<urdf_joints_.size();i++)
  {
    jointDevices_[urdf_joints_[i]->name].urdf_index=i;
    std::cout << "jointDevices_ index: " << i
              << " model_.names[i]: " << urdf_joints_[i]->name
              << std::endl;
  }
  // Initialize urdf file vector.
  control_.resize(urdf_joints_.size());
}

void Device::increment()
{
  int time = stateSOUT_.getTime();
  sotDEBUG(25) << "Time : " << time << std::endl;

  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallBefore_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (before): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (before)" << std::endl;
  }

  /* Force the recomputation of the control. */
  controlSIN( time );
  const Vector & controlIN = controlSIN.accessCopy();
  sotDEBUG(25) << "u" <<time<<" = " << controlIN << endl;  
  updateControl(controlIN);
  
  sotDEBUG(25) << "q" << time << " = " << control_ << endl;

  /* Position the signals corresponding to sensors. */
  stateSOUT_ .setConstant( control_ ); stateSOUT_.setTime( time+1 );


  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try
  {
    periodicCallAfter_.run(time+1);
  }
  catch (std::exception& e)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << e.what () << std::endl;
  }
  catch (const char* str)
  {
    dgRTLOG()
        << "exception caught while running periodical commands (after): "
        << str << std::endl;
  }
  catch (...)
  {
    dgRTLOG()
        << "unknown exception caught while"
        << " running periodical commands (after)" << std::endl;
  }

  // Others signals.
  motorcontrolSOUT_ .setConstant( control_ );
}


void Device::updateControl(const Vector & controlIN)
{
  if (sanityCheck_ && controlIN.hasNaN())
  {
    dgRTLOG () << "Device::integrate: Control has NaN values: " << '\n'
               << controlIN.transpose() << '\n';
    return;
  }

  JointSHWControlType_iterator it_control_type;
  for(it_control_type  = jointDevices_.begin();
      it_control_type != jointDevices_.end();
      it_control_type++)  
  {
    int lctl_index = it_control_type->second.control_index;
    int u_index = it_control_type->second.urdf_index;

    if (lctl_index==-1)
    {
      if (debug_mode_>1)
      {
        std::cerr << "No control index for joint "
                  << urdf_joints_[u_index]->name << std::endl;
      }
      break;
    }
      
    if (u_index!=-1)
    {    
      control_[u_index] = controlIN[lctl_index];

      // Position from control is directly provided.
      if (it_control_type->second.SoTcontrol==POSITION)
      {
        double lowerLim = urdf_joints_[u_index]->limits->lower;
        double upperLim = urdf_joints_[u_index]->limits->upper;
        if (control_[u_index] < lowerLim)
        {
          control_[u_index] = lowerLim;
        }
        else if (control_[u_index] > upperLim)
        {
          control_[u_index] = upperLim;
        }        
      }
    }
  }   
}


int Device::ParseYAMLString(const std::string & aYamlString)
{
  YAML::Node map_global = YAML::Load(aYamlString);

  YAML::Node map_sot_controller = map_global["sot_controller"];

  for (YAML::const_iterator it=map_sot_controller.begin();
       it!=map_sot_controller.end();
       it++)
  {
    if (debug_mode_>1)
    {
      std::cout << "key:" << it->first.as<string>() << std::endl;
    }
    std::string name_elt_of_sot_controller = it->first.as<string>();

    YAML::Node elt_of_sot_controller = it->second;
    
    if (name_elt_of_sot_controller=="map_hardware_sot_control")
    {
      int r=ParseYAMLMapHardware(elt_of_sot_controller);
      if (r<0) return r;
    }
    else if (name_elt_of_sot_controller=="sensors")
    { 
      int r=ParseYAMLSensors(elt_of_sot_controller);
      if (r<0) return r;
    }
  }
  UpdateSignals();
  return 0;
}

int Device::ParseYAMLJointSensor(std::string & joint_name, YAML::Node &aJointSensor)
{
  JointSoTHWControlType & aJointSoTHWControlType = jointDevices_[joint_name];

  if (debug_mode_>1)
  {
    std::cout << "JointSensor for " << joint_name << std::endl;
  }
  
  for (std::size_t i=0;i<aJointSensor.size();i++)
  {
    std::string  aSensor = aJointSensor[i].as<string>();
    if (debug_mode_>1)
    {
      std::cout << "Found " << aSensor<< std::endl;
    }
    if (aSensor=="temperature")
    {
      aJointSoTHWControlType.temperature_index = temperature_index_;
      temperature_index_++;
    }
    else if (aSensor=="velocity")
    {
      aJointSoTHWControlType.velocity_index = velocity_index_;
      velocity_index_++;
    }            
    else if (aSensor=="current")
    {
      aJointSoTHWControlType.current_index = current_index_;
      current_index_++;
    }
    else if (aSensor=="torque")
    {
      aJointSoTHWControlType.torque_index = torque_index_;
      torque_index_++;
    }
    else if (aSensor=="joint_angles")
    {
      aJointSoTHWControlType.joint_angle_index = joint_angle_index_;
      joint_angle_index_++;
    }
    else if (aSensor=="motor_angles")
    {
      aJointSoTHWControlType.motor_angle_index = motor_angle_index_;
      motor_angle_index_++;
    }
  }
  return 0;
}


int Device::ParseYAMLMapHardware(YAML::Node & map_hs_control)
{
  if (debug_mode_>1)
  {
    std::cout << "map_hs_control.size(): "
    << map_hs_control.size() << std::endl;
    std::cout << map_hs_control << std::endl;
  }
  unsigned int i=0;
  for (YAML::const_iterator it=map_hs_control.begin();
       it!=map_hs_control.end();
       it++)
  {
    if (debug_mode_>1)
    {
      std::cout << i << " " << std::endl;
      std::cout << "key:" << it->first.as<string>() << std::endl;
    }

    std::string jointName = it->first.as<string>();
    YAML::Node aNode = it->second;

    if (debug_mode_>1)
    {
      std::cout << "Type of value: " << aNode.Type() << std::endl;
    }

    for (YAML::const_iterator it2=aNode.begin();
         it2!=aNode.end();
         it2++)  
    {
      std::string aKey = it2->first.as<string>();
      if (debug_mode_>1)
      {
        std::cout << "-- key:" << aKey << std::endl;
      }
      if (aKey=="hw")
      {
        std::string value = it2->second.as<string>();
        if (debug_mode_>1)
        {          
          std::cout << "-- Value: " << value << std::endl;
        }
        setHWControlType(jointName,value);
      }
      else if (aKey=="sot")
      {
        std::string value = it2->second.as<string>();
        if (debug_mode_>1)
        {
          std::cout << "-- Value: " << value << std::endl;
        }  
        setSoTControlType(jointName,value);
      }
      else if (aKey=="controlPos")
      {
        unsigned int index= it2->second.as<int>();
        if (debug_mode_>1)
        {
          std::cout << "-- index: " << index << std::endl;
        }  
        setControlPos(jointName,index);
      }
      else if (aKey=="sensors")
      {
        YAML::Node aNode = it2->second;
        if (debug_mode_>1)
        {
          std::cout << "-- type: " << aNode.Type() << std::endl;
        } 
        ParseYAMLJointSensor(jointName,aNode);
      }
    }
    i++;
  }
  return 0;
}

/* Sensor signals */
int Device::ParseYAMLSensors(YAML::Node &map_sensors)
{
  if (map_sensors.IsNull())
  {
    std::cerr << "Device::ParseYAMLString: No sensor detected in YamlString "  << std::endl;
    return -1;
  }

  for (YAML::const_iterator it=map_sensors.begin();
     it!=map_sensors.end();
     it++)
  {
    if (debug_mode_>1)
    {
      std::cout << "sensor_type:" << it->first.as<string>() << std::endl;
    }
    std::string sensor_type = it->first.as<string>();
    
    YAML::Node aNode = it->second;
    if (debug_mode_>1)
    {
      std::cout << "Type of value: " << aNode.Type() << std::endl;
    }
    
    // Iterates over types of node.
    for (YAML::const_iterator it2=aNode.begin();
         it2!=aNode.end();
         it2++)  
    {
      std::string sensor_name = it2->first.as<string>();
      if (debug_mode_>1)
      {
        std::cout << "-- sensor_name:" << sensor_name << std::endl;
      }

      if (sensor_type=="force_torque")
      {
        std::string force_sensor_name = it2->second.as<string>();
        if (debug_mode_>1)
        {
          std::cout << "-- force_sensor_name: " << force_sensor_name << std::endl;
        }
        CreateAForceSignal(force_sensor_name);
      }
      else if (sensor_type=="imu")
      {
        std::string imu_sensor_name = it2->second.as<string>();
        if (debug_mode_>1)
        {
          std::cout << "-- Value: " << imu_sensor_name << std::endl;
        }
        CreateAnImuSignal(imu_sensor_name);
      }
      else 
      {
        std::cerr << "The sensor type " << sensor_type
                  << " is not recognized" << std::endl;
      }
    }
  }
  return 0;
}


void Device::CreateAForceSignal(const std::string & force_sensor_name)
{
  dynamicgraph::Signal<dg::Vector, int> * aForceSOUT_;
  /* --- SIGNALS --- */
  aForceSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector6)::" +
                                        force_sensor_name);
  forcesSOUT_.push_back(aForceSOUT_);
  signalRegistration(*aForceSOUT_);
}

void Device::CreateAnImuSignal(const std::string &imu_sensor_name)
{
  IMUSOUT * anImuSOUT_;
  /* --- SIGNALS --- */
  anImuSOUT_ = new IMUSOUT(imu_sensor_name,getName());
  imuSOUT_.push_back(anImuSOUT_);
  signalRegistration(anImuSOUT_->attitudeSOUT
                     << anImuSOUT_->accelerometerSOUT
                     << anImuSOUT_->gyrometerSOUT);
}


int Device::UpdateSignals()
{
  if ((torque_index_!=0) && (pseudoTorqueSOUT_!=0))
  {
    pseudoTorqueSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector)::ptorque");
  }
  
  if ((current_index_!=0) && (currentsSOUT_!=0))
  {
    currentsSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector)::currents");
  }

  if ((temperature_index_!=0) && (temperatureSOUT_!=0))
  {
    temperatureSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector)::temperatures");
  }

  if ((motor_angle_index_!=0) && (motor_anglesSOUT_!=0))
  {
    motor_anglesSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector)::motor_angles");
  }

  if ((joint_angle_index_!=0) && (joint_anglesSOUT_!=0))
  {
    joint_anglesSOUT_ = new Signal<Vector, int>("Device("+getName()+")::output(vector)::joint_angles");
  }

  return 0;
}



/* --- DISPLAY ------------------------------------------------------------ */

void Device::display ( std::ostream& os ) const
{
  os <<name<<": "<<control_<<endl; 
}

/* Helpers for the controller */
void Device::setSensorsForce(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  Eigen::Matrix<double, 6, 1> dgforces;
  
  sotDEBUGIN(15);
  map<string,dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("forces");
  if (it!=SensorsIn.end())
  {
    // Implements force recollection.
    const vector<double>& forcesIn = it->second.getValues();
    for(std::size_t i=0;i<forcesSOUT_.size();++i)
    {
      sotDEBUG(15) << "Force sensor " << i << std::endl;
      for(int j=0;j<6;++j)
      {
        dgforces(j) = forcesIn[i*6+j];
        sotDEBUG(15) << "Force value " << j << ":" << dgforces(j) << std::endl;
      }
      forcesSOUT_[i]->setConstant(dgforces);
      forcesSOUT_[i]->setTime (t);
    }
  }
  sotDEBUGIN(15);
}

void Device::setSensorsIMU(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  Eigen::Matrix<double,3, 1> aVector3d;
  
  map<string,dgsot::SensorValues>::iterator it;

  //TODO: Confirm if this can be made quaternion
  for(std::size_t k=0;k<imuSOUT_.size();++k)
  {
    it = SensorsIn.find("attitude");
    if (it!=SensorsIn.end())
    {
      const vector<double>& attitude = it->second.getValues ();
      Eigen::Matrix<double, 3, 3> pose;
    
      for (unsigned int i = 0; i < 3; ++i)
      {
        for (unsigned int j = 0; j < 3; ++j)
        {
          pose (i, j) = attitude[i * 3 + j];
        }
      }
      imuSOUT_[k]->attitudeSOUT.setConstant (pose);
      imuSOUT_[k]->attitudeSOUT.setTime (t);
    }

    it = SensorsIn.find("accelerometer_0");
    if (it!=SensorsIn.end())
    {
      const vector<double>& accelerometer =
        SensorsIn ["accelerometer_0"].getValues ();
      for (std::size_t i=0; i<3; ++i)
      {
        aVector3d(i) = accelerometer [i];
      }
      imuSOUT_[k]->accelerometerSOUT.setConstant (aVector3d);
      imuSOUT_[k]->accelerometerSOUT.setTime (t);
    }
      
    it = SensorsIn.find("gyrometer_0");
    if (it!=SensorsIn.end())
    {
      const vector<double>& gyrometer = SensorsIn ["gyrometer_0"].getValues ();
      for (std::size_t i=0; i<3; ++i)
      {
        aVector3d(i) = gyrometer [i];
      }
      imuSOUT_[k]->gyrometerSOUT.setConstant (aVector3d);
      imuSOUT_[k]->gyrometerSOUT.setTime (t);
    }
  }
}

void Device::setSensorsEncoders(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  dg::Vector dgRobotState, motor_angles, joint_angles;
  map<string,dgsot::SensorValues>::iterator it;

  if (motor_anglesSOUT_!=0)
  {
    it = SensorsIn.find("motor-angles");
    if (it!=SensorsIn.end())
    {
      const vector<double>& anglesIn = it->second.getValues();
      dgRobotState.resize (anglesIn.size () + 6);
      motor_angles.resize(anglesIn.size ());
      for (unsigned i = 0; i < 6; ++i)
      {
        dgRobotState (i) = 0.;
      }
      for (unsigned i = 0; i < anglesIn.size(); ++i)
      {
        dgRobotState (i + 6) = anglesIn[i];
        motor_angles(i)= anglesIn[i];
      }
      robotState_.setConstant(dgRobotState);
      robotState_.setTime(t);
      motor_anglesSOUT_->setConstant(motor_angles);
      motor_anglesSOUT_->setTime(t);
    }
  }

  if (joint_anglesSOUT_!=0)
  {
    it = SensorsIn.find("joint-angles");
    if (it!=SensorsIn.end())
    {
      const vector<double>& joint_anglesIn = it->second.getValues();
      joint_angles.resize (joint_anglesIn.size () );
      for (unsigned i = 0; i < joint_anglesIn.size(); ++i)
      { 
        joint_angles (i) = joint_anglesIn[i];
      }
      joint_anglesSOUT_->setConstant(joint_angles);
      joint_anglesSOUT_->setTime(t);
    }
  }
}

void Device::setSensorsVelocities(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  dg::Vector dgRobotVelocity;
  
  map<string,dgsot::SensorValues>::iterator it;
  
  it = SensorsIn.find("velocities");
  if (it!=SensorsIn.end())
  {
    const vector<double>& velocitiesIn = it->second.getValues();
    dgRobotVelocity.resize (velocitiesIn.size () + 6);
    for (unsigned i = 0; i < 6; ++i)
    {
      dgRobotVelocity (i) = 0.;
    }
    for (unsigned i = 0; i < velocitiesIn.size(); ++i)
    {
      dgRobotVelocity (i + 6) = velocitiesIn[i];
    }
    robotVelocity_.setConstant(dgRobotVelocity);
    robotVelocity_.setTime(t);
  }
}

void Device::setSensorsTorquesCurrents(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  dg::Vector torques,currents;
  
  map<string,dgsot::SensorValues>::iterator it;

  if (pseudoTorqueSOUT_!=0)
  {
    it = SensorsIn.find("torques");
    if (it!=SensorsIn.end())
    {
      const std::vector<double>& vtorques = SensorsIn["torques"].getValues();
      torques.resize(vtorques.size());
      for(std::size_t i = 0; i < vtorques.size(); ++i)
      {  
        torques (i) = vtorques [i];
      }
      pseudoTorqueSOUT_->setConstant(torques);
      pseudoTorqueSOUT_->setTime(t);
    }
  }

  if (currentsSOUT_!=0)
  {
    it = SensorsIn.find("currents");
    if (it!=SensorsIn.end())
    {
      const std::vector<double>& vcurrents = SensorsIn["currents"].getValues();
      currents.resize(vcurrents.size());
      for(std::size_t i = 0; i < vcurrents.size(); ++i)
      {
        currents (i) = vcurrents[i];
      }
      currentsSOUT_->setConstant(currents);
      currentsSOUT_->setTime(t);
    }
  }
}

void Device::setSensorsGains(map<string,dgsot::SensorValues> &SensorsIn, int t)
{
  dg::Vector p_gains, d_gains;
  
  map<string,dgsot::SensorValues>::iterator it;
  if (p_gainsSOUT_!=0)
  {
    it = SensorsIn.find("p_gains");
    if (it!=SensorsIn.end())
    {
      const std::vector<double>& vp_gains = SensorsIn["p_gains"].getValues();
      p_gains.resize(vp_gains.size());
      for(std::size_t i = 0; i < vp_gains.size(); ++i)
      {
        p_gains (i) = vp_gains[i];
      }
      p_gainsSOUT_->setConstant(p_gains);
      p_gainsSOUT_->setTime(t);
    }
  }

  if (d_gainsSOUT_!=0)
  {
    it = SensorsIn.find("d_gains");
    if (it!=SensorsIn.end())
    {
      const std::vector<double>& vd_gains = SensorsIn["d_gains"].getValues();
      d_gains.resize(vd_gains.size());
      for(std::size_t i = 0; i < vd_gains.size(); ++i)
      {
        d_gains (i) = vd_gains[i];
      }
      d_gainsSOUT_->setConstant(d_gains);
      d_gainsSOUT_->setTime(t);
    }
  }
}

void Device::setSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  sotDEBUGIN(25) ;
  int t = stateSOUT_.getTime () + 1;

  setSensorsForce(SensorsIn,t);
  setSensorsIMU(SensorsIn,t);
  setSensorsEncoders(SensorsIn,t);
  setSensorsVelocities(SensorsIn,t);
  setSensorsTorquesCurrents(SensorsIn,t);
  setSensorsGains(SensorsIn,t);

  sotDEBUGOUT(25);
}

void Device::setupSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void Device::nominalSetSensors(map<string,dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}


void Device::cleanupSetSensors(map<string, dgsot::SensorValues> &SensorsIn)
{
  setSensors (SensorsIn);
}

void Device::getControl(map<string,dgsot::ControlValues> &controlOut)
{
  ODEBUG5FULL("start");
  sotDEBUGIN(25) ;
  const Vector & controlIN = controlSIN.accessCopy();
  vector<double> lcontrolOut;
  lcontrolOut.resize(controlIN.size());
  
  // Integrate control
  increment();
  sotDEBUG (25) << "control = " << control_ << std::endl;

  ODEBUG5FULL("control = "<< control_);

  // Specify the joint values for the controller.
  JointSHWControlType_iterator it_control_type;
  for(it_control_type  = jointDevices_.begin();
      it_control_type != jointDevices_.end();
      it_control_type++)
  {
    int lctl_index = it_control_type->second.control_index;
    if (it_control_type->second.HWcontrol==TORQUE)
    {
      lcontrolOut[lctl_index] = controlIN[lctl_index];
    }
    else if (it_control_type->second.HWcontrol==POSITION)
    {
      int u_index = it_control_type->second.urdf_index;
      lcontrolOut[lctl_index] = control_[u_index];
    }
  }
  
  controlOut["control"].setValues(lcontrolOut);
  
  ODEBUG5FULL("end");
  sotDEBUGOUT(25) ;
}


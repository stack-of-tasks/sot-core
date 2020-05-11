//=========================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date       Author       Notes
// 29/09/2011 SOH Madgwick Initial release
// 02/10/2011 SOH Madgwick Optimised for reduced CPU load
// 11/05/2017   T Flayols  Make it a dynamic-graph entity
//
//=========================================================================

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/core/madgwickahrs.hh>

#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;
using namespace std;

typedef Vector6d Vector6;

#define PROFILE_MADGWICKAHRS_COMPUTATION "MadgwickAHRS computation"

#define INPUT_SIGNALS m_accelerometerSIN << m_gyroscopeSIN
#define OUTPUT_SIGNALS m_imu_quatSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef MadgwickAHRS EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MadgwickAHRS, "MadgwickAHRS");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
MadgwickAHRS::MadgwickAHRS(const std::string &name)
    : Entity(name), CONSTRUCT_SIGNAL_IN(accelerometer, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(gyroscope, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_OUT(imu_quat, dynamicgraph::Vector,
                           m_gyroscopeSIN << m_accelerometerSIN),
      m_initSucceeded(false), m_beta(betaDef), m_q0(1.0), m_q1(0.0), m_q2(0.0),
      m_q3(0.0), m_sampleFreq(512.0) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init",
             makeCommandVoid1(*this, &MadgwickAHRS::init,
                              docCommandVoid1("Initialize the entity.",
                                              "Timestep in seconds (double)")));
  addCommand("getBeta",
             makeDirectGetter(*this, &m_beta,
                              docDirectGetter("Beta parameter", "double")));
  addCommand("setBeta",
             makeCommandVoid1(
                 *this, &MadgwickAHRS::set_beta,
                 docCommandVoid1("Set the filter parameter beta", "double")));
  addCommand("set_imu_quat",
             makeCommandVoid1(
                 *this, &MadgwickAHRS::set_imu_quat,
                 docCommandVoid1("Set the quaternion as [w,x,y,z]", "vector")));
}

void MadgwickAHRS::init(const double &dt) {
  if (dt <= 0.0)
    return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
  m_sampleFreq = 1.0 / dt;
  m_initSucceeded = true;
}

void MadgwickAHRS::set_beta(const double &beta) {
  if (beta < 0.0 || beta > 1.0)
    return SEND_MSG("Beta must be in [0,1]", MSG_TYPE_ERROR);
  m_beta = beta;
}

void MadgwickAHRS::set_imu_quat(const dynamicgraph::Vector &imu_quat) {
  assert(imu_quat.size() == 4);
  m_q0 = imu_quat[0];
  m_q1 = imu_quat[1];
  m_q2 = imu_quat[2];
  m_q3 = imu_quat[3];
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(imu_quat, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG(
        "Cannot compute signal imu_quat before initialization!");
    return s;
  }
  const dynamicgraph::Vector &accelerometer = m_accelerometerSIN(iter);
  const dynamicgraph::Vector &gyroscope = m_gyroscopeSIN(iter);

  getProfiler().start(PROFILE_MADGWICKAHRS_COMPUTATION);
  {
    // Update state with new measurment
    madgwickAHRSupdateIMU(gyroscope(0), gyroscope(1), gyroscope(2),
                          accelerometer(0), accelerometer(1), accelerometer(2));
    if (s.size() != 4)
      s.resize(4);
    s(0) = m_q0;
    s(1) = m_q1;
    s(2) = m_q2;
    s(3) = m_q3;
  }
  getProfiler().stop(PROFILE_MADGWICKAHRS_COMPUTATION);

  return s;
}

/* --- COMMANDS ------------------------------------------------------ */

/* ------------------------------------------------------------------- */
// ************************ PROTECTED MEMBER METHODS ********************
/* ------------------------------------------------------------------- */

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
double MadgwickAHRS::invSqrt(double x) {
  /*
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;*/
  return (1.0 / sqrt(x)); // we're not in the 70's
}

// IMU algorithm update
void MadgwickAHRS::madgwickAHRSupdateIMU(double gx, double gy, double gz,
                                         double ax, double ay, double az) {
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double o2q0, o2q1, o2q2, o2q3, o4q0, o4q1, o4q2, o8q1, o8q2;
  double q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-m_q1 * gx - m_q2 * gy - m_q3 * gz);
  qDot2 = 0.5 * (m_q0 * gx + m_q2 * gz - m_q3 * gy);
  qDot3 = 0.5 * (m_q0 * gy - m_q1 * gz + m_q3 * gx);
  qDot4 = 0.5 * (m_q0 * gz + m_q1 * gy - m_q2 * gx);

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    o2q0 = 2.0 * m_q0;
    o2q1 = 2.0 * m_q1;
    o2q2 = 2.0 * m_q2;
    o2q3 = 2.0 * m_q3;
    o4q0 = 4.0 * m_q0;
    o4q1 = 4.0 * m_q1;
    o4q2 = 4.0 * m_q2;
    o8q1 = 8.0 * m_q1;
    o8q2 = 8.0 * m_q2;
    q0q0 = m_q0 * m_q0;
    q1q1 = m_q1 * m_q1;
    q2q2 = m_q2 * m_q2;
    q3q3 = m_q3 * m_q3;

    // Gradient decent algorithm corrective step
    s0 = o4q0 * q2q2 + o2q2 * ax + o4q0 * q1q1 - o2q1 * ay;
    s1 = o4q1 * q3q3 - o2q3 * ax + 4.0 * q0q0 * m_q1 - o2q0 * ay - o4q1 +
         o8q1 * q1q1 + o8q1 * q2q2 + o4q1 * az;
    s2 = 4.0 * q0q0 * m_q2 + o2q0 * ax + o4q2 * q3q3 - o2q3 * ay - o4q2 +
         o8q2 * q1q1 + o8q2 * q2q2 + o4q2 * az;
    s3 = 4.0 * q1q1 * m_q3 - o2q1 * ax + 4.0 * q2q2 * m_q3 - o2q2 * ay;
    if (!((s0 == 0.0) && (s1 == 0.0) && (s2 == 0.0) && (s3 == 0.0))) {
      // normalise step magnitude
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= m_beta * s0;
      qDot2 -= m_beta * s1;
      qDot3 -= m_beta * s2;
      qDot4 -= m_beta * s3;
    }
  }

  // Integrate rate of change of quaternion to yield quaternion
  m_q0 += qDot1 * (1.0 / m_sampleFreq);
  m_q1 += qDot2 * (1.0 / m_sampleFreq);
  m_q2 += qDot3 * (1.0 / m_sampleFreq);
  m_q3 += qDot4 * (1.0 / m_sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
  m_q0 *= recipNorm;
  m_q1 *= recipNorm;
  m_q2 *= recipNorm;
  m_q3 *= recipNorm;
}

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void MadgwickAHRS::display(std::ostream &os) const {
  os << "MadgwickAHRS " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}
} // namespace sot
} // namespace dynamicgraph

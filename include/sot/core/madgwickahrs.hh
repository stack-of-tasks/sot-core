//========================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date       Author        Notes
// 29/09/2011 SOH Madgwick  Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// 11/05/2017 T Flayols     Make it a dynamic graph entity
// 26/03/2019 G Buondonno   Converted to double
//
//========================================================================

/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
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

#ifndef __sot_torque_control_madgwickahrs_H__
#define __sot_torque_control_madgwickahrs_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(madgwickahrs_EXPORTS)
#define SOTMADGWICKAHRS_EXPORT __declspec(dllexport)
#else
#define SOTMADGWICKAHRS_EXPORT __declspec(dllimport)
#endif
#else
#define SOTMADGWICKAHRS_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include <sot/core/matrix-geometry.hh>

#include "boost/assign.hpp"

#define betaDef 0.01  // 2 * proportional g

namespace dynamicgraph {
namespace sot {
/** \addtogroup Filters
\section subsec_madgwickahrs MadgwickAHRS filter
\class MadgwickARHS
This class implements the MadgwickAHRS filter as described
in http://x-io.co.uk/res/doc/madgwick_internal_report.pdf
This method uses a gradient descent approach to compute the orientation
from an IMU.

The signals input are:
<ul>
<li>m_accelerometerSIN: \f$[a_x, a_y, a_z]^T\f$ in \f$m.s^{-2}\f$</li>
<li>m_gyroscopeSIN: \f$[g_x, g_y, g_z]^T\f$ in \f$rad.s^{-1}\f$</li>
<li>m_imu_quatSOUT: \f$[q_0, q_1, q_2, q_3]^T\f$ </li> estimated rotation
as a quaternion</li>
</ul>

The internal parameters are:
<ul>
<li>\f$Beta\f$: Gradient step weight (default to 0.01) </li>
<li>\f$m_sampleFref\f$: Sampling Frequency computed from the control
period when using init.</li>
</ul>
*/
class SOTMADGWICKAHRS_EXPORT MadgwickAHRS : public ::dynamicgraph::Entity {
  typedef MadgwickAHRS EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  MadgwickAHRS(const std::string &name);

  void init(const double &dt);
  void set_beta(const double &beta);

  /// Set the quaternion as [w,x,y,z]
  void set_imu_quat(const dynamicgraph::Vector &imu_quat);

  /* --- SIGNALS --- */
  /// ax ay az in m.s-2
  DECLARE_SIGNAL_IN(accelerometer, dynamicgraph::Vector);
  /// gx gy gz in rad.s-1
  DECLARE_SIGNAL_IN(gyroscope, dynamicgraph::Vector);
  /// Estimated orientation of IMU as a quaternion
  DECLARE_SIGNAL_OUT(imu_quat, dynamicgraph::Vector);

 protected:
  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream &os) const;

  /* --- METHODS --- */
  double invSqrt(double x);
  void madgwickAHRSupdateIMU(double gx, double gy, double gz, double ax,
                             double ay, double az);

 protected:
  /// true if the entity has been successfully initialized
  bool m_initSucceeded;
  /// 2 * proportional gain (Kp)
  double m_beta;
  /// quaternion of sensor frame
  double m_q0, m_q1, m_q2, m_q3;
  /// sample frequency in Hz
  double m_sampleFreq;

};  // class MadgwickAHRS
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_madgwickahrs_H__

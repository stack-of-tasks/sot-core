//=====================================================================================================
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
//=====================================================================================================

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

#if defined (WIN32)
#  if defined (madgwickahrs_EXPORTS)
#    define SOTMADGWICKAHRS_EXPORT __declspec(dllexport)
#  else
#    define SOTMADGWICKAHRS_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTMADGWICKAHRS_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <sot/core/matrix-geometry.hh>
#include <map>
#include "boost/assign.hpp"

#define betaDef 0.01 // 2 * proportional g

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTMADGWICKAHRS_EXPORT MadgwickAHRS
          :public::dynamicgraph::Entity
      {
        typedef MadgwickAHRS EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        MadgwickAHRS( const std::string & name );

        void init(const double& dt);
        void set_beta(const double & beta);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(accelerometer,              dynamicgraph::Vector);  /// ax ay az in m.s-2
        DECLARE_SIGNAL_IN(gyroscope,                  dynamicgraph::Vector);  /// gx gy gz in rad.s-1
        DECLARE_SIGNAL_OUT(imu_quat,                  dynamicgraph::Vector);  /// Estimated orientation of IMU as a quaternion

      protected:
        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        /* --- METHODS --- */
        double invSqrt(double x);
        void madgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az) ;
        //void madgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);

      protected:
        bool     m_initSucceeded;        /// true if the entity has been successfully initialized
        double   m_beta;                 /// 2 * proportional gain (Kp)
        double   m_q0, m_q1, m_q2, m_q3; /// quaternion of sensor frame
        double   m_sampleFreq;           /// sample frequency in Hz

      }; // class MadgwickAHRS
    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph

#endif // #ifndef __sot_torque_control_madgwickahrs_H__

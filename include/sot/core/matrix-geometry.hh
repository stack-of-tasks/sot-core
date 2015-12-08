/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_MATRIX_HOMOGENEOUS_H__
#define __SOT_MATRIX_HOMOGENEOUS_H__


/* --- Matrix --- */
#include <sot/core/api.hh>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/eigen-io.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
  namespace sot {
    typedef Eigen::Transform<double,3, Eigen::Affine> SOT_CORE_EXPORT MatrixHomogeneous;
    typedef Eigen::Matrix<double,3,3> SOT_CORE_EXPORT MatrixRotation;
    typedef Eigen::AngleAxis<double> SOT_CORE_EXPORT VectorUTheta;
    typedef Eigen::Quaternion<double> SOT_CORE_EXPORT VectorQuaternion;
    typedef Eigen::Vector3d SOT_CORE_EXPORT VectorRotation;
    typedef Eigen::Vector3d SOT_CORE_EXPORT VectorRollPitchYaw;
    typedef Eigen::Matrix<double,6,6> SOT_CORE_EXPORT MatrixForce;
    typedef Eigen::Matrix<double,6,6> SOT_CORE_EXPORT MatrixTwist; 

    inline void buildFrom (const MatrixHomogeneous& MH, MatrixTwist& MT) {
      
      Eigen::Vector3d _t = MH.translation();
      MatrixRotation R(MH.linear());
      Eigen::Matrix3d Tx;
      Tx << 0, -_t(2), _t(1),
	_t(2), 0, -_t(0),
	-_t(1), _t(0), 0;
      Eigen::Matrix3d sk; sk = Tx*R;
      
      MT.block<3,3>(0,0) = R;
      MT.block<3,3>(0,3) = sk;
      MT.block<3,3>(3,0).setZero();
      MT.block<3,3>(3,3) = R;
    }

  } // namespace sot
} // namespace dynamicgraph

namespace Eigen {
  inline std::ostream& operator << (std::ostream &os, 
				    Eigen::AngleAxis<double> inst) {
    os << inst.toRotationMatrix() <<std::endl;
    return os;
  }
  
  inline std::ostream& operator << (std::ostream &os, 
				    Eigen::Transform<double, 3, Eigen::Affine> inst) {
    os << inst.matrix() <<std::endl;
    return os;
  }
  
  //TODO : CHECK TYPE AND WRITE PROPER
  inline std::istringstream& operator >> (std::istringstream &iss, 
					  Eigen::Transform<double,3,Eigen::Affine> &inst)  {
    std::string ss("new test string");
    Matrix3d m;
    m = AngleAxisd(0.0, Vector3d::UnitZ());
    inst = m;
    iss >> ss;
    return iss;
  }
  
  //TODO : CHECK TYPE AND WRITE PROPER
  inline std::istringstream& operator >> (std::istringstream &iss, 
					  Eigen::AngleAxis<double> &inst)  {
    std::string ss("new test string");
    Matrix3d m;
    m = AngleAxisd(0.0, Vector3d::UnitZ());
    inst = m;
    iss >> ss;
    return iss;
  }
}
 
#endif /* #ifndef __SOT_MATRIX_HOMOGENEOUS_H__ */

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

#include <sot/core/vector-quaternion.hh>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot;

static const double ANGLE_MINIMUM = 0.0001;
static const double SINC_MINIMUM = 1e-8;
static const double COSC_MINIMUM = 2.5e-4;

VectorRotation& VectorQuaternion::
fromMatrix( const MatrixRotation& rot )
{
  sotDEBUGIN(15) ;
  
  const ml::Matrix& rotmat = rot;

  double d0 = rotmat(0,0), d1 = rotmat(1,1), d2 = rotmat(2,2);

  // The trace determines the method of decomposition
  double rr = 1.0 + d0 + d1 + d2;

  double & _x = vector(1);
  double & _y = vector(2);
  double & _z = vector(3);
  double & _r = vector(0);

  if (rr>0)
    {
      double s = 0.5 / sqrt(rr);
      _x = (rotmat(2,1) - rotmat(1,2)) * s;
      _y = (rotmat(0,2) - rotmat(2,0)) * s;
      _z = (rotmat(1,0) - rotmat(0,1)) * s;
      _r = 0.25 / s;
    } 
  else 
    {
      // Trace is less than zero, so need to determine which
      // major diagonal is largest
      if ((d0 > d1) && (d0 > d2)) 
	{
	  double s = 0.5 / sqrt(1 + d0 - d1 - d2);
	  _x = 0.5 * s;
	  _y = (rotmat(0,1) + rotmat(1,0)) * s;
	  _z = (rotmat(0,2) + rotmat(2,0)) * s;
	  _r = (rotmat(1,2) + rotmat(2,1)) * s;
	} 
      else if (d1 > d2) 
	{
	  double s = 0.5 / sqrt(1 + d0 - d1 - d2);
	  _x = (rotmat(0,1) + rotmat(1,0)) * s;
	  _y = 0.5 * s;
	  _z = (rotmat(1,2) + rotmat(2,1)) * s;
	  _r = (rotmat(0,2) + rotmat(2,0)) * s;
	} 
      else
	{
	  double s = 0.5 / sqrt(1 + d0 - d1 - d2);
	  _x = (rotmat(0,2) + rotmat(2,0)) * s;
	  _y = (rotmat(1,2) + rotmat(2,1)) * s;
	  _z = 0.5 * s;
	  _r = (rotmat(0,1) + rotmat(1,0)) * s;
	}
    }

  // TRIAL (1) 

//   const float trace = 1 + rotmat(0,0) + rotmat(1,1) + rotmat(2,2);
//   if( trace>1e-6 )
//     {
//       const float s = sqrt(1+trace) * 2;
//       vector(0) = rotmat(2,1) - rotmat(1,2) / s;
//       vector(1) = (rotmat(0,2) - rotmat(2,0)) / s;
//       vector(2) = (rotmat(1,0) - rotmat(0,1)) / s;
//       vector(3) = s / 4;
//     }
//   else if (rotmat(0,0) > rotmat(1,1) && rotmat(0,0) > rotmat(2,2))
//     {
//       const float s = sqrt(1.0f + rotmat(0,0) - rotmat(1,1) - rotmat(2,2)) * 2;
//       vector(0) = s / 4;
//       vector(1) = (rotmat(1,0) + rotmat(0,1)) / s;
//       vector(2) = (rotmat(0,2) + rotmat(2,0)) / s;
//       vector(3) = (rotmat(2,1) - rotmat(1,2)) / s;
//     }
//   else if (rotmat(1,1) > rotmat(2,2))
//     {
//       const float s = sqrt(1.0f + rotmat(1,1) - rotmat(0,0) - rotmat(2,2)) * 2;
//       vector(0) = (rotmat(1,0) + rotmat(0,1)) / s;
//       vector(1) = s / 4;
//       vector(2) = (rotmat(2,1) + rotmat(1,2)) / s;
//       vector(3) = (rotmat(0,2) - rotmat(2,0)) / s;
//     }
//   else
//     {
//       const float s = sqrt(1.0f + rotmat(2,2) - rotmat(0,0) - rotmat(1,1)) * 2;
//       vector(0) = (rotmat(0,2) + rotmat(2,0)) / s;
//       vector(1) = (rotmat(2,1) + rotmat(1,2)) / s;
//       vector(2) = s / 4;
//       vector(3) = (rotmat(1,0) - rotmat(0,1)) / s;
//     }
  
  sotDEBUGOUT(15) ;
  return *this;
}


#include <sot/core/vector-utheta.hh>

VectorRotation& VectorQuaternion::
fromVector( const VectorUTheta& ut )
{
  sotDEBUGIN(15) ;
  
  double theta = sqrt( ut(0)*ut(0)+ut(1)*ut(1)+ut(2)*ut(2) );
  double si = sin(theta);
  double co = cos(theta);
  vector(0)=ut(0)/si;
  vector(1)=ut(1)/si;
  vector(2)=ut(2)/si;
  vector(3)=co;
    
  sotDEBUGOUT(15) ;
  return *this;
}


MatrixRotation& VectorQuaternion::
toMatrix( MatrixRotation& rot ) const
{
  sotDEBUGIN(15) ;

  ml::Matrix& rotmat = rot;

  const double& _x = vector(1);
  const double& _y = vector(2);
  const double& _z = vector(3);
  const double& _r = vector(0);
  
  double x2 = _x * _x;
  double y2 = _y * _y;
  double z2 = _z * _z;
  double r2 = _r * _r;

  rotmat(0,0) = r2 + x2 - y2 - z2;         // fill diagonal terms
  rotmat(1,1) = r2 - x2 + y2 - z2;
  rotmat(2,2) = r2 - x2 - y2 + z2;

  double xy = _x * _y;
  double yz = _y * _z;
  double zx = _z * _x;
  double rx = _r * _x;
  double ry = _r * _y;
  double rz = _r * _z;

  rotmat(0,1) = 2 * (xy - rz);             // fill off diagonal terms
  rotmat(0,2) = 2 * (zx + ry);
  rotmat(1,0) = 2 * (xy + rz);
  rotmat(1,2) = 2 * (yz - rx);
  rotmat(2,0) = 2 * (zx - ry);
  rotmat(2,1) = 2 * (yz + rx);

  // TRIAL (2)

//   double Nq = w*w + x*x + y*y + z*z;
//   double s= 0.0;  if( Nq>0.0 ) s = 2/Nq;  else s=0.0;

//   double X = x*s; double Y = y*s; double Z = z*s;
//   double wX = w*X; double wY = w*Y; double wZ = w*Z;
//   double xX = x*X; double xY = x*Y; double xZ = x*Z;
//   double yY = y*Y; double yZ = y*Z; double zZ = z*Z;

//   rotmat(0,0) = 1.0-(yY+zZ);
//   rotmat(0,1) = xY-wZ;
//   rotmat(0,2) = xZ+wY;
  
//   rotmat(1,0) = xY+wZ;
//   rotmat(1,1) = 1.0-(xX+zZ);
//   rotmat(1,2) = yZ-wX;
  
//   rotmat(2,0) = xZ-wY;
//   rotmat(2,1) = yZ+wX;
//   rotmat(2,2) = 1.0-(xX+yY);
   

  // TRIAL (1)

        //   const double& a = vector(0);
//   const double& b = vector(1);
//   const double& c = vector(2);
//   const double& d = vector(3);

//   double a2=a*a;
//   double b2=b*b;
//   double c2=c*c;
//   double d2=d*d;

//   const double bc = b*c;
//   const double ad = a*d;
//   const double bd = b*d;
//   const double ac = a*c;
//   const double ab = a*b;
//   const double cd = c*d;

//   rotmat(0,0) =  a2+b2-c2-d2  ;
//   rotmat(1,0) =  bc-ad ;
//   rotmat(2,0) =  ac+bd ;
//   rotmat(0,1) =  ad+bc ;
//   rotmat(1,1) =  a2-b2+c2-d2 ;
//   rotmat(2,1) =  cd-ab ;
//   rotmat(0,2) =  bd-ac ;
//   rotmat(1,2) =  ab+cd ;
//   rotmat(2,2) =  a2-b2-c2+d2 ;

  sotDEBUGOUT(15) ;
  return rot;
}

VectorQuaternion& VectorQuaternion::conjugate(VectorQuaternion& res) const
{
  res.vector(0) = vector(0);
  res.vector(1) = -vector(1);
  res.vector(2) = -vector(2);
  res.vector(3) = -vector(3);
  return res;
}

VectorQuaternion& VectorQuaternion::multiply(const VectorQuaternion& q2, VectorQuaternion& res) const
{
  double & a1 = vector(0);
  double & b1 = vector(1);
  double & c1 = vector(2);
  double & d1 = vector(3);

  double & a2 = q2.vector(0);
  double & b2 = q2.vector(1);
  double & c2 = q2.vector(2);
  double & d2 = q2.vector(3);

  res.vector(0) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
  res.vector(1) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
  res.vector(2) = a1*c2 + c1*a2 + d1*b2 - b1*d2;
  res.vector(3) = a1*d2 + d1*a2 + b1*c2 - c1*b2;

  return res;
}

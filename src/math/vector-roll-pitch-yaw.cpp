/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      VectorRollPitchYaw.cpp
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/debug.h>
using namespace std;
using namespace sot;


static const double ANGLE_MINIMUM = 0.0001;
static const double SINC_MINIMUM = 1e-8;
static const double COSC_MINIMUM = 2.5e-4;

sotVectorRotation& VectorRollPitchYaw::
fromMatrix( const MatrixRotation& rot )
{
  sotDEBUGIN(15) ;
  
  const double & nx = rot(2,2);
  const double & ny = rot(2,1);

  vector(0) = atan2(ny,nx);
  vector(1) = atan2(-rot(2,0),
		    sqrt(ny*ny+nx*nx));
  vector(2) = atan2(rot(1,0),rot(0,0));
  
  sotDEBUGOUT(15) ;
  return *this;
}


MatrixRotation& VectorRollPitchYaw::
toMatrix( MatrixRotation& rot ) const
{
  sotDEBUGIN(15) ;

  const double cr = cos(vector(0)); // ROLL
  const double sr = sin(vector(0));
  const double cp = cos(vector(1)); // PITCH
  const double sp = sin(vector(1));
  const double cy = cos(vector(2)); // YAW
  const double sy = sin(vector(2));
  
  rot(0,0) = cy*cp;
  rot(0,1) = cy*sp*sr-sy*cr;
  rot(0,2) = cy*sp*cr+sy*sr;

  rot(1,0) = sy*cp;
  rot(1,1) = sy*sp*sr+cy*cr;
  rot(1,2) = sy*sp*cr-cy*sr;

  rot(2,0) = -sp;
  rot(2,1) = cp*sr;
  rot(2,2) = cp*cr;

  sotDEBUGOUT(15) ;
  return rot;
}

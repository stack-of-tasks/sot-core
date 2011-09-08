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

#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/debug.hh>
using namespace std;
using namespace dynamicgraph::sot;


VectorRotation& VectorRollPitchYaw::
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

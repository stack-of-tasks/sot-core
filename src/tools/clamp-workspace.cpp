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

#include <cmath>

#include <sot/core/clamp-workspace.hh>

using namespace std;

#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ClampWorkspace,"ClampWorkspace");

ClampWorkspace::
ClampWorkspace( const string& fName )
  : Entity( fName )
  ,positionrefSIN( NULL,"ClampWorkspace("+name+")::input(double)::positionref")
  ,positionSIN( NULL,"ClampWorkspace("+name+")::input(double)::position")

  ,alphaSOUT( boost::bind(&ClampWorkspace::computeOutput,this, _1, _2),
	      positionrefSIN<<positionSIN,
	      "ClampWorkspace("+name+")::output(vector)::alpha" )

  ,alphabarSOUT( boost::bind(&ClampWorkspace::computeOutputBar,this, _1, _2),
	      positionrefSIN<<positionSIN,
	      "ClampWorkspace("+name+")::output(vector)::alphabar" )

  ,handrefSOUT( boost::bind(&ClampWorkspace::computeRef,this, _1, _2),
	      positionrefSIN<<positionSIN,
	      "ClampWorkspace("+name+")::output(vector)::ref" )

  ,timeUpdate(0)

  ,alpha(6,6)
  ,alphabar(6,6)

  ,pd(3)

  ,beta(1)
  ,scale(0)
  ,dm_min(0.019)
  ,dm_max(0.025)
  ,dm_min_yaw(0.019)
  ,dm_max_yaw(0.119)
  ,theta_min(-30.*3.14159/180.)
  ,theta_max(5.*3.14159/180.)
  ,mode(1)
  ,frame(FRAME_POINT)

{
  alpha.fill(0.);
  alphabar.fill(1.);
  bounds[0] = std::make_pair(0.15, 0.5);
  bounds[1] = std::make_pair(-0.4, -0.25);
  bounds[2] = std::make_pair(0.15, 0.55);

  signalRegistration( positionrefSIN<<positionSIN<<
		      alphaSOUT<<alphabarSOUT<<handrefSOUT );
}

void ClampWorkspace::update( int time )
{
  if( time<=timeUpdate ){ return; }

  alpha.fill(0.);
  alphabar.setIdentity();

  const MatrixHomogeneous& posref = positionrefSIN.access( time );
  const MatrixHomogeneous& pos = positionSIN.access ( time );

  MatrixHomogeneous prefMw = posref.inverse();
  prefMp.noalias() = prefMw*pos;
  dynamicgraph::Vector x(3); prefMp.extract(x);

  for(int i = 0; i < 3; ++i) {
    double check_min = std::max(x(i) - bounds[i].first, 0.);
    double check_max = std::max(bounds[i].second - x(i), 0.);
    double dm = std::min(check_min, check_max);

    double Y = (dm - dm_min) / (dm_max - dm_min);
    if(Y < 0){ Y = 0; }
    if(Y > 1){ Y = 1; }

    switch(mode) {
    case 0:
      alpha(i,i) = 0;
      alphabar(i,i) = 1;
      break;
    case 1:
      alpha(i,i) = 1;
      alphabar(i,i) = 0;
      break;
    case 2:
    default:
      alpha(i,i) = 0.5 * ( 1 + tanh(1/Y - 1/(1-Y)) );
      alphabar(i,i) = 1 - alpha(i);
    }

    if(i == 2) {
      Y = (dm - dm_min_yaw) / (dm_max_yaw - dm_min_yaw);
      if(Y < 0){ Y = 0; }
      if(Y > 1){ Y = 1; }
      if(mode == 2) {
	alpha(i+3,i+3) = 0.5 * ( 1 + tanh(1/Y - 1/(1-Y)) );
	alphabar(i+3,i+3) = 1 - alpha(i+3);
      }
    }
  }

  if(frame == FRAME_POINT) {
    MatrixHomogeneous prefMp_tmp = prefMp;
    MatrixHomogeneous pMpref = prefMp.inverse();
    for( int i = 0;i<3;++i ) {
      pMpref(i,3) = 0;
      prefMp_tmp(i,3) = 0;
    }
    MatrixTwist pTpref(pMpref);
    MatrixTwist prefTp(prefMp_tmp);

    alpha.noalias() = pTpref*alpha*prefTp;

    alphabar.noalias() = pTpref*alphabar*prefTp;
  }

  for(int i = 0; i < 3; ++i) {
    pd(i) = 0.5 * (bounds[i].first + bounds[i].second);
  }

  VectorRollPitchYaw rpy;
  rpy(0) = 0;
  rpy(1) = -3.14159256 / 2;
  rpy(2) = theta_min + 
    (theta_max - theta_min) *
    (x(1) - bounds[1].first)/(bounds[1].second - bounds[1].first);
  rpy.toMatrix(Rd);

  handref.buildFrom(Rd, pd);

  timeUpdate = time;
}

dynamicgraph::Matrix&
ClampWorkspace::computeOutput( dynamicgraph::Matrix& res,int time )
{
  update(time);
  res.noalias() = alpha;
  return res;
}

dynamicgraph::Matrix&
ClampWorkspace::computeOutputBar( dynamicgraph::Matrix& res,int time )
{
  update(time);
  res.noalias() = alphabar;
  return res;
}

MatrixHomogeneous&
ClampWorkspace::computeRef( MatrixHomogeneous& res,int time )
{
  update(time);
  res.noalias() = handref;
  return res;
}

void ClampWorkspace::display( std:: ostream& os ) const
{
  os << "ClampWorkspace<" << name << ">" << endl << endl;
  os << "alpha: " << alpha << endl;
  os << "pos in ws: " << prefMp << endl;
  os << "bounds: " << bounds[0].first << " " << bounds[0].second << " "
      << bounds[1].first << " " << bounds[1].second << " "
      << bounds[2].first << " " << bounds[2].second << endl;
}

void ClampWorkspace::commandLine( const std::string& cmdLine,
				 std::istringstream& cmdArgs,
				 std::ostream& os )
{
  if( cmdLine == "help" ) {
    os << "ClampWorkspace:" << endl;
    os << " - beta" << endl;
    os << " - scale" << endl;
    os << " - dm_min" << endl;
    os << " - dm_max" << endl;
    os << " - mode" << endl;
    os << " - bounds" << endl;
  }
  else if( cmdLine == "beta") {
    double tmp;
    if( cmdArgs>>tmp ){ beta = tmp; }
    os << "beta = " << beta << endl;
  }
  else if( cmdLine == "scale" ) {
    double tmp;
    if( cmdArgs>>tmp){ scale = tmp; }
    os << "scale = " << scale << endl;
  }
  else if( cmdLine == "dm_min" ) {
    double tmp;
    if( cmdArgs>>tmp){ dm_min = tmp; }
    os << "dm_min = " << dm_min << endl;
  }
  else if( cmdLine == "dm_max" ) {
    double tmp;
    if( cmdArgs>>tmp){ dm_max = tmp; }
    os << "dm_max = " << dm_max << endl;
  }
  else if( cmdLine == "dm_min_yaw" ) {
    double tmp;
    if( cmdArgs>>tmp){ dm_min_yaw = tmp; }
    os << "dm_min_yaw = " << dm_min_yaw << endl;
  }
  else if( cmdLine == "dm_max_yaw" ) {
    double tmp;
    if( cmdArgs>>tmp){ dm_max_yaw = tmp; }
    os << "dm_max_yaw = " << dm_max_yaw << endl;
  }
  else if( cmdLine == "theta_min" ) {
    double tmp;
    if( cmdArgs>>tmp){ theta_min = tmp*3.14159256/180; }
    os << "theta_min = " << theta_min << endl;
  }
  else if( cmdLine == "theta_max" ) {
    double tmp;
    if( cmdArgs>>tmp){ theta_max = tmp*3.14159256/180; }
    os << "theta_max = " << theta_max << endl;
  }
  else if( cmdLine == "mode" ) {
    int tmp;
    if( cmdArgs>>tmp ){ mode = tmp; }
    os << "mode = " << mode << endl;
  }
  else if( cmdLine == "bounds" ) {
    int i = 0;
    for( ;i<3;++i ) {
      double tmp, tmp2;
      if( cmdArgs>>tmp>>tmp2 ){ bounds[i] = std::make_pair(tmp, tmp2); }
      else{ break; }
    }
    if((i == 3) || !i) {
      os << "bounds: " << bounds[0].first << " " << bounds[0].second << " "
	 << bounds[1].first << " " << bounds[1].second << " "
	 << bounds[2].first << " " << bounds[2].second << endl;
    }
    else {
      os << "syntax error" << endl;
    }
  }
  else if( cmdLine == "frame" ) {
    string prm;
    if(cmdArgs>>prm) {
      if(prm == "point"){ frame = FRAME_POINT; }
      else{ frame = FRAME_REF; }
    }
    if(frame == FRAME_POINT){ os << "frame: point" << endl; }
    else{ os << "frame: ref" << endl; }
  }
  else { Entity::commandLine( cmdLine,cmdArgs,os ); }
}

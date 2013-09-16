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

#include <sot/core/smooth-reach.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SmoothReach, "SmoothReach");

SmoothReach::SmoothReach(const std::string & name)
  : Entity(name)

  ,start(0), goal(0)
  ,startTime(-1), lengthTime(-1)
  ,isStarted(false), isParam(true)

  , startSIN(NULL, "SmoothReach("+name+")::input(vector)::start")
  ,goalSOUT(boost::bind(&SmoothReach::goalSOUT_function, this, _1, _2),
	    sotNOSIGNAL
	    "SmoothReach("+name+")::output(vector)::goal")
{
  sotDEBUGIN(5);

  signalRegistration( startSIN << goalSOUT );

  sotDEBUGOUT(5);
}

double smoothFunction( double x )
{
  return x;
}


dynamicgraph::Vector& SmoothReach::
goalSOUT_function( dynamicgraph::Vector & goal, const int& time)
{
  if( isParam )
    {
      start = startSIN(time);
      startTime = time;

      assert( start.size() == goal.size() );
      isParam = false; isStarted = true;
    }

  if( isReady )
    {
      double x = double(time-start)/length;
      if( x>1 ) x=1;
      double x1 = smoothFunction(x);
      double x0 = 1-x1;
      goal = start*x0 + goal*x1;
    }

  return goal;
}

void SmoothReach::
gset( const dynamicgraph::Vector & goalDes, const int & lengthDes )
{
  goal = goalDes;
  length = lengthDes;
  isParam = true;
}

const dynamicgraph::Vector & SmoothReach::
getGoal( void )   { return goal; }

const int & SmoothReach::
getLength( void ) { return length; }

const int & SmoothReach::
getStart( void )  { return start; }

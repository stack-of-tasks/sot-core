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

/* SOT */

#include <sot-core/debug.h>
class ControlGR__INIT
{
public:ControlGR__INIT( void ) { sot::DebugTrace::openFile(); }
};
ControlGR__INIT ControlGR_initiator;
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot-core/control-gr.h>
#include <sot-core/binary-op.h>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph;
using namespace sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControlGR,"ControlGR");

const double ControlGR::
TIME_STEP_DEFAULT = .001;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_ControlGR_INIT \

ControlGR::
ControlGR( const std::string & name )
 :Entity(name)
 ,TimeStep(0)
 ,matrixASIN(NULL,"ControlGR("+name+")::input(matrix)::matrixA")
 ,accelerationSIN(NULL,"ControlGR("+name+")::input(vector)::acceleration")
 ,gravitySIN(NULL,"ControlGR("+name+")::input(vector)::gravity")
 ,controlSOUT( boost::bind(&ControlGR::computeControl,this,_1,_2),
	       matrixASIN << accelerationSIN << gravitySIN,
	      "ControlGR("+name+")::output(vector)::control" )
{
  init(TimeStep);
  Entity::signalRegistration( matrixASIN << accelerationSIN << gravitySIN << controlSOUT );
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void ControlGR::
init(const double& Stept)
{
  TimeStep = Stept;

  return;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void ControlGR::
display( std::ostream& os ) const
{
  os << "ControlGR "<<getName();
  try{
    os <<"control = "<<controlSOUT; 
  }
  catch (ExceptionSignal e) {}
   os <<" ("<<TimeStep<<") ";
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double& ControlGR::setsize(int dimension)

{
	_dimension = dimension;
        return _dimension;
}

ml::Vector& ControlGR::
computeControl( ml::Vector &tau, int t ) 
{
  sotDEBUGIN(15);	

  const ml::Matrix& matrixA = matrixASIN(t);	  
  const ml::Vector& acceleration = accelerationSIN(t);
  const ml::Vector& gravity = gravitySIN(t);		  
  double tp = 0.0;		  
  unsigned size = acceleration.size();		
  tau.resize(size); 
  //tau*=0;
 /* for(unsigned i = 0u; i < size; ++i)
    {
	tp = gravity(i); 
	tau(i) = acceleration;
	tau(i) *= matrixA;
	tau(i) += tp;
    }*/
  
tau = matrixA*acceleration;
sotDEBUG(15) << "torque = A*ddot(q)= " << matrixA*acceleration << std::endl;
tau += gravity;

/*
tau(1) *= 0;
tau(7) *= 0;
tau(24) *=0;
tau(17)=0;*/

 sotDEBUG(15) << "matrixA =" << matrixA << std::endl;
 sotDEBUG(15) << "acceleration =" << acceleration << std::endl;
 sotDEBUG(15) << "gravity =" << gravity << std::endl;
 sotDEBUG(15) << "gravity compensation torque =" << tau << std::endl;
 sotDEBUGOUT(15);

  return tau;

}




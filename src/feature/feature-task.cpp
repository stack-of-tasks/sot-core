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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot-core/debug.h>
#include <sot-core/feature-task.h>
#include <sot-core/exception-feature.h>
#include <sot-core/task.h>
#include <dynamic-graph/pool.h>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


#include <sot-core/factory.h>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureTask,"FeatureTask");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



FeatureTask::
FeatureTask( const string& pointName )
  : FeatureGeneric( pointName )
{
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


ml::Vector& FeatureTask::
computeError( ml::Vector& res,int time )
{ 
  const ml::Vector& err = errorSIN.access(time);
  const Flags &fl = selectionSIN.access(time);
  const unsigned int & dim = dimensionSOUT(time);

  unsigned int curr = 0;
  res.resize( dim );
  if( err.size()<dim )
    { SOT_THROW ExceptionFeature( ExceptionFeature::UNCOMPATIBLE_SIZE,
				     "Error: dimension uncompatible with des->errorIN size."
				     " (while considering feature <%s>).",getName().c_str() ); }

  FeatureTask * sdes = NULL;
  if( desiredValueSIN )
    {
      FeatureAbstract* sdesAbs = desiredValueSIN(time);
      sdes = dynamic_cast<FeatureTask*>(sdesAbs);
    }
  
  sotDEBUG(15) << "Err = " << err;
  sotDEBUG(25) << "Dim = " << dim << endl;

  if( sdes )
    {
      const ml::Vector& errDes = sdes->errorSIN(time);
      sotDEBUG(15) << "Err* = " << errDes;
      if( errDes.size()<dim )
	{ SOT_THROW ExceptionFeature( ExceptionFeature::UNCOMPATIBLE_SIZE,
					 "Error: dimension uncompatible with des->errorIN size."
					 " (while considering feature <%s>).",getName().c_str() ); }

      for( unsigned int i=0;i<err.size();++i ) if( fl(i) ) 
	if( fl(i) ) res( curr++ ) = err(i)-errDes(i);
    }
  else for( unsigned int i=0;i<err.size();++i )
    if( fl(i) ) res( curr++ ) = -err(i);
  
  return res; 

}



/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void FeatureTask::
display( std::ostream& os ) const
{
  os << "Feature from task <" << getName();
  if( taskPtr ) os <<  ": from task " << taskPtr->getName();
  os << std::endl;
}


void FeatureTask::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "FeatureTask: " 
	 << "  - task [<taskname>] " << std::endl;
    }
  else if( cmdLine == "task" )
    {
      cmdArgs >>std::ws; 
      if( cmdArgs.good() )
	{
	  std::string name; cmdArgs >> name;
	  Task& task = dynamic_cast< Task & > (g_pool .getEntity( name ));
	  taskPtr = &task;

	  errorSIN.plug( &task.errorSOUT );
	  jacobianSIN.plug( &task.jacobianSOUT );
	  activationSIN.plug( &task.featureActivationSOUT );
	} else {
	  if( taskPtr ) os << "task = " << (*taskPtr) << std::endl;
	  else  os << "task = NULL " <<std::endl;
	}
    }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

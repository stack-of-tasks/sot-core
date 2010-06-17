/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeatureTask.cpp
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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot-core/debug.h>
#include <sot-core/feature-task.h>
#include <sot-core/exception-feature.h>
#include <dynamic-graph/pool.h>
using namespace std;

#include <sot-core/factory.h>
SOT_FACTORY_FEATURE_PLUGIN(sotFeatureTask,"FeatureTask");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



sotFeatureTask::
sotFeatureTask( const string& pointName )
  : sotFeatureGeneric( pointName )
{
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


ml::Vector& sotFeatureTask::
computeError( ml::Vector& res,int time )
{ 
  const ml::Vector& err = errorSIN.access(time);
  const sotFlags &fl = selectionSIN.access(time);
  const unsigned int & dim = dimensionSOUT(time);

  unsigned int curr = 0;
  res.resize( dim );
  if( err.size()<dim )
    { SOT_THROW sotExceptionFeature( sotExceptionFeature::UNCOMPATIBLE_SIZE,
				     "Error: dimension uncompatible with des->errorIN size."
				     " (while considering feature <%s>).",getName().c_str() ); }

  sotFeatureTask * sdes = NULL;
  if( desiredValueSIN )
    {
      sotFeatureAbstract* sdesAbs = desiredValueSIN(time);
      sdes = dynamic_cast<sotFeatureTask*>(sdesAbs);
    }
  
  sotDEBUG(15) << "Err = " << err;
  sotDEBUG(25) << "Dim = " << dim << endl;

  if( sdes )
    {
      const ml::Vector& errDes = sdes->errorSIN(time);
      sotDEBUG(15) << "Err* = " << errDes;
      if( errDes.size()<dim )
	{ SOT_THROW sotExceptionFeature( sotExceptionFeature::UNCOMPATIBLE_SIZE,
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

void sotFeatureTask::
display( std::ostream& os ) const
{
  os << "Feature from task <" << getName();
  if( taskPtr ) os <<  ": from task " << taskPtr->getName();
  os << std::endl;
}


void sotFeatureTask::
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
	  sotTaskAbstract& task = dynamic_cast< sotTaskAbstract & > (pool .getEntity( name ));
	  taskPtr = &task;

	  errorSIN.plug( &task.taskSOUT );
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

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotTaskPD.cpp
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

/* SOT */
#include <sot-core/task-pd.h>
#include <sot-core/sotDebug.h>
using namespace std;



#include <sot-core/factory.h>

SOT_FACTORY_TASK_PLUGIN(sotTaskPD,"TaskPD");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


sotTaskPD::
sotTaskPD( const std::string& n )
  :sotTask(n)
   ,previousError()
   ,beta(1)
   ,errorDotSOUT( boost::bind(&sotTaskPD::computeErrorDot,this,_1,_2),
		  errorSOUT,
		  "sotTaskPD("+n+")::output(vector)::errorDotOUT" )
   ,errorDotSIN(  NULL,
		  "sotTaskPD("+n+")::input(vector)::errorDot" )
{
  taskSOUT.setFunction( boost::bind(&sotTaskPD::computeTaskModif,this,_1,_2) );
  taskSOUT.addDependancy( errorDotSOUT );

  signalRegistration( errorDotSOUT<<errorDotSIN );
  errorDotSIN.plug( &errorDotSOUT );
}


/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

ml::Vector& sotTaskPD::
computeErrorDot( ml::Vector& errorDot,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  const ml::Vector & errCur = errorSOUT(time);
  if(  previousError.size() == errCur.size() )
    {
      errorDot = errCur;
      errorDot -= previousError;
      previousError = errCur;
    }
  else
    {
      errorDot.resize( errCur.size() );
      errorDot.fill(0.);
      previousError = errCur;
    }
  sotDEBUG(15) << "# Out }" << endl;
  return errorDot;
}

sotVectorMultiBound& sotTaskPD::
computeTaskModif( sotVectorMultiBound& task,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  const ml::Vector & errorDot = errorDotSIN(time);
  sotTask::computeTaskExponentialDecrease(task,time);

  sotDEBUG(25) << " Task = " << task;
  sotDEBUG(25) << " edot = " << errorDot;

  for( unsigned int i=0;i<task.size(); ++i )
    { task[i] = task[i].getSingleBound()-( beta * errorDot(i) ); }

  sotDEBUG(15) << "# Out }" << endl;
  return task;
}




/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <sot-core/pool.h>

void sotTaskPD::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "TaskPD: "<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine =="beta" )
    {
      cmdArgs >> ws;
      if( cmdArgs.good() )
	{ cmdArgs >> beta; } else { os << beta; }
    }
  else  //sotTaskPDAbstract::
    sotTask::commandLine( cmdLine,cmdArgs,os );

}

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

/* SOT */
#include <sot-core/task-pd.h>
#include <sot-core/debug.h>

using namespace std;
using namespace sot;
using namespace dynamicgraph;



#include <sot-core/factory.h>

SOT_FACTORY_TASK_PLUGIN(TaskPD,"TaskPD");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


TaskPD::
TaskPD( const std::string& n )
  :Task(n)
   ,previousError()
   ,beta(1)
   ,errorDotSOUT( boost::bind(&TaskPD::computeErrorDot,this,_1,_2),
		  errorSOUT,
		  "sotTaskPD("+n+")::output(vector)::errorDotOUT" )
   ,errorDotSIN(  NULL,
		  "sotTaskPD("+n+")::input(vector)::errorDot" )
{
  taskSOUT.setFunction( boost::bind(&TaskPD::computeTaskModif,this,_1,_2) );
  taskSOUT.addDependency( errorDotSOUT );

  signalRegistration( errorDotSOUT<<errorDotSIN );
  errorDotSIN.plug( &errorDotSOUT );
}


/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

ml::Vector& TaskPD::
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

VectorMultiBound& TaskPD::
computeTaskModif( VectorMultiBound& task,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  const ml::Vector & errorDot = errorDotSIN(time);
  Task::computeTaskExponentialDecrease(task,time);

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

void TaskPD::
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
    Task::commandLine( cmdLine,cmdArgs,os );

}

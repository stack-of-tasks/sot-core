/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      TaskAbstract.cpp
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
#include <sot-core/task-abstract.h>
#include <sot-core/pool.h>

using namespace sot;
using namespace dynamicgraph;


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


TaskAbstract::
TaskAbstract( const std::string& n )
  :Entity(n)
  ,memoryInternal(NULL)
  ,taskSOUT( "sotTaskAbstract("+n+")::output(vector)::task" )
  ,jacobianSOUT( "sotTaskAbstract("+n+")::output(matrix)::jacobian" )
  ,featureActivationSOUT( "sotTaskAbstract("+n+")::output(vector)::activation" )
{
  taskRegistration();
  signalRegistration( taskSOUT<<jacobianSOUT
		      <<featureActivationSOUT );
}


void TaskAbstract::
taskRegistration( void )
{
  sotPool.registerTask(name,this);
}


void TaskAbstract::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "TaskAbstract: " << std::endl
	 << " - memory <CMD> <ARGS>. " << std::endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if ( "memory"==cmdLine )
    {
      if( NULL==memoryInternal )
        { os << "Internal Memory is null." << std::endl; }
      else
        {
          std::string name; cmdArgs >> name;
          memoryInternal->commandLine( name,cmdArgs,os );
        }
    }
  else
    {
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotTaskAbstract.cpp
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
#include <sot-core/sotTaskAbstract.h>
#include <sot-core/sotPool.h>


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


sotTaskAbstract::
sotTaskAbstract( const std::string& n )
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


void sotTaskAbstract::
taskRegistration( void )
{
  sotPool.registerTask(name,this);
}


void sotTaskAbstract::
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


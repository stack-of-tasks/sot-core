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
#include <sot/core/task-abstract.hh>
#include <sot/core/pool.hh>

using namespace dynamicgraph::sot;
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
{
  taskRegistration();
  signalRegistration( taskSOUT<<jacobianSOUT );
}


void TaskAbstract::
taskRegistration( void )
{
  PoolStorage::getInstance()->registerTask(name,this);
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


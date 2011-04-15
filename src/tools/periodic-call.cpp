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
#include <sot/core/periodic-call.hh>
#include <dynamic-graph/pool.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <algorithm>
#include <dynamic-graph/python/interpreter.hh>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/exception-factory.h>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



PeriodicCall::
PeriodicCall( void )
  : signalMap()
    ,cmdList()
    ,innerTime( 0 )
  ,py_sh( NULL )
{

}

void PeriodicCall::
setPyInterpreter( dynamicgraph::python::Interpreter* ptr )
{ py_sh = ptr; }

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::
addSignal( const std::string &name, SignalBase<int>& sig )
{
  signalMap[ name ] = &sig;
  return ;
}

void PeriodicCall::
addSignal( const std::string& sigpath )
{
  istringstream sigISS( sigpath );
  SignalBase<int>& signal = ::dynamicgraph::PoolStorage::getInstance()->getSignal( sigISS );
  addSignal( sigpath,signal );
  return ;
}

void PeriodicCall::
rmSignal( const std::string &name )
{
  signalMap.erase( name );
  return ;
}



/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::
addCmd( const std::string& cmdLine )
{
  cmdList.push_back( cmdLine );
  return ;
}

void PeriodicCall::
rmCmd( const std::string& args )
{
  CmdListType::iterator iter = std::find( cmdList.begin(),cmdList.end(),args);
  if( cmdList.end()!=iter ) { cmdList.erase( iter ); }
  return ;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::
runSignals( const int& t )
{
  for( SignalMapType::iterator iter = signalMap.begin();
       signalMap.end()!=iter; ++iter )
    {
      (*iter).second ->recompute( t );
    }
  return ;
}

void PeriodicCall::
runCmds( void )
{
  if( NULL==py_sh )
    {
      ExceptionTools( ExceptionTools::PY_SHELL_PTR,"Python interpreter not set." );
    }

  for( CmdListType::const_iterator iter = cmdList.begin();
       cmdList.end()!=iter; ++iter )
    {
      py_sh->python( *iter );
    }
  return ;
}

void PeriodicCall::
run( const int & t )
{
  runSignals( t ); runCmds();
  return ;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


void PeriodicCall::
display( std::ostream& os ) const
{
  os <<"  (t=" << innerTime << ")" <<endl;

  os<<" -> SIGNALS:"<<endl;
  for( SignalMapType::const_iterator iter = signalMap.begin();
       signalMap.end()!=iter; ++iter )
    {
      os << " - " << (*iter).first << endl;
    }

  os<<" -> CMDS:"<<endl;
  for( CmdListType::const_iterator iter = cmdList.begin();
       cmdList.end()!=iter; ++iter )
    {
      os << " - " <<  (*iter) << endl;
    }

}

static std::string readLineStr( istringstream& args )
{
  stringbuf* pbuf=args.rdbuf();
  const unsigned int size = pbuf->in_avail();
  char * buffer = new char[ size+1 ];
  pbuf->sgetn( buffer,size );

  buffer[size]='\0';
  std::string res( buffer );
  delete [] buffer;
  return res;
}

bool PeriodicCall::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "PeriodicCall:"<<endl
	 <<"  - addSignal/rmSignal  <int> " <<endl
	 <<"  - addCmd/rmCmd  " <<endl
	 <<"  - runSignal/runCmd " <<endl
	 <<"  - run" <<endl;
    }
  else if( cmdLine == "addSignal" )
    {
      std::string sigpath; cmdArgs >> std::skipws >> sigpath;
      addSignal( sigpath );
    }
  else if( cmdLine == "rmSignal" )
    {
      std::string sigpath; cmdArgs >> std::skipws >> sigpath;
      rmSignal( sigpath );
    }
  else if( cmdLine == "runSignals" )
    {      runSignals( innerTime++ );    }

  else if( cmdLine == "addCmd" )
    {      addCmd( readLineStr(cmdArgs) );    }
  else if( cmdLine == "rmCmd" )
    {      rmCmd( readLineStr(cmdArgs) );    }
  else if( cmdLine == "runCmds" )
    {      runCmds();    }

  else if( cmdLine == "run" )
    {      run( innerTime++ );    }
  else if( cmdLine == "clear" )
    {     clear();    }
  else if( cmdLine == "print" )
    { display(os) ; }
  else { return false; }
  return true;
}

#define ADD_COMMAND( name,def )                                     \
if (commandMap.count(prefix+name) != 0) {                            \
  DG_THROW ExceptionFactory(ExceptionFactory::OBJECT_CONFLICT,        \
			    "Command " + prefix+name +	               \
			    " already registered in Entity.");          \
 }                                                                       \
commandMap.insert( std::make_pair( prefix+name,def ) )


void PeriodicCall::addSpecificCommands(Entity& ent,
				       Entity::CommandMap_t& commandMap,
				       const std::string& prefix )
{
  using namespace dynamicgraph::command;

  /* Explicit typage to help the compiler. */
  boost::function< void( const std::string& ) >
    addSignal  = boost::bind( &PeriodicCall::addSignal, this,_1 ),
    rmSignal = boost::bind( &PeriodicCall::rmSignal, this,_1 );
  boost::function< void( void ) >
    clear  = boost::bind( &PeriodicCall::clear, this );
  boost::function< void( std::ostream& ) >
    disp  = boost::bind( &PeriodicCall::display, this,_1 );

   ADD_COMMAND("addSignal",
   	      makeCommandVoid1(ent,addSignal,
   			       docCommandVoid1("Add the signal to the refresh list",
   					       "string (sig name)")));
   ADD_COMMAND("rmSignal",
	       makeCommandVoid1(ent,rmSignal,
				docCommandVoid1("Remove the signal to the refresh list",
						"string (sig name)")));
   ADD_COMMAND("clear",
	       makeCommandVoid0(ent,clear,
				docCommandVoid0("Clear all signals and commands from the refresh list.")));

   ADD_COMMAND("disp",
	       makeCommandVerbose(ent,disp,
				  docCommandVerbose("Print the list of to-refresh signals and commands.")));

}






/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


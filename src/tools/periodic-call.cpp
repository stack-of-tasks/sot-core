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
#include <sot-core/debug.h>
#include <sot-core/exception-tools.h>
#include <algorithm>
#include <dynamic-graph/python/interpreter.hh>

using namespace std;
using namespace dynamicgraph;
using namespace sot;

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
  SignalBase<int>& signal = g_pool .getSignal( sigISS );
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



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


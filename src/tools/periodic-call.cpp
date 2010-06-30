/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PeriodicCall.cpp
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
#include <sot-core/periodic-call.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/interpreter.h>
#include <sot-core/debug.h>

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
{

}


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
addSignal( istringstream& args )
{
  string signalName; args >> signalName;
  istringstream sigISS( signalName );
  SignalBase<int>& signal = g_pool .getSignal( sigISS );
  addSignal( signalName,signal );
  return ;
}

void PeriodicCall::
rmSignal( const std::string &name )
{
  signalMap.erase( name );
  return ;
}

void PeriodicCall::
rmSignal(  istringstream& args )
{
  string signalName; args >> signalName;
  rmSignal( signalName );
  return ;
}



/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void PeriodicCall::
addCmd( istringstream& args )
{
  stringbuf* pbuf=args.rdbuf();
  const unsigned int size = pbuf->in_avail();
  char *buffer = new char[ size+1 ];
  pbuf->sgetn( buffer,size );

  buffer[size]='\0';
  cmdList.push_back( buffer );
  
  delete buffer;
  return ;
}

void PeriodicCall::
rmCmd( istringstream& args )
{
  stringbuf* pbuf=args.rdbuf();
  const unsigned int size = pbuf->in_avail();
  char * buffer = new char [size+1];
  pbuf->sgetn( buffer,size );
  buffer[size]='\0';
  
  CmdListType::iterator iter = cmdList.begin();
  while( cmdList.end()!=iter )
    {
      const string& str = *iter;
      if( 0==str.compare( 0,size,buffer ) )
	{  iter = cmdList.erase( iter );	}
      else { ++iter; }
      //cout<<str << " <=> " << buffer << ": " << str.compare( 0,size,buffer )<<endl;
    }
	
  delete buffer;
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
  ostringstream onull; onull.clear( ios::failbit );

  for( CmdListType::const_iterator iter = cmdList.begin();
       cmdList.end()!=iter; ++iter )
    {
      istringstream iss( *iter );
      string cmdName; iss >> cmdName;
      g_shell.cmd( cmdName,iss,onull );
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
	 <<"  - run" <<endl
	 <<"  - setTime t " << endl;
    }
  else if( cmdLine == "addSignal" )
    {      addSignal( cmdArgs );    }
  else if( cmdLine == "rmSignal" )
    {      rmSignal( cmdArgs );    }
  else if( cmdLine == "runSignals" )
    {      runSignals( innerTime++ );    }

  else if( cmdLine == "addCmd" )
    {      addCmd( cmdArgs );    }
  else if( cmdLine == "rmCmd" )
    {      rmCmd( cmdArgs );    }
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


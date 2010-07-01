/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      DistantShell.cpp
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

#include <sot-core/distant-shell.h>
#include <sot-core/debug.h>

using namespace sot;
using namespace dynamicgraph;


#ifdef WIN32
#include <Windows.h>
#endif

DistantShell::
~DistantShell( void )
{
  sotDEBUGIN(5);
  endLoop = true;
  
#ifdef HAVE_LIBBOOST_THREAD
  threadLoop->join();
#endif

  //usleep(1000*1000);
  sotDEBUGOUT(5);
}


void DistantShell::
loop( const int lat )
{
  sotDEBUGIN(15);
  while(! endLoop )
    {
      sotDEBUG(45) <<  lat <<std::endl;
      if( filein.loop() )
	{
	  while( filein.ready() )
	    {
	      std::istringstream iss( filein.next() );
	      std::string cmdLine; 
	      sotDEBUG(35) << cmdLine << std::endl;
	      iss>>cmdLine;
	      shell.cmd( cmdLine,iss,os );
	    }
	}
#ifndef WIN32
	usleep( lat*1000 );
#else	  
	Sleep(lat);
#endif
    }
  sotDEBUGOUT(15);
}

void* 
__staticLoop( void* autoref )
{
  DistantShell* ref = static_cast<DistantShell *>( autoref );
  ref->loop();
  return NULL;
}

void DistantShell::
loopInThread( void )
{
  sotDEBUGIN(5);
#ifdef HAVE_LIBBOOST_THREAD
//   pthread_attr_init(&loopThreadAttr);
//   pthread_create(&loopThread,&loopThreadAttr,__staticLoop,this);
//  threadLoop = new boost::thread( boost::bind( &DistantShell::loop,this ) );
  threadLoop = new boost::thread (boost::bind( &DistantShell::loop,this,
					       latency ));
#else
  #ifndef WIN32
  #warning Unable to compile with libboost_thread
  #else
  #pragma message ( "Unable to compile with libboost_thread\n" ) 
  #endif
  sotDEBUG(5) << "No boost:thread. Unable to launch in thread." <<std::endl;
#endif
  sotDEBUGOUT(5);
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */



#include <sot-core/factory.h>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DistantShellPlugin,"DShell");

using namespace std;

DistantShellPlugin::
DistantShellPlugin( const std::string& name )
  : Entity(name)
    , dshell( g_shell,"/tmp/nmansard/"+name )
{
  sotDEBUGINOUT(5);
}


DistantShellPlugin::
~DistantShellPlugin( void )
{
  sotDEBUGINOUT(5);
}


void DistantShellPlugin::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{

  sotDEBUGIN(15) << "CMD = " << cmdLine <<endl;

  if( cmdLine == "help") 
    {
      os << "distant shell: "<<endl
	 << "  - open <file>" << endl
	 << "  - start" << endl
	 << "  - stop" << endl
	 << "  - latency <uint>" << endl;
    }
  else if( cmdLine == "open" )
    {
      std::string f; cmdArgs >> f; 
      dshell.inputFile(f);
    }
  else if( cmdLine == "start" )
    {
      dshell.loopInThread();
    }
  else if( cmdLine == "stop" )
    {
      dshell.terminate();
    }
  else if( cmdLine == "latency" )
    {
      unsigned int l; cmdArgs >> l;
      dshell.setLatency(l);
    }
  else
    Entity::commandLine( cmdLine,cmdArgs,os );
  
  sotDEBUGOUT(15);

}


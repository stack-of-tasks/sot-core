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

#include <sot-core/sequencer.h>
#include <sot-core/debug.h>
#include <sot-core/exception-tools.h>
#include <sot-core/sot.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/interpreter.h>
#include <dynamic-graph/factory.h>

using namespace sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Sequencer,"Sequencer");

Sequencer::
Sequencer( const std::string & name )
  :Entity(name)
  ,timeInit(-1)
  ,playMode(false)
  ,outputStreamPtr(NULL)
  ,noOutput(false)
  ,triggerSOUT( boost::bind(&Sequencer::trigger,this,_1,_2),
		sotNOSIGNAL,
		"Sequencer("+name+")::output(dummy)::trigger" )
{
  sotDEBUGIN(5);
  
  signalRegistration( triggerSOUT );
  triggerSOUT.setNeedUpdateFromAllChildren( true );

  sotDEBUGOUT(5);
}


Sequencer::
~Sequencer( void )
{
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SPECIFIC EVENT ------------------------------------------------------- */
/* --- SPECIFIC EVENT ------------------------------------------------------- */
/* --- SPECIFIC EVENT ------------------------------------------------------- */

class sotEventTaskBased
  : public Sequencer::sotEventAbstract
{
protected:
  TaskAbstract * taskPtr;
  const std::string defaultTaskName;
public:
  sotEventTaskBased( const std::string name = "",TaskAbstract* task = NULL )
    :sotEventAbstract( name )
    ,taskPtr( task )
    ,defaultTaskName("NULL")
  {}

  void init( std::istringstream& cmdArgs )
  {
    cmdArgs >> std::ws;
    if( cmdArgs.good() )
      {
	std::string taskname; cmdArgs >> taskname;
	sotDEBUG(15) << "Add task " << taskname << std::endl;
	taskPtr  
	  = dynamic_cast< TaskAbstract* > (&g_pool.getEntity( taskname ));
      }
  }
  virtual void display( std::ostream& os ) const 
  { if( taskPtr ) os << taskPtr->getName(); else os << "NULL"; }
  virtual const std::string& getName() const 
  { if( taskPtr ) return taskPtr->getName(); else return defaultTaskName; }

};

class sotEventAddATask
  : public sotEventTaskBased
{
public:
  sotEventAddATask( const std::string name = "",TaskAbstract* task=NULL )
    :sotEventTaskBased( name,task )
  {
    eventType = EVENT_ADD;
  }

  void operator()( Sot* sotptr )
  {
    sotDEBUGIN(15);
    sotDEBUG(45) << "Sot = " << sotptr << ". Task = " << taskPtr << "." << std::endl;
    if( (NULL!=sotptr)&&(NULL!=taskPtr) ) sotptr->push(*taskPtr ); 
    sotDEBUGOUT(15);
  }

  virtual void display( std::ostream& os ) const 
  {os << "Add<"; sotEventTaskBased::display(os); os<<">"; }

};


class sotEventRemoveATask
  : public sotEventTaskBased
{
public:
  sotEventRemoveATask( const std::string name = "",TaskAbstract* task=NULL )
    :sotEventTaskBased( name,task )
  {
    eventType = EVENT_RM;
  }

  void operator()( Sot* sotptr )
  { 
    sotDEBUGIN(15);
    sotDEBUG(45) << "Sot = " << sotptr << ". Task = " << taskPtr << "." << std::endl;
    if( (NULL!=sotptr)&&(NULL!=taskPtr) ) sotptr->remove(*taskPtr );
    sotDEBUGOUT(15);
  }

  virtual void display( std::ostream& os ) const 
  { os << "Remove<"; sotEventTaskBased::display(os); os<<">"; }

};


class sotEventCmd
  : public Sequencer::sotEventAbstract
{
protected:
  std::string cmd;

public:
  sotEventCmd( const std::string cmdLine = "" )
    :sotEventAbstract( cmdLine+"<cmd>" )
    ,cmd( cmdLine )
  {
    eventType = EVENT_CMD;
	sotDEBUGINOUT(15);
  }

  void init( std::istringstream& args )
  {
    sotDEBUGIN(15);
    std::stringbuf* pbuf=args.rdbuf();
    const unsigned int size = pbuf->in_avail();
    char* buffer = new char ( size+1 );
    pbuf->sgetn( buffer,size );

    buffer[size]='\0';
    cmd = buffer;
    sotDEBUGOUT(15);
	delete buffer;
  }
  const std::string & getEventCmd() const 
  {	return cmd; }
  virtual void display( std::ostream& os ) const 
  { os << "Run: " << cmd; }
  virtual void operator() ( Sot* /*sotPtr*/ )
  { 
    std::ostringstream onull; onull.clear( std::ios::failbit );
    std::istringstream iss( cmd );
    std::string cmdName; iss >> cmdName;
    g_shell.cmd( cmdName,iss,onull );
  } ;
};


/* --- TASK MANIP ----------------------------------------------------------- */
/* --- TASK MANIP ----------------------------------------------------------- */
/* --- TASK MANIP ----------------------------------------------------------- */

void Sequencer::
addTask( sotEventAbstract* task,const unsigned int timeSpec )
{
  TaskMap::iterator listKey = taskMap.find( timeSpec );
  if( taskMap.end()==listKey )
    {  
      sotDEBUG(15) << "New element at " << timeSpec << std::endl;
      taskMap[timeSpec].push_back( task ); 
    }
  else 
    {
      TaskList& tl = listKey->second;
      tl.push_back( task ); 
  }
}

//rmTask
void Sequencer::
rmTask( int eventType, const std::string & name,const unsigned int time )
{
	TaskMap::iterator listKey = taskMap.find( time );
	if( taskMap.end()!=listKey )	//the time exist
	{
		TaskList& tl = listKey->second;
		for( TaskList::iterator itL = tl.begin();  itL != tl.end(); ++itL)
		{
			if ((*itL)->getEventType() == eventType && (*itL)->getName() == name) 
			{
				tl.remove(*itL);
				break;
			}
		}

		//remove the list if empty
		if (tl.empty())
			taskMap.erase(listKey);
	}
}

//clearAll
void Sequencer::
clearAll( )
{
  TaskMap::iterator itM;
  for(itM = taskMap.begin(); itM != taskMap.end(); ++itM )
  {
	TaskList::iterator itL;
	TaskList& currentMap = itM->second;
	for (itL=currentMap.begin(); itL!=currentMap.end(); ++itL)
		delete (*itL);
	itM->second.clear();
  }
  //remove all the lists
  taskMap.clear();
}
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

int& Sequencer::
trigger( int& dummy,const int& timeSpec )
{
  sotDEBUGIN(15);

  if(! playMode ) return dummy; 
  if( -1==timeInit ) timeInit = timeSpec;

  sotDEBUG(15) << "Ref time: " << (timeSpec-timeInit) << std::endl;
  TaskMap::iterator listKey = taskMap.find( timeSpec-timeInit );
  if( taskMap.end()!=listKey )
    {  
      sotDEBUG(1) << "Time: "<< (timeSpec-timeInit) << ": we've got a task to do!" 
		  << std::endl;
      TaskList & tl = listKey->second;
      for( TaskList::iterator iter=tl.begin();iter!=tl.end();++iter )
	{
	  if( *iter )
	    { 
	      (*iter)->operator() (sotPtr);
	      if( NULL!=outputStreamPtr ) 
		{ 
		  (*outputStreamPtr) << "At time t=" << timeSpec << ": ";
		  (*iter)->display(*outputStreamPtr); 
		  (*outputStreamPtr) << std::endl;
		}
	    }
	}
    }

  sotDEBUGOUT(15);
  return dummy;
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

void Sequencer::
display( std::ostream& os ) const
{
  if (noOutput) return;

  os << "Sequencer " << getName() << "(t0=" << timeInit
     << ",mode=" << ( (playMode)?"play":"pause" ) << "): " << std::endl;
  for( TaskMap::const_iterator iterMap = taskMap.begin();
       iterMap!=taskMap.end();iterMap++ )
    {
      os << " - t=" << (iterMap->first) << ":\t";
      const TaskList & tl = iterMap->second;
      for( TaskList::const_iterator iterList = tl.begin();
	   iterList!=tl.end();iterList++ )
	{
	  (*iterList)->display(os); os << " ";
	}
      os << std::endl;
    }

}


void Sequencer::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  sotDEBUG(25) << "Cmd " << cmdLine <<std::endl;

  if( cmdLine == "help" )
    {
      os << "Sequencer: " << std::endl
	 << " - sot [<sotname>]" << std::endl
	 << " - addEvent: <eventType> <time> <args>" << std::endl
	 << " - rmEvent: <eventType> <time> <args>" << std::endl
	 << " - clear: erase all events" << std::endl
	 << " - start/stop" << std::endl
	 << " - reset: reset the t0 counter. " << std::endl
	 << " - verbose/normal/mute: change output model : detailed/default (normal messages)/only errors" << std::endl;
    }
  else if( cmdLine == "sot" )
    {
      cmdArgs>>std::ws; if( cmdArgs.good() )
	{
	  std::string sotname; cmdArgs >> sotname;
	  Sot * sotptr = dynamic_cast< Sot* > (& (g_pool.getEntity( sotname )));
	  if(! sotptr ) os << "! Entity <" << sotname << "> does not exist "
			   << "(see you later, next patient please!"
			   << std::endl;
	  else setSotRef( sotptr );
	} 
      else
	{ 
	  if( sotPtr ) os << "sot = " << sotPtr->getName() << std::endl;
	  else os << "No sot specified yet. " << std::endl;
	}
    }
  else if( cmdLine == "addEvent" )
    {
      std::string eventType; 
      unsigned int timeref=0;
	

      cmdArgs>>std::ws; if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! addEvent: <eventType> <time> <args>" <<std::endl;} return; }
      cmdArgs >> eventType >> std::ws;

      if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! addEvent: <eventType> <time> <args>" <<std::endl;} return; }
      cmdArgs >> timeref ;
      
      sotEventTaskBased* event;
      if( eventType=="add" )	 event = new sotEventAddATask();
      else if( eventType=="rm" ) event = new sotEventRemoveATask();
      else
	{
	  os<<"! Event type <" <<eventType <<"> is not recognized." <<std::endl; 
	  return; 
	}
      event->init( cmdArgs );
      addTask( event,timeref );
    }
  else if( cmdLine == "rmEvent" )
  {
	  std::string eventType; 
	  unsigned int timeref=0;


	  cmdArgs>>std::ws; if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! rmEvent: <eventType> <time> <args>" <<std::endl;} return; }
	  cmdArgs >> eventType >> std::ws;

	  if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! rmEvent: <eventType> <time> <args>" <<std::endl;} return; }
	  cmdArgs >> timeref ;

	  //the type of event
	  int event;
	  if( eventType=="add" )	 event = sotEventAbstract::EVENT_ADD;
	  else if( eventType=="rm" ) event = sotEventAbstract::EVENT_RM;
	  else
	  {
		  os<<"! Event type <" <<eventType <<"> is not recognized." <<std::endl; 
		  return; 
	  }

	  //the name of the event
	  std::string name;
	  cmdArgs >> name;
	  rmTask( event, name, timeref );
  }
  else if( cmdLine == "addCmd" )
    {
      unsigned int timeref=0; cmdArgs >> std::ws;
      sotDEBUG(15)<<std::endl;
      if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! addEvent: <eventType> <time> <args>" <<std::endl;} return; }
      sotDEBUG(15)<<std::endl;
      cmdArgs >> timeref ;
      sotDEBUG(15)<<std::endl;
      sotEventCmd * eventcmd = new sotEventCmd();
      sotDEBUG(15)<<std::endl;
      eventcmd->init( cmdArgs );
      addTask( eventcmd,timeref );
    }
  else if( cmdLine == "rmCmd" )
    {
      unsigned int timeref=0; cmdArgs >> std::ws;
      if(! cmdArgs.good() ) 
	  { if (!noOutput) {os <<"! addEvent: <eventType> <time> <args>" <<std::endl;} return; }
      cmdArgs >> timeref ;
	  int index = sotEventAbstract::EVENT_CMD;

	  //get the name of the command
	  std::string name;
	  cmdArgs >>  name;
      rmTask( index, name,timeref );
    }
  else if( "clear"==cmdLine ) { clearAll(); }
  else if( "reset"==cmdLine ) { timeInit = -1; }
  else if( "start"==cmdLine ) { playMode = true; }
  else if( "stop"==cmdLine ) { playMode = false; }
  else if( "verbose"==cmdLine ) { outputStreamPtr = &os; }
  else if( "mute"==cmdLine ) { outputStreamPtr = NULL; noOutput=true;}
  else if( "normal"==cmdLine ) { noOutput=false; }
  else { Entity::commandLine( cmdLine,cmdArgs,os); }
}


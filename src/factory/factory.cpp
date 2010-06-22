/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFactory.cpp
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
#include <sot-core/debug.h>
#include <sot-core/factory.h>

using namespace std;
using namespace sot;


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


FactoryStorage::
~FactoryStorage( void )
{
  sotDEBUGINOUT(25);
}

/* --------------------------------------------------------------------- */
void FactoryStorage::
registerTask( const std::string& entname,TaskConstructor_ptr ent )
{
  TaskMap::iterator entkey;
  if( existTask(entname,entkey) ) // key does exist
    {
//       SOT_THROW ExceptionFactory( ExceptionFactory::OBJECT_CONFLICT,
// 				 "Another task class already defined with the same name. ",
// 				 "( while adding task class <%s> inside the factory).",
// 				 entname.c_str() );
      sotERRORF("Another task class already defined with the same name. "
		"( while adding task class <%s> inside the factory).",
		entname.c_str() );
    }
  else
    {
 //      sotDEBUG(10) << "Register task <"<< entname 
// 		   << "> in the factory." <<std::endl;
      taskMap[entname] = ent;
    }
}

TaskAbstract* FactoryStorage::
newTask( const std::string& classname,const std::string& objname  )
{
  TaskMap::iterator entPtr;
  if(! existTask(classname,entPtr) ) // key does not exist
    {
      SOT_THROW ExceptionFactory( ExceptionFactory::UNREFERED_OBJECT,
				     "Unknown task."," (while calling new_task <%s>)",
				     classname.c_str() );
    }
  return entPtr->second(objname);
}
bool FactoryStorage::
existTask( const std::string& name, TaskMap::iterator& entPtr )
{
  entPtr = taskMap .find( name );
  return ( entPtr != taskMap.end() );
}
bool FactoryStorage::
existTask( const std::string& name )
{
  TaskMap::iterator entPtr;return existTask( name,entPtr );
}


/* --------------------------------------------------------------------- */
void FactoryStorage::
registerFeature( const std::string& entname,FeatureConstructor_ptr ent )
{
  FeatureMap::iterator entkey;
  if( existFeature(entname,entkey) ) // key does exist
    {
//       SOT_THROW ExceptionFactory( ExceptionFactory::OBJECT_CONFLICT,
// 				 "Another feature already defined with the same name. ",
// 				 "(while adding feature class <%s> inside the factory).",
// 				 entname.c_str() );
      sotERRORF("Another feature already defined with the same name. "
		"(while adding feature class <%s> inside the factory).",
		entname.c_str() );
   }
  else
    {
//       sotDEBUG(10) << "Register feature <"<< entname 
// 		   << "> in the factory." <<std::endl;
      featureMap[entname] = ent;
    }
}

FeatureAbstract* FactoryStorage::
newFeature( const std::string& classname,const std::string& objname  )
{
  FeatureMap::iterator entPtr;
  if(! existFeature(classname,entPtr) ) // key does not exist
    {
      SOT_THROW ExceptionFactory( ExceptionFactory::UNREFERED_OBJECT,
				     "Unknown feature."," (while calling new_feature <%s>)",
				     classname.c_str() );
    }
  return entPtr->second(objname);
}
bool FactoryStorage::
existFeature( const std::string& name, FeatureMap::iterator& entPtr )
{
  //  sotDEBUGINOUT(25) << "(name=<"<<name<<">)."<<std::endl;
  entPtr = featureMap .find( name );
  //  sotDEBUG(6) << "str: "<< entPtr->first <<std::endl;
  //  sotDEBUG(6) << "ptr: "<< entPtr->second <<std::endl;
  return ( entPtr != featureMap.end() );
}
bool FactoryStorage::
existFeature( const std::string& name )
{
  FeatureMap::iterator entPtr;return existFeature( name,entPtr );
}



/* --------------------------------------------------------------------- */
/* --- REGISTERERS ----------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotFeatureRegisterer::
sotFeatureRegisterer( const std::string& featureClassName,
		      FactoryStorage::FeatureConstructor_ptr maker)
{ 
  //sotDEBUG(3) << "Register feature class: "<< featureClassName << std::endl;
  sotFactory.registerFeature(featureClassName,maker);
}

sotTaskRegisterer::
sotTaskRegisterer( const std::string& taskClassName,
		   FactoryStorage::TaskConstructor_ptr maker) 
{  
  //sotDEBUG(3) << "Register task class: "<< taskClassName << std::endl;
  sotFactory.registerTask(taskClassName,maker);
}


/* --------------------------------------------------------------------- */
/* ---  COMMAND LINE ---------------------------------------------------- */
/* --------------------------------------------------------------------- */
void FactoryStorage::
commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os<< "factory ";
      string cmd2; cmdArgs >> cmd2; 
      if(! cmdArgs.good())  
	os << " <arg>\t\t\t\taccess to the factory (help <arg> for more detail)" <<endl;
      else if( cmd2 == "list" ) 
	os << "list\t\t:List all available creator." << endl;
      else if( cmd2 == "listFeatures" )
	os <<"listFeatures\t:List available features." << endl;
      else if( cmd2 == "listTasks" )
	os << " - listTasks\t:List available tasks." << endl;
    }
  else if( cmdLine == "list" )
    {
      commandLine("listTasks",cmdArgs,os);
      commandLine("listFeatures",cmdArgs,os);
    }
  else if( cmdLine == "listFeatures" )
    {
      os <<" List of available features:" << endl;
      for( FeatureMap::iterator iter = featureMap.begin();iter!=featureMap.end();++iter )
	{ os << "  - " << iter->first << endl; }
    }
  else if( cmdLine == "listTasks" )
    {
      os <<" List of available tasks:" << endl;
      for( TaskMap::iterator iter = taskMap.begin();iter!=taskMap.end();++iter )
	{ os << "  - " << iter->first << endl; }
    }
  return;
}



/// The global sotFactory object
namespace sot {
FactoryStorage sotFactory;
}

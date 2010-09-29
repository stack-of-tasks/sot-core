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

#ifndef __SOT_FACTORY_HH__
#define __SOT_FACTORY_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- STD --- */
#include <map>
#include <string>

/* --- SOT --- */
#include <sot-core/exception-factory.h>
#include <sot-core/sot-core-api.h>
#include <dynamic-graph/factory.h>

namespace sot {

class FeatureAbstract;
class TaskAbstract;

/* --------------------------------------------------------------------- */
/* --- FACTORY ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! @ingroup factory
  \brief This class implements the factory that allows creation of
  objects loaded from dynamic link libraries. Objects are referenced by their
  class names, and can be created by passing this string to one of the three
  public "new" methods (newEntity, newFeature or newTask).
  The factory instance (singleton) is publicly available under the name sotFactory
  (include factory.h).  A task, feature or entity can register itself by
   using the SOT_FACTORY_{ENTITY,TASK,FEATURE}_PLUGIN macro. See Task.cpp for
   an example.
*/
class SOT_CORE_EXPORT FactoryStorage
{
 public:
  typedef FeatureAbstract* (*FeatureConstructor_ptr)( const std::string& );
  typedef TaskAbstract* (*TaskConstructor_ptr)( const std::string& );

 protected:
  typedef std::map< std::string,TaskConstructor_ptr > TaskMap;
  typedef std::map< std::string,FeatureConstructor_ptr > FeatureMap;

  FeatureMap featureMap;
  TaskMap taskMap;

 public:

  ~FactoryStorage( void );

  void registerTask( const std::string& entname,TaskConstructor_ptr ent );
  TaskAbstract* newTask( const std::string& name,const std::string& objname );
  bool existTask( const std::string& name, TaskMap::iterator& entPtr );
  bool existTask( const std::string& name );

  void registerFeature( const std::string& entname,FeatureConstructor_ptr ent );
  FeatureAbstract* newFeature( const std::string& name,const std::string& objname );
  bool existFeature( const std::string& name, FeatureMap::iterator& entPtr );
  bool existFeature( const std::string& name );

  void  commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
		     std::ostream& os );


};

SOT_CORE_EXPORT extern FactoryStorage sotFactory;

/* --- REGISTERER ----------------------------------------------------------- */
/* --- REGISTERER ----------------------------------------------------------- */
/* --- REGISTERER ----------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- ENTITY REGISTERER ---------------------------------------------------- */
/* -------------------------------------------------------------------------- */
typedef dynamicgraph::EntityRegisterer sotEntityRegisterer;

/* -------------------------------------------------------------------------- */
/* --- FEATURE REGISTERER --------------------------------------------------- */
/* -------------------------------------------------------------------------- */

class SOT_CORE_EXPORT sotFeatureRegisterer
{
 private:
  sotFeatureRegisterer( void );

 public:
  sotFeatureRegisterer( const std::string& featureClassName,
			FactoryStorage::FeatureConstructor_ptr maker);
};

#define SOT_FACTORY_FEATURE_PLUGIN(sotFeatureType,className) \
  const std::string sotFeatureType::CLASS_NAME = className;  \
  extern "C" {                                               \
    FeatureAbstract *sotFeatureMaker##_##sotFeatureType( const std::string& objname )\
    {                                                        \
      return new sotFeatureType( objname );                  \
    }                                                        \
  sotFeatureRegisterer reg##_##sotFeatureType( className,    \
					       &sotFeatureMaker##_##sotFeatureType );   \
  }                                                          \


/* -------------------------------------------------------------------------- */
/* --- TASK REGISTERER ------------------------------------------------------ */
/* -------------------------------------------------------------------------- */

class SOT_CORE_EXPORT sotTaskRegisterer
{
 private:
  sotTaskRegisterer( void );

 public:
  sotTaskRegisterer( const std::string& taskClassName,
		     FactoryStorage::TaskConstructor_ptr maker);
};


#define SOT_FACTORY_TASK_PLUGIN(sotTaskType,className) \
  const std::string sotTaskType::CLASS_NAME = className; \
  extern "C" {                                            \
    TaskAbstract *sotTaskMaker##_##sotTaskType( const std::string& objname )    \
    {                                                     \
      return new sotTaskType( objname );                 \
    }                                                     \
  sotTaskRegisterer reg##_##sotTaskType( className,&sotTaskMaker##_##sotTaskType );   \
  }                                                       \

} // namespace sot

#endif /* #ifndef __SOT_FACTORY_HH__ */





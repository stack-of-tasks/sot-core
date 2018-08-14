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

#ifndef __SOT_POOL_HH__
#define __SOT_POOL_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- STD --- */
#include <map>
#include <string>
#include <sstream>

/* --- SOT --- */
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/signal-base.h>
#include "sot/core/api.hh"
#include <dynamic-graph/pool.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {

// Preliminary declarations
class FeatureAbstract;
class TaskAbstract;


/*! @ingroup factory
  \brief This singleton class keep tracks of all features and tasks.

  Three kinds of objects are handled:
  \li The controllers, i.e. the tasks which inherits from TaskAbstract.
  \li The features, i.e. the information which inherits from FeatureAbstract.

  \li Any object which need to be inside the SoT and which inherits from Entity.

  This class provides the necessary operations to register, unregister each
  instance of thoses classes.
  As tasks and features derived from Entities, they should be registered
  as such.

  \note From the code it is not very clear why we should not unregister
  from the tasks and the features...

  The role of this class is also to look for the object supporting
  a command, and to apply this command.

  It also returns references to signals from their fully-qualified names.
 */
class SOT_CORE_EXPORT PoolStorage
{
 public:
  /*! \name Define types to simplify the writing
    @{
   */
  /*! \brief Sorted set of tasks with unique key (name). */
  typedef std::map< std::string,TaskAbstract* > Tasks;

  /*! \brief Sorted set of features with unique key (name). */
  typedef std::map< std::string,FeatureAbstract* > Features;
  /*! @} */

 protected:
  /*! \name Fields of the class to manage the three entities.
    Also the name is singular, those are true sets.
    @{
  */

  /*! \brief Set of controllers */
  Tasks task;

  /*! \brief Set of features */
  Features feature;
  /*! @} */

 public:
  /*! \brief Default destructor */
  ~PoolStorage( void );

  /// \brief Get unique instance of the class
  static PoolStorage* getInstance();

  /// \brief destroy unique instance of the class
  static void destroy();

  /*! \name Methods related to the handling of the features
    @{
   */
  /*! \brief Registering a feature. */
  void registerFeature( const std::string& entname,FeatureAbstract* ent );

  /*! \brief Get a reference to a feature. */
  FeatureAbstract& getFeature( const std::string& name );
  /*! @} */

  /*! \name Methods related to the handling of the tasks
    @{
   */
  /*! \brief Registering a task. */
  void registerTask( const std::string& entname,TaskAbstract* ent );
  /*! \brief Get a reference to a task. */
  TaskAbstract& getTask( const std::string& name );
  /*! @} */

  /*! \brief This method looks for the object named objectName,
    and ask to provide the function functionName with the arguments cmdArg.
    If the method of the object displays some information this will
    be done on os.

  /*! \brief This method write a graph description on the file named FileName. */
  void writeGraph(const std::string &aFileName);
  void writeCompletionList(std::ostream& os);

 private:
  PoolStorage();
  static PoolStorage* instance_;
};

} /* namespace sot */} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_POOL_HH__ */






/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      pool.h
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
#include <sot-core/exception-factory.h>
#include <dynamic-graph/signal-base.h>
#include <sot-core/sot-core-api.h>
#include <dynamic-graph/pool.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

// Preliminary declarations
class FeatureAbstract;
class TaskAbstract;


/*! @ingroup factory
  \brief This class keep tracks of all the objects in the stack of Tasks.

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

    The commands specific to the <b>pool<\b> object are:
    \li <b>list</b> : List all the entities registered in the pool.
    \li <b>listFeature</b> : List all the features registered in the pool.
    \li <b>listTask</b> : List all the tasks registered in the pool.
  */
  void commandLine( const std::string& objectName,const std::string& functionName,
		    std::istringstream& cmdArg, std::ostream& os );

  /*! \brief This method write a graph description on the file named FileName. */
  void writeGraph(const std::string &aFileName);
  void writeCompletionList(std::ostream& os);
};

SOT_CORE_EXPORT extern sot::PoolStorage sotPool;

} // namespace sot

#endif /* #ifndef __SOT_POOL_HH__ */





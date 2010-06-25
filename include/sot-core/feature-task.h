/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-task.h
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


#ifndef __SOT_FEATURE_TASK_HH__
#define __SOT_FEATURE_TASK_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-generic.h>
#include <sot-core/task-abstract.h>
#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotFeatureTask_EXPORTS)
#    define SOTFEATURETASK_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATURETASK_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATURETASK_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTFEATURETASK_EXPORT FeatureTask 
: public FeatureGeneric
{

 public: 
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  TaskAbstract * taskPtr;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

 public:

  /*! \brief Default constructor */
  FeatureTask( const std::string& name );

  /*! \brief Default destructor */
  virtual ~FeatureTask( void ) {}

  /*! \name Methods to trigger computation related to this feature. 
    @{
  */

  /*! \brief Compute the error between the desired value and the value itself. */
  virtual ml::Vector& computeError( ml::Vector& res,int time );

  /*! @} */

  /*! \brief Display the information related to this task implementation. */
  virtual void display( std::ostream& os ) const;

  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );


} ;

} // namespace sot

#endif // #ifndef __SOT_FEATURE_TASK_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

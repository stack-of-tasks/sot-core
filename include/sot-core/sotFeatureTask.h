/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeatureTask.h
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
#include <sot-core/sotFeatureGeneric.h>
#include <sot-core/sotTaskAbstract.h>
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


class SOTFEATURETASK_EXPORT sotFeatureTask 
: public sotFeatureGeneric
{

 public: 
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  sotTaskAbstract * taskPtr;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

 public:

  /*! \brief Default constructor */
  sotFeatureTask( const std::string& name );

  /*! \brief Default destructor */
  virtual ~sotFeatureTask( void ) {}

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



#endif // #ifndef __SOT_FEATURE_TASK_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

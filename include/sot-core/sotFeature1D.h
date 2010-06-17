/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeature1D.h
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


#ifndef __SOT_FEATURE_1D_HH__
#define __SOT_FEATURE_1D_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/sotFeatureAbstract.h>
#include <sot-core/sotExceptionTask.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotFeature1D_EXPORTS)
#    define SOTFEATURE1D_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATURE1D_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATURE1D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/*!
  \class sotFeature1D
  \brief Simple test: the task is defined to be e_2 = .5 . e'.e, with
  e the mother task. The jacobian is then J_2 = e'.J, J being the jacobian
  of the mother task.
  
*/
class SOTFEATURE1D_EXPORT sotFeature1D 
: public sotFeatureAbstract
{

 public: 
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  /*! \name Signals 
    @{
  */
  /*! \name Input signals 
    @{
   */
  /*! \brief Input for the error. */
  SignalPtr< ml::Vector,int > errorSIN;

  /*! \brief Input for the Jacobian. */
  SignalPtr< ml::Matrix,int > jacobianSIN;

  /*! \brief Input for the activation. */
  SignalPtr< ml::Vector,int > activationSIN;
  /*! @} */
  
  /*! \name Output signals 
    @{
  */
  /*! \brief Publish the jacobian of the feature according to the robot state. */
  using sotFeatureAbstract::jacobianSOUT;

  /*! \brief Publish the error between the desired and the current value of the feature. */
  using sotFeatureAbstract::errorSOUT;

  /*! \brief Publish the activation of this feature. */
  using sotFeatureAbstract::activationSOUT;

 public:

  /*! \brief Default constructor */
  sotFeature1D( const std::string& name );

  /*! \brief Default destructor */
  virtual ~sotFeature1D( void ) {}

  /*! \brief Get the dimension of the feature. */
  virtual unsigned int& getDimension( unsigned int & dim, int time );

  /*! \name Methods to trigger computation related to this feature. 
    @{
  */

  /*! \brief Compute the error between the desired value and the value itself. */
  virtual ml::Vector& computeError( ml::Vector& res,int time );

  /*! \brief Compute the Jacobian of the value according to the robot state.. */
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

  /*! \brief Compute the activation according to the time */
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );
  /*! @} */

  /*! \brief Display the information related to this 1D implementation. */
  virtual void display( std::ostream& os ) const;

/*   void commandLine( const std::string& cmdLine, */
/* 		    std::istringstream& cmdArgs, */
/* 		    std::ostream& os ); */


} ;



#endif // #ifndef __SOT_FEATURE_1D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

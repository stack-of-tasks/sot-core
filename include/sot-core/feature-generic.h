/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-generic.h
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


#ifndef __SOT_FEATURE_GENERIC_HH__
#define __SOT_FEATURE_GENERIC_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotFeatureGeneric_EXPORTS)
#    define SOTFEATUREGENERIC_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATUREGENERIC_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATUREGENERIC_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureGeneric
  \brief Class that defines a generic implementation of the abstract interface for features. 
  
  This class is very useful if the feature can be easily computed using
  the basic operator provided. For instance a free space controller on a end-effector
  is basically directly computed from the Jacobian provided by dyn and some appropriate
  addition and soustraction.
  Instead of building a specific feature for this, it is possible to use the signals
  and plug the computed error, Jacobian and activation to the input of this 
  generic feature implementation.
  
*/
class SOTFEATUREGENERIC_EXPORT FeatureGeneric 
: public FeatureAbstract
{

 public: 
  /*! Field storing the class name. */
  static const std::string CLASS_NAME;
  /*! Returns the name of the class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  unsigned int dimensionDefault;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  /*! \name dg::Signals
    @{
  */
  /*! \name Input signals 
    @{
   */
  /*! \brief Input for the error. */
  dg::SignalPtr< ml::Vector,int > errorSIN;

  /*! \brief Input for the errordot. */
  dg::SignalPtr< ml::Vector,int > errordotSIN;

  /*! \brief Input for the Jacobian. */
  dg::SignalPtr< ml::Matrix,int > jacobianSIN;

  /*! \brief Input for the activation. */
  dg::SignalPtr< ml::Vector,int > activationSIN;
  /*! @} */
  
  /*! \name Output signals 
    @{
  */
  /*! \brief Publish the jacobian of the feature according to the robot state. */
  using FeatureAbstract::jacobianSOUT;

  /*! \brief Publish the error between the desired and the current value of the feature. */
  using FeatureAbstract::errorSOUT;

  /*! \brief Publish the activation of this feature. */
  using FeatureAbstract::activationSOUT;

  /*! \brief New signal the errordot. */
  dg::SignalTimeDependent< ml::Vector,int > errordotSOUT;

 public:

  /*! \brief Default constructor */
  FeatureGeneric( const std::string& name );

  /*! \brief Default destructor */
  virtual ~FeatureGeneric( void ) {}

  /*! \brief Get the dimension of the feature. */
  virtual unsigned int& getDimension( unsigned int & dim, int time );

  /*! \name Methods to trigger computation related to this feature. 
    @{
  */

  /*! \brief Compute the error between the desired value and the value itself. */
  virtual ml::Vector& computeError( ml::Vector& res,int time );

  /*! \brief Compute the desired doterror of the desired value. */
  virtual ml::Vector& computeErrorDot( ml::Vector& res,int time );

  /*! \brief Compute the Jacobian of the value according to the robot state.. */
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );

  /*! \brief Compute the activation according to the time */
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );
  /*! @} */

  /*! \brief Display the information related to this generic implementation. */
  virtual void display( std::ostream& os ) const;

  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );


} ;

} // namespace sot


#endif // #ifndef __SOT_FEATURE_GENERIC_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

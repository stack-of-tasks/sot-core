/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-visual-point.h
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


#ifndef __SOT_FEATURE_VISUALPOINT_HH__
#define __SOT_FEATURE_VISUALPOINT_HH__

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
#  if defined (sotFeatureVisualPoint_EXPORTS)
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATUREVISUALPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/*!
  \class sotFeatureVisualPoint
  \brief Class that defines 2D visualPoint visual feature
*/
class SOTFEATUREVISUALPOINT_EXPORT sotFeatureVisualPoint 
: public sotFeatureAbstract
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  ml::Matrix L;

  

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  SignalPtr< ml::Vector,int > xySIN;
  /** FeatureVisualPoint depth (required to compute the interaction matrix)
   * default Z = 1m. */
  SignalPtr< double,int > ZSIN;
  SignalPtr< ml::Matrix,int > articularJacobianSIN;

  using sotFeatureAbstract::desiredValueSIN;
  using sotFeatureAbstract::selectionSIN;

  using sotFeatureAbstract::jacobianSOUT;
  using sotFeatureAbstract::errorSOUT;
  using sotFeatureAbstract::activationSOUT;

 public:
  sotFeatureVisualPoint( const std::string& name );
  virtual ~sotFeatureVisualPoint( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );
  
  virtual ml::Vector& computeError( ml::Vector& res,int time ); 
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ); 
  virtual ml::Vector& computeActivation( ml::Vector& res,int time ); 

  /** Static Feature selection. */
  inline static sotFlags selectX( void ) { return FLAG_LINE_1; }
  inline static sotFlags selectY( void ) { return FLAG_LINE_2; }

  virtual void display( std::ostream& os ) const;


} ;

} // namespace sot

#endif // #ifndef __SOT_FEATURE_VISUALPOINT_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

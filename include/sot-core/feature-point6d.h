/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-point6d.h
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


#ifndef __SOT_FEATURE_POINT6D_HH__
#define __SOT_FEATURE_POINT6D_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>
#include <sot-core/matrix-homogeneous.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotFeaturePoint6d_EXPORTS)
#    define SOTFEATUREPOINT6D_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATUREPOINT6D_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATUREPOINT6D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/*!
  \class sotFeaturePoint6d
  \brief Class that defines point-3d control feature
*/
class SOTFEATUREPOINT6D_EXPORT sotFeaturePoint6d 
: public sotFeatureAbstract
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  enum ComputationFrameType 
    {
      FRAME_DESIRED
      ,FRAME_CURRENT 
    };
  static const ComputationFrameType COMPUTATION_FRAME_DEFAULT;
  ComputationFrameType computationFrame;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  SignalPtr< sotMatrixHomogeneous,int > positionSIN;
  SignalPtr< ml::Matrix,int > articularJacobianSIN;

  using sotFeatureAbstract::desiredValueSIN;
  using sotFeatureAbstract::selectionSIN;

  using sotFeatureAbstract::jacobianSOUT;
  using sotFeatureAbstract::errorSOUT;
  using sotFeatureAbstract::activationSOUT;

 public:
  sotFeaturePoint6d( const std::string& name );
  virtual ~sotFeaturePoint6d( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );
  
  virtual ml::Vector& computeError( ml::Vector& res,int time ); 
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ); 
  virtual ml::Vector& computeActivation( ml::Vector& res,int time ); 

  /** Static Feature selection. */
  inline static sotFlags selectX( void )  { return FLAG_LINE_1; }
  inline static sotFlags selectY( void )  { return FLAG_LINE_2; }
  inline static sotFlags selectZ( void )  { return FLAG_LINE_3; }
  inline static sotFlags selectRX( void ) { return FLAG_LINE_4; }
  inline static sotFlags selectRY( void ) { return FLAG_LINE_5; }
  inline static sotFlags selectRZ( void ) { return FLAG_LINE_6; }

  inline static sotFlags selectTranslation( void ) { return sotFlags(7); }
  inline static sotFlags selectRotation( void ) { return sotFlags(56); }

  virtual void display( std::ostream& os ) const;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

} ;

} // namespace sot

#endif // #ifndef __SOT_FEATURE_POINT6D_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

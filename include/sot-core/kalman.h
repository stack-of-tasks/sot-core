
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Kalman.h
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

#ifndef __SOT_KALMAN_H
#define __SOT_KALMAN_H

/* -------------------------------------------------------------------------- */
/* --- INCLUDE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot-core/constraint.h>
#include <sot-core/sot-core-api.h>

/* -------------------------------------------------------------------------- */
/* --- CLASSE --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

namespace sot {

namespace dg = dynamicgraph;

class SOT_CORE_EXPORT Kalman
:public dg::Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
  

protected:
  unsigned int size_state ;
  unsigned int size_measure ;
  double dt;

 public: 

  dg::SignalPtr< ml::Vector,int > measureSIN;         // Y=X_mes=H.X
  dg::SignalPtr< ml::Matrix,int > modelTransitionSIN; // A
  dg::SignalPtr< ml::Matrix,int > modelMeasureSIN;    // H
  dg::SignalPtr< ml::Matrix,int > noiseTransitionSIN; // Q
  dg::SignalPtr< ml::Matrix,int > noiseMeasureSIN;    // R
  dg::SignalPtr< ml::Matrix,int > modelControlSIN;    // B
  dg::SignalPtr< ml::Vector,int > controlSIN;         // u

  dg::SignalPtr< ml::Vector,int > statePrecSIN;       // X(t=0)
  dg::SignalPtr< ml::Matrix,int > variancePrecSIN;    // P(t=0)

  dg::SignalTimeDependent< ml::Vector,int > statePredictedSOUT; // Xpre
  dg::SignalTimeDependent< ml::Matrix,int > variancePredictedSOUT; // Ppre
  dg::SignalTimeDependent< ml::Vector,int > stateUpdatedSOUT;  // Xest
/*   dg::SignalTimeDependent< ml::Matrix,int > varianceUpdatedSOUT;   // Pest */

/*   dg::SignalTimeDependent< ml::Matrix,int > gainSINTERN;         // K */
/*   dg::SignalTimeDependent< ml::Matrix,int > innovationSINTERN;   // S */


public:
  /*!  \f${\bf x}_{k \mid k} \f$ valeur estime de l'etat  
   * \f${\bf x}_{k \mid k} =  {\bf x}_{k \mid k-1} + {\bf W}_k
   * \left[ {\bf z}_k -  {\bf H x}_{k \mid k-1} \right]\f$
   */
  //ml::Vector Xest ;

  /*! \f${\bf x}_{k \mid k-1} \f$ : valeur predite  
   * de l'etat \f$ {{\bf x}}_{k|k-1}  =  {\bf A}_{k-1} 
   * {\bf x}_{k-1\mid k-1}\f$.
   */
  //ml::Vector Xpre ;

protected:
  /*! \f${\bf P}_{k \mid k-1} \f$ : matrice de
   * covariance de l'erreur de prediction.
   * \f$ {\bf A}_{k-1}  {\bf P}_{k-1 \mid k-1} 
   * {\bf A}^T_{k-1}   + {\bf Q}_k\f$. 
   */
  //ml::Matrix Ppre ;

  /*!  \f${\bf P}_{k \mid k}\f$ matrice de covariance 
   * de l'erreur d'estimation.
   * \f${\bf P}_{k \mid k} = \left({\bf I - W}_k {\bf H} \right)  
   * {\bf P}_{k \mid  k-1}\f$
   */
  ml::Matrix Pest ;

  /*! \f${\bf W}_k\f$ : Gain du filtre de Kalman
   * \f$ {\bf W}_k = {\bf P}_{k \mid k-1} {\bf H}^T 
   * \left[  {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf R}_k \right]^{-1}\f$ 
   */
  ml::Matrix W ;

  /*! \f$ \bf I\f$ : matrice identite. */
  //ml::Matrix Ident;

public:
  //! matrice d�crivant le modele d'evolution de l'etat
  //ml::Matrix A ;
  //! matrice d�crivant le modele d'evolution de la mesure
  //ml::Matrix H ;
  //! Variance du bruits sur le modele de mesure
  //ml::Matrix R ;
  //! Variance du bruits sur le modele d'etat
  //ml::Matrix Q ;
  //! Command: X_+1 = A.X + Bu
  //ml::Matrix B;
  //ml::Vector u;


public:
  Kalman( const std::string & name ) ;
  void  Init( int _size_state,int _size_measure );

  //double Prediction( const ml::Matrix &A ) ;
  //double Prediction( const ml::Matrix &A,const ml::Vector&u ) ;
  //double Prediction( void ) ;
  double Prediction(int n_sensor) ;
  //double Filtering(ml::Vector &Xmes) ;
  double Filtering(int n_sensor, ml::Vector *Xmes) ;

  ml::Vector& predict( ml::Vector& Xpre,const int& time );
  ml::Vector& filter( ml::Vector& Xest,const int& time );
  ml::Matrix & computeVariancePredicted( ml::Matrix& Ppre,const int& time ); 
/*   ml::Matrix & computeVarianceUpdated( ml::Matrix& Pest,const int& time ); */

  void reset( void );

  /* --- Initialization --- */
  void initFilterCteAcceleration( const double &dt, 
				  const ml::Vector &Z0, 
 				  const ml::Vector &Z1, 
 				  const ml::Vector &Z2, 
 				  const ml::Vector &sigma_noise, 
				  const ml::Vector &sigma_state ) ; 
  void initFilterCteVelocity( const double &dt, 
			      const ml::Vector &Z0, 
			      const ml::Vector &Z1, 
			      const ml::Vector &sigma_noise, 
			      const ml::Vector &sigma_state );
/*   void initFilterSinger( const double &dt, */
/* 			 const double &a, */
/* 			 const ml::Vector &Z0, */
/* 			 const ml::Vector &Z1, */
/* 			 const ml::Vector  &sigma_noise, */
/* 			 const ml::Vector &sigma_state) ; */
/*   void initFilterCteVelocityRobot( const double& dt,  */
/* 				   const ml::Vector &Z0,  */
/* 				   const ml::Vector &Z1,  */
/* 				   const ml::Vector &sigma_noise,  */
/* 				   const ml::Vector &sigma_state ); */
/*   void initFilterCteAccelerationRobot( const double &dt, */
/* 				       const ml::Vector &Z0, */
/* 				       const ml::Vector &Z1, */
/* 				       const ml::Vector &Z2, */
/* 				       const ml::Vector &sigma_noise, */
/* 				       const ml::Vector &sigma_state ) ; */

  static ml::Matrix& A_constantAcc( const int& sizeMeasure,
				    const double& dt,
				    ml::Matrix & A );
  
  static  ml::Matrix& A_constantSpeed( const int& sizeMeasure,
				       const double& dt,
				       ml::Matrix & A );
  
  static  ml::Matrix& A_constantSpeedRobot( const int& sizeMeasure,
					    const double& dt,
					    ml::Matrix & A );
  
  /* --- Entity --- */
  void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
    
} ;


} // namespace sot



/*!
  \file Kalman.h
  \brief  Generic kalman filtering implementation
*/

#endif

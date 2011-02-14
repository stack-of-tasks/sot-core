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

/* --- SOT --- */
#include <sot-core/kalman.h>          /* Header of the class implemented here.   */
#include <sot-core/debug.h>
#include <sot-core/exception-tools.h>
#include <sot-core/factory.h>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Kalman,"Kalman");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


void
Kalman::Init(int _size_state, int _size_measure)
{
  size_state = _size_state;
  size_measure = _size_measure ;
//   A.resize(size_state, size_state) ;
//   H.resize(size_measure,  size_state) ;

//   R.resize(size_measure, size_measure) ;
//   Q.resize(size_state, size_state) ;

//   Xest.resize(size_state) ; Xest.fill(0.);
//   Xpre.resize(size_state) ; Xpre.fill(0.) ;

  Pest.resize(size_state, size_state) ; Pest.fill(.0) ;

  //Ident.resize(size_state, size_state) ;
  //dt = -1 ;


//   B.resize(size_state,_size_measure);
//   u.resize(_size_measure);

}

Kalman::Kalman( const std::string& name )
  : Entity(name)
    ,measureSIN( NULL,"Kalman("+name+")::input(vector)::Ymes" )
    ,modelTransitionSIN( NULL,"Kalman("+name+")::input(matrix)::A" )
    ,modelMeasureSIN( NULL,"Kalman("+name+")::input(matrix)::H" )
    ,noiseTransitionSIN( NULL,"Kalman("+name+")::input(matrix)::Q" )
    ,noiseMeasureSIN( NULL,"Kalman("+name+")::input(matrix)::R" )
    ,modelControlSIN( NULL,"Kalman("+name+")::input(matrix)::B" )
    ,controlSIN( NULL,"Kalman("+name+")::input(vector)::u" )

    ,statePrecSIN( NULL,"Kalman("+name+")::input(vector)::X0" )
    ,variancePrecSIN( NULL,"Kalman("+name+")::input(matrix)::P0" )

    ,statePredictedSOUT( boost::bind(&Kalman::predict,this,_1,_2),
			 modelTransitionSIN<<statePrecSIN
			 <<modelControlSIN<<controlSIN,
			 "Kalman("+name+")::output(vector)::Xpre" )
    ,variancePredictedSOUT( boost::bind(&Kalman::computeVariancePredicted,this,_1,_2),
			    variancePrecSIN<<noiseTransitionSIN<<modelTransitionSIN,
			    "Kalman("+name+")::output(vector)::Ppre" )
    ,stateUpdatedSOUT( boost::bind(&Kalman::filter,this,_1,_2),
		       statePredictedSOUT<<measureSIN
		       <<modelMeasureSIN<<noiseMeasureSIN
		       <<variancePredictedSOUT,
		       "Kalman("+name+")::output(vector)::Xest" )
  
{
  signalRegistration( measureSIN << modelTransitionSIN 
		      << modelMeasureSIN << noiseTransitionSIN 
		      << noiseMeasureSIN << modelControlSIN 
		      << controlSIN << statePrecSIN << variancePrecSIN
		      << statePredictedSOUT << stateUpdatedSOUT 
		      << variancePredictedSOUT );

  //Init( size_state,size_measure) ;
}

/* -------------------------------------------------------------------------- */
/* --- PREDICTION ----------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

// double
// Kalman::Prediction(const ml::Matrix &_A)
// {
//   A=_A;
//   return Prediction();
// }

// double
// Kalman::Prediction(const ml::Matrix &_A,const ml::Vector& _u)
// {
//   A=_A; u=_u;
//   return Prediction();
// }


ml::Matrix & Kalman::
computeVariancePredicted( ml::Matrix& Ppre,const int& time )
{
  sotDEBUGIN(15);

  const ml::Matrix &Q = noiseTransitionSIN( time );
  const ml::Matrix &A = modelTransitionSIN( time );
  const ml::Matrix &Pt_1 = variancePrecSIN( time );

  //const ml::Matrix &Pest = varianceUpdatedSOUT.accessCopy();

  /* Matrice de covariance T de l'erreur de prediction. */
  /* Bar-Shalom  5.2.3.5 
   * Ppre = A.Pt_1.A^T + Q  */
  ml::Matrix AP( A.nbRows(),Pt_1.nbCols() );
  A.multiply(Pt_1,AP);
  AP.multiply( A.transpose(),Ppre );
  Ppre += Q;

  sotDEBUG(15) << "A " <<std::endl << A ;
  sotDEBUG(15) << "Pt_1 " <<std::endl<< Pt_1 ;
  sotDEBUG(15) << "At " <<std::endl<< A.transpose() ;
    
  sotDEBUG(15) << "Ppre " << std::endl << Ppre << std::endl ;
  
  sotDEBUGOUT(15);
  return Ppre;
}

/*!
 * \brief Mise a jour du filtre de Kalman : Etape de prediction
 *
 * �quations de pr�diction
 *   \f[
 *     {{\bf x}}_{k|k-1}   =  {\bf A}_{k-1} {\bf x}_{k-1\mid k-1}
 *   \f]
 *   \f[
 *     {\bf P}_{k \mid k-1}  = {\bf A}_{k-1}  {\bf P}_{k-1 \mid k-1}
 *      {\bf A}^T_{k-1} + {\bf Q}_k
 *   \f]
 */
ml::Vector& Kalman::
predict( ml::Vector& Xpre,const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector &Xest = statePrecSIN( time );
  const ml::Matrix &A = modelTransitionSIN( time );
  
  /* --- Assert --- */
  if( Xest.size()!=size_state )
    {  SOT_THROW ExceptionTools( ExceptionTools::KALMAN_SIZE,
				    "Uncompatible sizes.",
				    "(size(Xest)=%d != size_state=%d",
				    Xest.size(),size_state); }

  /* --- Prediction --- */
  sotDEBUG(15) << "A = " << std::endl << A << std::endl ;
  sotDEBUG(15) << "Xest = "<< std::endl  << Xest  ;
  
  /* Bar-Shalom  5.2.3.2 */
  Xpre = A*Xest;
  
  try {
    const ml::Matrix &B = modelControlSIN( time );
    const ml::Vector &u = controlSIN( time );
    sotDEBUG(25) << "u = " << u;
    sotDEBUG(25) << "B = " <<std::endl<< B;
    
    Xpre += B*u;
  } catch (...) { sotDEBUG(15) << "No control set." <<std::endl; }
  
  sotDEBUG(25) << "Xpre = " << Xpre  ;
  
  sotDEBUGOUT(15);
  return Xpre;
}


/* -------------------------------------------------------------------------- */
/* --- FILTRAGE ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
 * \brief  Mise a jour du filtre de Kalman : Etape de Filtrage
 * 
 * \param Z : nouvelle mesure
 * 
 * �quations de filtrage
 * \f[
 *    {\bf x}_{k \mid k} = {\bf x}_{k \mid k-1} + {\bf W}_k  \left[ {\bf Z}_k -
 *    {\bf H x}_{k \mid k-1} \right]
 * \f]
 * gain du filtre de Kalman
 * \f[
 *   {\bf W_k} = {\bf P}_{k \mid k-1} {\bf H}^T
 *   \left[  {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf R}_k \right]^{-1}
 * \f]
 * \f[
 *    {\bf P}_{k \mid k} = \left({\bf I - K}_k {\bf H} \right)  {\bf P}_{k \mid k-1}
 * \f]
 */
ml::Vector& Kalman::
filter( ml::Vector& Xest,const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector & Xpre = statePredictedSOUT( time );
  const ml::Vector & Z = measureSIN( time );
  const ml::Matrix & H = modelMeasureSIN( time );
  const ml::Matrix & R = noiseMeasureSIN( time );
  const ml::Matrix & Ppre = variancePredictedSOUT( time );

  sotDEBUG(25)<<"H = "<<std::endl<<H;
  sotDEBUG(25)<<"Ppre = "<<std::endl<<Ppre;
  sotDEBUG(25)<<"R = "<<std::endl<<R;

  /* PpreHt = Pre.H^T */
  ml::Matrix PpreHt( Ppre.nbRows(),H.nbRows() );
  Ppre.multiply( H.transpose(),PpreHt );
  sotDEBUG(45)<<"PpreHt = "<<std::endl<<PpreHt;

  /* S is the innovation: S= H.P.H^T + R */
  /* Bar-Shalom  5.2.3.11. */
  ml::Matrix S( R.nbRows(),R.nbCols() );
  H.multiply( PpreHt,S );
  S+=R;
  sotDEBUG(25)<<"S = "<<std::endl<<S;
  sotDEBUG(25)<<"Sinv = "<<std::endl<<S.inverse();
  
  /* W is the gain: W = P.H.S^-1 */
  //W = (Ppre * H.transpose())* S.inverse() ;
  PpreHt.multiply( S.inverse(),W );

  /* Bar-Shalom  5.2.3.15. 
   * Pest = Pre - W.S.W^T */
  // Pest = Ppre - W*S*W.transpose() ;
  W.multiply( S*W.transpose(),Pest );
  Pest *= -1; Pest += Ppre;

  sotDEBUG(15)<<"X = " <<Xpre;
  sotDEBUG(15)<<"Gain = " <<std::endl<< W;
  sotDEBUG(15)<<"Z = " <<Z;
  sotDEBUG(15)<<"HX = " <<(H*Xpre);
  
    /* Bar-Shalom  5.2.3.12 5.2.3.13 5.2.3.7. */
    // {
    //       ml::Matrix WZ_HX = W*( Z-(H*Xpre) );
    //       for( unsigned int i=0;i<Xpre.size();++i )
    // 	{ Xest(i) = Xpre(i) + WZ_HX(i,0); }
    //     }
    
    //Xest = Xpre + (W*(Z - (H*Xpre))).column(1) ;
  
  /* Residual: y = z - Hx */
  ml::Vector y( Z.size() ); 
  y = H*Xpre; y *= -1; y += Z;

  //Xest = Xpre + (W*(Z-(H*Xpre)));
  Xest = W*y;
  Xest += Xpre; 
  
  sotDEBUG(15)<<"Xest = " <<Xest;
    
  statePrecSIN = Xest;
  variancePrecSIN = Pest;

  sotDEBUGOUT(15);
  return Xest;

}

/* -------------------------------------------------------------------------- */
/* --- MODELE --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

void Kalman::
reset( void )
{
//   const ml::Vector & X0 = stateInitSIN.accessCopy();
//   const ml::Matrix & P0 = varianceInitSIN.accessCopy();
   
//   stateUpdatedSOUT.setConstant(X0);
//   stateUpdatedSOUT.setFunction( boost::bind(&Kalman::filter,this,_1,_2) );

//   Pest = P0;
}


ml::Matrix& Kalman::
A_constantSpeed( const int & sizeMeasure,
		 const double & dt,
		 ml::Matrix & A )
{
  A.resize( 2*sizeMeasure,2*sizeMeasure );
  
  for (int i=0;  i < sizeMeasure ;  i++ )
  {
    /* modele sur l'etat. */

    /*         | 1  dt  dt2/2 |
     *     A = | 0   1   dt   |
     *         | 0   0    1   |  */
    A(i,i) = 1 ;
    A(i,i+sizeMeasure) = dt ;
    A(i+sizeMeasure,i+sizeMeasure) = 1 ;
  }

  return A;
}

ml::Matrix& Kalman::
A_constantSpeedRobot( const int & sizeMeasure,
		      const double & dt,
		      ml::Matrix & A )
{
  A.resize( 3*sizeMeasure,3*sizeMeasure );

  for (int i=0;  i < sizeMeasure ;  i++ )
  {
    A(i,i) = 1 ;
    A(i,i+sizeMeasure) = dt ;
    A(i+sizeMeasure,i+sizeMeasure) = 1 ;
  }

  return A;
}

ml::Matrix& Kalman::
A_constantAcc( const int & sizeMeasure,
	       const double & dt,
	       ml::Matrix & A )
{
  const double dt2=dt*dt/2.;

  for (int i=0;  i < sizeMeasure ;  i++ )
  {
    /* modele sur l'etat. */

    /*         | 1  dt  dt2/2 |
     *     A = | 0   1   dt   |
     *         | 0   0    1   |  */
    A(i,i) = 1;
    A(i,i+sizeMeasure) = dt;
    A(i,i+2*sizeMeasure) = dt2;
    
    A(i+sizeMeasure,i+sizeMeasure) = 1;
    A(i+sizeMeasure,i+2*sizeMeasure) = dt;
    
    A(i+2*sizeMeasure,i+2*sizeMeasure) = dt2;
  }

  return A;
}

/*!
 * \brief Filter initialization for a constant velocity model
 *
 * \param dt : time between two measures
 * \param Vn : Variance of measure noise
 * \param Vw : Variance of the state noise
 * 
 * mod�le d'�tat
 *   \f[ {\bf x} = \left[\; y  \quad \frac{\partial y}{\partial t}
 *    \;\right]^T = \left[\;  y  \quad \dot{ y}\;\right]^T
 *   \f]
 * 
 * Mod�lisation des filtres
 * \f[
 *   \begin{array}{rcl} \\
 *     {\bf x}(t+1) &= &  {\bf Ax} (t) + {\bf Q} \\
 *     {\bf z}(t) &=& {\bf H x}(t) + {\bf R}
 *   \std::end{array}
 * \f]
 *
 *
 * La matrice A d�crit le mod�le d'�volution de l'�tat. Dans 
 * le cas pr�sent elle est donn�e par :
 *   \f[
 *     {\bf A}= \left( \begin{array}{cc} 1 & \Delta t  \\
 *     0 & 1 \std::end{array} \right)
 *   \f]
 *
 * Le bruit \f${\bf Q} = \left( \begin{array}{c} \;q_1 \quad q_2\; 
 *    \std::end{array} \right)^T\f$
 * vient mod�liser les variations sur le mod�le � vitesse constante 
 * (dues aux acc�l�rations)
 *
 * En effet on a:
 *   \f[
 *     \left\{
 *     \begin{array}{rcl}
 *        y(t+1)& =& y(t) + \Delta(t) \dot y(t) +
 *        \underbrace{\frac{\Delta t^2}{2} \ddot y(t)}_{q_1} \\
 *        \dot y(t+1) &=& \dot y(t) + \underbrace{\Delta(t) 
 *        \ddot y(t)}_{q_2}
 *     \std::end{array}
 *     \right.
 *   \f]
 * et donc
 *   \f[
 *      \left\{
 *      \begin{array}{rcccccc}
 *         y(t+1)& =& y(t) &+& \Delta(t) \dot y(t) &+& {q_1} \\
 *       \dot y(t+1) &=& & &\dot y(t) &+& {q_2}
 *     \std::end{array}
 *     \right. \qquad \Leftrightarrow  
 *     \qquad {\bf x}(t+1) = {\bf A x}(t) + Q
 *   \f]
 */
void Kalman::
initFilterCteVelocity( const double& dt, 
		       const ml::Vector &Z0, 
		       const ml::Vector &Z1, 
		       const ml::Vector &sigma_noise,   // .01
		       const ml::Vector &sigma_state )  // 5
{
  sotDEBUGIN(15);
  sotDEBUG(25) << "Z0 = " << Z0  << std::endl;
  sotDEBUG(35) << "Z1 = " << Z1  << std::endl;
  sotDEBUG(35) << "sR = " << sigma_noise  << std::endl;
  sotDEBUG(35) << "sQ = " << sigma_state  << std::endl;

  double dt2 = dt*dt ;
  double dt3 = dt2*dt ;

  ml::Matrix A,H,B,R,Q,P0;
  ml::Vector X0;
  
  size_measure = Z0.size();
  size_state = Z0.size()*2; // state = [pose;velocity] 

  A.resize(size_state,size_state);    A.fill(0.);
  H.resize(size_measure, size_state); H.fill(0.);
  R.resize(size_measure,size_measure);R.fill(0.);
  Q.resize(size_state,size_state);    Q.fill(0.);
  B.resize(size_state,size_measure);  B.fill(0.);
  X0.resize(size_state);              X0.fill(0.);
  P0.resize(size_state,size_state) ;  P0.fill(.0) ;
  //Ident.resize(size_state,size_state) ;   Ident.setIdentiy();

  /* Initialize the matrices that described the models. */
  for( unsigned int i=0;i<size_measure;++i )
  {
    /* modele sur l'etat. 
     *         | 1  dt |
     *     A = |       |
     *         | 0   1 |  */
    A_constantSpeed( size_measure,dt,A );

    /* modele sur la mesure. */
    H(i,i) = 1;

    /* Commande */
    B(i,i) = 1;

    double sR = sigma_noise(i) ;
    double sQ = sigma_state(i) ;

    /* bruit de mesure. */
    R(i,i) = sR ;

    /* bruit d'etat. */
    Q(i,i) = sQ*dt3/3;
    Q(i,i+size_measure) = sQ*dt2/2;
    Q(i+size_measure,i) = sQ *dt2/2;
    Q(i+size_measure,i+size_measure) = sQ*dt;

    /* T(1|1). */
    P0(i,i)     = sR ;
    P0(i,i+size_measure)     =  sR/(2*dt) ;
    P0(i+size_measure,i)     =    sR/(2*dt) ;
    P0(i+size_measure,i+size_measure) =  sQ*2*dt/3.0+ sR/(2*dt2) ;


    X0(i) = Z1(i) ;
    X0(i+size_measure) = (Z1(i) - Z0(i))/dt ; ;
  }

  modelTransitionSIN = A;
  modelMeasureSIN = H;
  noiseMeasureSIN = R;
  noiseTransitionSIN = Q;
  modelControlSIN = B;
  statePrecSIN = X0;
  variancePrecSIN = P0;

  sotDEBUG(25) <<"A = "<<std::endl<< A;

  sotDEBUGOUT(15);
  return ;
}



/*!
 * \brief Filter initialization for a constant velocity model
 *
 * \param dt : time between two measures
 * \param Vn : Variance of measure noise
 * \param Vw : Variance of the state noise
 *
 * mod�le d'�tat
 *   \f[ S = \left[\; y  \quad \frac{\partial y}{\partial t}\;\right]^T 
 *    = \left[\;  y  \quad \dot{ y}\;\right]^T
 *   \f]
 *
 * Mod�lisation des filtres
 * \f[
 * \begin{array}{rclll} \\
 * S(t+1) &= & A S(t) + W(t)&~~~~~~~~~~~& S(t+1) \mbox{ est un vecteur} 
 *  \left[\;
 *     y\quad \dot y\;\right]^T \\
 *     X(t) &=& C S(t) + N(t)&& X(t) \mbox{ est un scalaire}
 *   \std::end{array}
 * \f]
 *
 * La matrice A d�crit le mod�le d'�volution de l'�tat. Dans le cas
 *  pr�sent elle est donn�e par :
 * \f[
 * A= \left( \begin{array}{cc} 1 & \Delta t  \\ 0 & 1 \std::end{array} \right)
 * \f]
 *
 * Le bruit 
 *   \f$W = \left( \begin{array}{c} \;W_1 \quad W_2\;
 *   \std::end{array} \right)^T\f$
 * vient mod�liser les variations sur le mod�le � vitesse
 * constante (dues aux acc�l�rations)
 * 
 * En effet on a:
 * \f[
 * \left\{
 * \begin{array}{rcl}
 *     y(t+1)& =& y(t) + \Delta(t) \dot y(t) +
 *     \underbrace{\frac{\Delta t^2}{2} \ddot y(t)}_{W_1} \\
 *     \dot y(t+1) &=& \dot y(t) + \underbrace{\Delta(t)
 *     \ddot y(t)}_{W_2}
 * \std::end{array} \right.
 * \f]
 * et donc
 * \f[
 * \left\{
 * \begin{array}{rcccccc}
 * y(t+1)& =& y(t) &+& \Delta(t) \dot y(t) &+& {W_1} \\
 * \dot y(t+1) &=& & &\dot y(t) &+& {W_2}
 * \std::end{array}
 * \right. \qquad \Leftrightarrow  \qquad S(t+1) = A S(t) + W
 * \f]
 */
void Kalman::
initFilterCteAcceleration( const double & dt,
			   const ml::Vector &Z0,
			   const ml::Vector &Z1,
			   const ml::Vector &Z2,
			   const ml::Vector &sigma_noise,
			   const ml::Vector &sigma_state )
{
  sotDEBUGF(15,"# In {");

  double dt2 = dt*dt ;
  double dt3 = dt2*dt ;
  double dt4 = dt3*dt ;
  double dt5 = dt4*dt ;
  
  ml::Matrix A,H,B,R,Q,P0;
  ml::Vector X0;
 
  size_measure = Z0.size();
  size_state = Z0.size()*3; // state = [pose;velocity;acc] 

  A.resize(size_state,size_state);    A.fill(0.);
  H.resize(size_measure, size_state); H.fill(0.);
  R.resize(size_measure,size_measure);R.fill(0.);
  Q.resize(size_state,size_state);    Q.fill(0.);
  B.resize(size_state,size_measure);  B.fill(0.);
  X0.resize(size_state);              X0.fill(0.);
  P0.resize(size_state,size_state) ;  P0.fill(.0) ;
 
  /* initialise les matrices decrivant les modeles. */
  for( unsigned int i=0;i<size_measure;++i )
  {
    /* modele sur l'etat. */
    A_constantAcc( size_measure,dt,A );

    /* modele sur la mesure. */
    H(i,i) = 1;

    /* Commande */
    B(i,i) = 1;

    double sR = sigma_noise(i) ;
    double sQ = sigma_state(i) ;

    /* bruit de mesure. */
    R(i,i) = sR ;

    /* bruit d'etat. */
    Q(i  ,i  ) =  sQ * dt5/20;
    Q(i  ,i+size_measure) =  sQ * dt4/8;
    Q(i  ,i+2*size_measure) =  sQ * dt3/6 ;

    Q(i+size_measure,i  ) = sQ * dt4/8 ;
    Q(i+size_measure,i+size_measure) = sQ * dt3/3 ;
    Q(i+size_measure,i+2*size_measure) = sQ * dt2/2 ;

    Q(i+2*size_measure,i  ) = sQ * dt3/6 ;
    Q(i+2*size_measure,i+size_measure) = sQ * dt2/2.0 ;
    Q(i+2*size_measure,i+2*size_measure) = sQ * dt ;


    /* Initialisation pour la matrice de covariance sur l'etat. */
    P0(i  ,i  ) = sR ;
    P0(i  ,i+size_measure) = 1.5/dt*sR ;
    P0(i  ,i+2*size_measure) = sR/(dt2) ;

    P0(i+size_measure,i  ) = 1.5/dt*sR ;
    P0(i+size_measure,i+size_measure) = dt3/3*sQ + 13/(2*dt2)*sR ;
    P0(i+size_measure,i+2*size_measure) = 9*dt2*sQ/40.0 +6/dt3*sR ;

    P0(i+2*size_measure,i  ) = sR/(dt2) ;
    P0(i+2*size_measure,i+size_measure) = 9*dt2*sQ/40.0 +6/dt3*sR ;
    P0(i+2*size_measure,i+2*size_measure) = 23*dt/30.0*sQ+6.0/dt4*sR ;


    /* Initialisation pour l'etat. */
    X0(i) = Z2(i) ;
    X0(i+size_measure) = ( 1.5 *Z2(i) - Z1(i) -0.5*Z0(i) ) /( 2*dt ) ;
    X0(i+2*size_measure) = ( Z2(i) - 2*Z1(i) + Z0(i) ) /( dt*dt ) ;
    
  }

  modelTransitionSIN = A;
  modelMeasureSIN = H;
  noiseMeasureSIN = R;
  noiseTransitionSIN = Q;
  modelControlSIN = B;
  statePrecSIN = X0;
  variancePrecSIN = P0;

  sotDEBUG(25) <<"A = "<<std::endl<< A;

  sotDEBUGF(15,"# Out }");
  return;
}

// void Kalman::
// initFilterSinger( const double& _dt,
// 		  const double& a,
// 		  const ml::Vector &Z0,
// 		  const ml::Vector &Z1,
// 		  const ml::Vector  &sigma_noise,
// 		  const ml::Vector &sigma_state )
// {
//   sotDEBUGF(15,"# In {");

//   dt = _dt ;

//   double dt2 = dt*dt ;
//   double dt3 = dt2*dt ;

//   const double a2 = a*a ;
//   const double a3 = a2*a ;
//   const double a4 = a3*a ;

//   Pest.fill(0.) ;
//   /* initialise les matrices decrivant les modeles. */
//   for( unsigned int i=0;i<size_measure;++i )
//   {
//     /* modele sur l'etat. */

//     /*         | 1  dt  1/a^2*(1+a.dt+e^(-a.dt) |
//      *     A = | 0   1      1/a.(1-e^(-a.dt)    |
//      *         | 0   0         e^(-a.dt)        |  */

//     A(3*i,3*i) = 1 ;
//     A(3*i,3*i+1) = dt ;
//     A(3*i,3*i+2) = 1/a2*(1+a*dt+exp(-a*dt)) ;
//     A(3*i+1,3*i+1) = 1 ;
//     A(3*i+1,3*i+2) = 1/a*(1-exp(-a*dt)) ;
//     A(3*i+2,3*i+2) = exp(-a*dt) ;


//     /* modele sur la mesure. */
//     H(i,3*i) = 1 ;
//     H(i,3*i+1) = 0 ;
//     H(i,3*i+2) = 0 ;

//     double sR = sigma_noise(i) ;
//     double sQ = sigma_state(i) ;

//     R(i,i) = (sR) ; /* bruit de mesure 1.5mm. */

//     Q(3*i  ,3*i  ) = sQ/a4*(1-exp(-2*a*dt)
// 			     +2*a*dt+2*a3/3*dt3-2*a2*dt2
// 			     -4*a*dt*exp(-a*dt) ) ;
//     Q(3*i  ,3*i+1) = sQ/a3*(1+exp(-2*a*dt)-2*exp(-a*dt)
// 			     +2*a*dt*exp(-a*dt)-2*a*dt+a2*dt2 ) ;
//     Q(3*i  ,3*i+2) = sQ/a2*(1-exp(-2*a*dt)-2*a*dt*exp(-a*dt) ) ;

//     Q(3*i+1,3*i  ) = Q(3*i  ,3*i+1) ;
//     Q(3*i+1,3*i+1) = sQ/a2*(4*exp(-a*dt)-3-exp(-2*a*dt)+2*a*dt ) ;
//     Q(3*i+1,3*i+2) = sQ/a*(exp(-2*a*dt)+1- 2*exp(-a*dt)) ;

//     Q(3*i+2,3*i  ) = Q(3*i,3*i+2) ;
//     Q(3*i+2,3*i+1) = Q(3*i+1,3*i+2) ;
//     Q(3*i+2,3*i+2) = sQ*(1-exp(-2*a*dt) ) ;


//     /* Initialisation pour la matrice de covariance sur l'etat. */
//     Pest(3*i  ,3*i  ) = sR ;
//     Pest(3*i  ,3*i+1) = 1/dt*sR ;
//     Pest(3*i  ,3*i+2) = 0 ;

//     Pest(3*i+1,3*i  ) = 1/dt*sR ;
//     Pest(3*i+1,3*i+1) = ( 2*sR/dt2 + sQ/(a4*dt2)
// 			   *(2-a2*dt2+2*a3*dt3/3.0 
// 			     -2*exp(-a*dt)-2*a*dt*exp(-a*dt)) );
//     Pest(3*i+1,3*i+2) = sQ/(a2*dt)*(exp(-a*dt)+a*dt-1)  ;

//     Pest(3*i+2,3*i  ) = 0  ;
//     Pest(3*i+2,3*i+1) =  Pest(3*i+1,3*i+2) ;
//     Pest(3*i+2,3*i+2) = 0  ;


//     /* Initialisation pour l'etat. */

//     Xest(3*i)   = Z1(i) ;
//     Xest(3*i+1) = ( Z1(i) - Z0(i) ) /(dt ) ;
//     Xest(3*i+2) = 0 ;

//   }

//   sotDEBUGOUT(15);
//   return;
// }

// void Kalman::
// initFilterCteVelocityRobot( const double & _dt, 
// 			    const ml::Vector &Z0, 
// 			    const ml::Vector &Z1, 
// 			    const ml::Vector &sigma_noise, 
// 			    const ml::Vector &sigma_state )
// {
//   this->initFilterCteVelocity( _dt,Z0,Z1,sigma_noise,sigma_state );

//   B.fill(.0); u.fill(.0);
//   for( unsigned int i=0;i<size_measure;++i )
//     {
//       B(i,i)=1;
//     }

//   return;
// }


// void Kalman::
// initFilterCteAccelerationRobot( const double & _dt, 
// 				const ml::Vector &Z0, 
// 				const ml::Vector &Z1, 
// 				const ml::Vector &Z2, 
// 				const ml::Vector  &sigma_noise, 
// 				const ml::Vector &sigma_state )
// {
//   this->initFilterCteAcceleration(_dt,Z0,Z1,Z2,sigma_noise,sigma_state);
  
//   B.fill(.0); u.fill(.0);
//   for( unsigned int i=0;i<size_measure;++i )
//     {
//       B(i,i)=1;
//     }

//   return;
// }


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


void Kalman::
display( std::ostream& os ) const
{
  os <<"Kalman <"<<name<<"> ... TODO";

}




void Kalman::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "Kalman:"<<std::endl
	 << " - constantVelocity <noise> [<noise_trans> [<dt>]]:"
	 << " - constantAcceleration <noise> [<noise_trans> [<dt>]]:"
	 << " init the matrices for cst v." << std::endl
	 << " - reset: reset the X,P updated state/var from X0,P0 signals." << std::endl;
    }
  else if( cmdLine == "constantVelocity" )
    {
      const ml::Vector & Z = measureSIN.accessCopy();
      
      double dt=1./200,noise,noiseState;
      cmdArgs >> noise;
      if( cmdArgs.good() ) { cmdArgs >> noiseState; } 
      else { noiseState = noise; }
      if( cmdArgs.good() ) { cmdArgs >> dt; } 
      ml::Vector noiseVect( Z.size() ); noiseVect.fill( noise );
      ml::Vector noiseStateVect( Z.size() ); noiseStateVect.fill( noiseState );
      sotDEBUG(15) << "args = " << noise 
		   << " " << noiseState << " " << dt << std::endl;

      initFilterCteVelocity( dt,Z,Z,noiseVect,noiseStateVect );
    }
  else if( cmdLine == "constantAcceleration" )
    {
      const ml::Vector & Z = measureSIN.accessCopy();
      
      double dt=1./200,noise,noiseState;
      cmdArgs >> noise;
      if( cmdArgs.good() ) { cmdArgs >> noiseState; } 
      else { noiseState = noise; }
      if( cmdArgs.good() ) { cmdArgs >> dt; } 
      ml::Vector noiseVect( Z.size() ); noiseVect.fill( noise );
      ml::Vector noiseStateVect( Z.size() ); noiseStateVect.fill( noiseState );
      sotDEBUG(15) << "args = " << noise 
		   << " " << noiseState << " " << dt << std::endl;

      initFilterCteAcceleration( dt,Z,Z,Z,noiseVect,noiseStateVect );
    }
  else if( cmdLine == "reset" ) 
    { reset();  }
  else { Entity::commandLine( cmdLine,cmdArgs,os ); }
}


/*!
  \file Kalman.cpp
  \brief  Generic kalman filtering implementation
*/


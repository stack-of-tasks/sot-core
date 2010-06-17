/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeatureLineDistance.cpp
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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <sot-core/sotDebug.h>
#include <sot-core/sotFeatureLineDistance.h>
#include <sot-core/exception-feature.h>

#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/vector-utheta.h>

using namespace std;

#include <sot-core/factory.h>
SOT_FACTORY_FEATURE_PLUGIN(sotFeatureLineDistance,"FeatureLineDistance");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotFeatureLineDistance::
sotFeatureLineDistance( const string& pointName )
  : sotFeatureAbstract( pointName )
    ,positionSIN( NULL,"sotFeatureLineDistance("+name+")::input(matrixHomo)::position" )
    ,articularJacobianSIN( NULL,"sotFeatureLineDistance("+name+")::input(matrix)::Jq" )
    ,positionRefSIN( NULL,"sotFeatureLineDistance("+name+")::input(vector)::positionRef" )
    ,vectorSIN( NULL,"sotFeatureVector3("+name+")::input(vector3)::vector" )
  ,lineSOUT( boost::bind(&sotFeatureLineDistance::computeLineCoordinates,this,_1,_2),
             positionSIN<<positionRefSIN,
             "sotFeatureAbstract("+name+")::output(vector)::line" )
{
  jacobianSOUT.addDependancy( positionSIN );
  jacobianSOUT.addDependancy( articularJacobianSIN );

  errorSOUT.addDependancy( positionSIN );

  activationSOUT.removeDependancy( desiredValueSIN );

  signalRegistration( positionSIN<<articularJacobianSIN
                      <<positionRefSIN<<lineSOUT<<vectorSIN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& sotFeatureLineDistance::
getDimension( unsigned int & dim, int time )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=1;
}

/* --------------------------------------------------------------------- */
ml::Vector& sotFeatureLineDistance::
computeLineCoordinates( ml::Vector& cood,int time )
{
  sotDEBUGIN(15);

  cood.resize(6);

  /* Line coordinates */
  const sotMatrixHomogeneous &pos = positionSIN(time);
  const ml::Vector & vect = vectorSIN(time);
  sotMatrixRotation R; pos.extract(R);
  ml::Vector v(3); R.multiply(vect,v);

  cood(0)= pos(0,3);
  cood(1)= pos(1,3);
  cood(2)= pos(2,3);
  cood(3)= v(0);
  cood(4)= v(1);
  cood(5)= v(2);

  sotDEBUGOUT(15);
  return cood;
}


/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
ml::Matrix& sotFeatureLineDistance::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  /* --- Compute the jacobian of the line coordinates --- */
  ml::Matrix Jline;
  {
    const ml::Matrix & Jq = articularJacobianSIN(time);

    const ml::Vector & vect = vectorSIN(time);
    const sotMatrixHomogeneous & M = positionSIN(time);
    sotMatrixRotation R; M.extract(R); // wRh

    ml::Matrix Skew(3,3);
    Skew( 0,0 ) = 0        ; Skew( 0,1 )=-vect( 2 );  Skew( 0,2 ) = vect( 1 );
    Skew( 1,0 ) = vect( 2 ); Skew( 1,1 )= 0        ;  Skew( 1,2 ) =-vect( 0 );
    Skew( 2,0 ) =-vect( 1 ); Skew( 2,1 )= vect( 0 );  Skew( 2,2 ) = 0        ;

    ml::Matrix RSk(3,3); R.multiply(Skew,RSk);

    Jline.resize(6,Jq.nbCols());
    for( unsigned int i=0;i<3;++i )
      for( unsigned int j=0;j<Jq.nbCols();++j )
        {
          Jline(i,j)   = 0;
          Jline(i+3,j) = 0;
          for( unsigned int k=0;k<3;++k )
            {
              Jline(i,j)   +=  R(i,k)*Jq(k,j);
              Jline(i+3,j) += -RSk(i,k)*Jq(k+3,j);
            }
        }
  }

  /* --- Compute the jacobian wrt the line coordinates --- */
  const ml::Vector &line = lineSOUT(time);
  const double & x0 = line(0);
  const double & y0 = line(1);
  const double & z0 = line(2);
  const double & a0 = line(3);
  const double & b0 = line(4);
  const double & c0 = line(5);

  const ml::Vector &posRef = positionRefSIN(time);
  const double & x1 = posRef(0);
  const double & y1 = posRef(1);
  const double & z1 = posRef(2);
  const double & a1 = posRef(3);
  const double & b1 = posRef(4);
  const double & c1 = posRef(5);

  /* Differential */
  const double a1_3 = a1*a1*a1;
  const double b1_3 = b1*b1*b1;
  const double c1_3 = c1*c1*c1;

  double K = c0*c0*a1*a1 - 2*c0*a1*a0*c1 - 2*c0*b1*b0*c1 + c0*c0*b1*b1 - 2*b0*a1*a0*b1 + b0*b0*a1*a1 + b0*b0*c1*c1 + a0*a0*b1*b1 + a0*a0*c1*c1;

  const double diffx0 = -b0*c1 + c0*b1;
  const double diffy0 = a0*c1 - c0*a1;
  const double diffz0 = -a0*b1 + b0*a1;

  const double diffa0 = 2*b0*c1*x0*a0*b1*b1 + 2*c0*b1*b1*x0*b0*a1 + 2*c0*c0*b1*x0*c1*a1 + 2*c1*c1*y0*c0*a1*a0 - 2*b0*c1*c1*x0*c0*a1 - 2*b0*b0*c1*x0*b1*a1 - 2*c1*c1*y0*c0*b1*b0 + 2*b0*b0*c1*x1*b1*a1 + 2*b0*c1*c1*x1*c0*a1 - 2*b0*c1*x1*a0*b1*b1 - c1*y0*c0*c0*a1*a1 + c1*y0*c0*c0*b1*b1 + c1*y0*b0*b0*a1*a1 - c1*y0*a0*a0*b1*b1 + c1*y1*c0*c0*a1*a1 - c1*y1*c0*c0*b1*b1 - c1*y1*b0*b0*a1*a1 + c1*y1*a0*a0*b1*b1 - b1*z0*c0*c0*a1*a1 + b1*z0*b0*b0*a1*a1 - b1*z0*b0*b0*c1*c1 + b1*z0*a0*a0*c1*c1 + b1*z1*c0*c0*a1*a1 - b1*z1*b0*b0*a1*a1 + b1*z1*b0*b0*c1*c1 - b1*z1*a0*a0*c1*c1 + 2*b0* c1_3*x0*a0 - 2*b0* c1_3*x1*a0 - 2*c0* b1_3*x0*a0 + 2*c0* b1_3*x1*a0 + c1_3*y0*b0*b0 - c1_3*y0*a0*a0 - c1_3*y1*b0*b0 + c1_3*y1*a0*a0 - b1_3*z0*c0*c0 + b1_3*z0*a0*a0 + b1_3*z1*c0*c0 - b1_3*z1*a0*a0 - 2*c1*c1*y1*c0*a1*a0 + 2*c1*c1*y1*c0*b1*b0 + 2*b1*b1*z0*c0*b0*c1 - 2*b1*b1*z0*b0*a1*a0 - 2*b1*b1*z1*c0*b0*c1 + 2*b1*b1*z1*b0*a1*a0 - 2*c0*b1*x0*a0*c1*c1 - 2*c0*b1*b1*x1*b0*a1 - 2*c0*c0*b1*x1*c1*a1 + 2*c0*b1*x1*a0*c1*c1 + 2*c0*a1*y0*a0*b1*b1 - 2*c0*a1*a1*y0*b1*b0 - 2*c0*a1*y1*a0*b1*b1 + 2*c0*a1*a1*y1*b1*b0 + 2*b0*a1*a1*z0*c1*c0 - 2*b0*a1*z0*a0*c1*c1 - 2*b0*a1*a1*z1*c1*c0 + 2*b0*a1*z1*a0*c1*c1;
  const double diffb0 = -2*c1*c1*x0*c0*b1*b0 + 2*c1*c1*x0*c0*a1*a0 - c1*x0*c0*c0*a1*a1 + c1*x0*c0*c0*b1*b1 + c1*x0*b0*b0*a1*a1 - c1*x0*a0*a0*b1*b1 + c1*x1*c0*c0*a1*a1 - c1*x1*c0*c0*b1*b1 - c1*x1*b0*b0*a1*a1 + c1*x1*a0*a0*b1*b1 + a1*z0*c0*c0*b1*b1 - a1*z0*b0*b0*c1*c1 - a1*z0*a0*a0*b1*b1 + a1*z0*a0*a0*c1*c1 - a1*z1*c0*c0*b1*b1 + a1*z1*b0*b0*c1*c1 + a1*z1*a0*a0*b1*b1 - a1*z1*a0*a0*c1*c1 - 2*a0* c1_3*y0*b0 + 2*a0* c1_3*y1*b0 + c1_3*x0*b0*b0 - c1_3*x0*a0*a0 - c1_3*x1*b0*b0 + c1_3*x1*a0*a0 + a1_3*z0*c0*c0 - 2*c1*c1*x1*c0*a1*a0 + 2*c1*c1*x1*c0*b1*b0 - 2*a1*a1*z0*c0*a0*c1 + 2*a1*a1*z0*b0*a0*b1 - a1_3*z0*b0*b0 - a1_3*z1*c0*c0 + a1_3*z1*b0*b0 + 2*a1*a1*z1*c0*a0*c1 - 2*a1*a1*z1*b0*a0*b1 + 2*c0*b1*b1*x0*a1*a0 - 2*c0*b1*x0*b0*a1*a1 - 2*c0*b1*b1*x1*a1*a0 + 2*c0*b1*x1*b0*a1*a1 + 2*a0*a0*c1*y0*a1*b1 - 2*a0*c1*y0*b0*a1*a1 + 2*a0*c1*c1*y0*c0*b1 - 2*a0*a0*c1*y1*a1*b1 + 2*a0*c1*y1*b0*a1*a1 - 2*a0*c1*c1*y1*c0*b1 - 2*c0*a1*a1*y0*a0*b1 + 2*c0* a1_3*y0*b0 - 2*c0* a1_3*y1*b0 + 2*c0*a1*y0*b0*c1*c1 - 2*c0*c0*a1*y0*c1*b1 + 2*c0*a1*a1*y1*a0*b1 - 2*c0*a1*y1*b0*c1*c1 + 2*c0*c0*a1*y1*c1*b1 + 2*a0*b1*z0*b0*c1*c1 - 2*a0*b1*b1*z0*c1*c0 - 2*a0*b1*z1*b0*c1*c1 + 2*a0*b1*b1*z1*c1*c0;

  ml::Matrix diffh(1,6);
  diffh(0,0)=diffx0/K;
  diffh(0,1)=diffy0/K;
  diffh(0,2)=diffz0/K;
  K*=K;
  diffh(0,3)=diffa0/K;
  diffh(0,4)=diffb0/K;
  diffh(0,5)=0;

  /* --- Multiply Jline=dline/dq with diffh=de/dline --- */
   J.resize(1,J.nbCols());
   diffh.multiply(Jline,J);
  //J=Jline;


  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
sotFeatureLineDistance::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

  /* Line coordinates */
  const ml::Vector &line = lineSOUT(time);
  const double & x0 = line(0);
  const double & y0 = line(1);
  const double & z0 = line(2);
  const double & a0 = line(3);
  const double & b0 = line(4);
  const double & c0 = line(5);

  const ml::Vector &posRef = positionRefSIN(time);
  const double & x1 = posRef(0);
  const double & y1 = posRef(1);
  const double & z1 = posRef(2);
  const double & a1 = posRef(3);
  const double & b1 = posRef(4);
  const double & c1 = posRef(5);

  error.resize(1);
  double K = c0*c0*a1*a1 - 2*c0*a1*a0*c1 - 2*c0*b1*b0*c1 + c0*c0*b1*b1 - 2*b0*a1*a0*b1 + b0*b0*a1*a1 + b0*b0*c1*c1 + a0*a0*b1*b1 + a0*a0*c1*c1;
  error(0) = ( - b0*c1*x0 + b0*c1*x1 + c0*b1*x0 - c0*b1*x1
               + a0*c1*y0 - a0*c1*y1 - c0*a1*y0 + c0*a1*y1
               - a0*b1*z0 + a0*b1*z1 + b0*a1*z0 - b0*a1*z1 ) / K;


  /* --- DEBUG --- */
//   error.resize(6);
//   error=line-posRef;

  sotDEBUGOUT(15);
  return error ;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
ml::Vector&
sotFeatureLineDistance::computeActivation( ml::Vector& act,int time )
{
  selectionSIN(time);
  act.resize(dimensionSOUT(time)) ; act.fill(1);
  return act ;
}

void sotFeatureLineDistance::
display( std::ostream& os ) const
{
  os <<"LineDistance <"<<name<<">";
}



void sotFeatureLineDistance::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeaturePoint: "<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else  //sotFeatureAbstract::
    Entity::commandLine( cmdLine,cmdArgs,os );

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

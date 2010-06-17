/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMatrixForce.cpp
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

#include <sot-core/sotMatrixForce.h>
#include <sot-core/sotMatrixHomogeneous.h>
#include <sot-core/sotMatrixRotation.h>
#include <sot-core/sotMatrixTwist.h>
#include <sot-core/sotDebug.h>

sotMatrixForce& sotMatrixForce::
buildFrom( const sotMatrixHomogeneous& M )
{
  ml::Matrix Tx(3,3);
  Tx( 0,0 )=  0       ;  Tx( 0,1 ) = -M( 2,3 ); Tx( 0,2 ) =  M( 1,3 );
  Tx( 1,0 )=  M( 2,3 );  Tx( 1,1 ) =  0       ; Tx( 1,2 ) = -M( 0,3 );
  Tx( 2,0 )= -M( 1,3 );  Tx( 2,1 ) =  M( 0,3 ); Tx( 2,2 )=   0       ;
  sotMatrixRotation R; M.extract(R);
  ml::Matrix sk(3,3); Tx.multiply(R,sk);
  
  sotDEBUG(15) << "Tx = " << Tx << std::endl;
  sotDEBUG(15) << "Sk = " << sk << std::endl;

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	elementAt( i,j ) = R(i,j);
	elementAt( i+3,j+3 ) = R(i,j);
	elementAt( i+3,j ) = sk(i,j);
	elementAt( i,j+3 ) = 0.;
      }

  return *this;
}



sotMatrixForce& sotMatrixForce::
inverse( sotMatrixForce& Vi ) const
{
  ml::Matrix Rt(3,3); 
  ml::Matrix Sk(3,3); 
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Rt(i,j)=elementAt( j,i );
	Sk(i,j)=elementAt( i+3,j );
      }
  ml::Matrix RtS(3,3), RtSRt(3,3); 
  Rt.multiply(Sk,RtS); 
  RtS.multiply(Rt,RtSRt);

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Vi(i,j) = Rt(i,j);
	Vi(i+3,j+3) = Rt(i,j);
	Vi( i,j+3 ) = 0.;
	Vi( i,j+3 ) = RtSRt(i,j);
      }

  return Vi;
}



sotMatrixForce& sotMatrixForce::
operator=( const ml::Matrix& m2)
{
  if( (m2.nbRows()==6)&&(m2.nbCols()==6) )
    ((ml::Matrix&)*this) = m2;
    
  return *this;

}

sotMatrixTwist& sotMatrixForce::
transpose( sotMatrixTwist& Vt ) const
{ return (sotMatrixTwist&)ml::Matrix::transpose((ml::Matrix&)Vt);}

sotMatrixTwist sotMatrixForce::
transpose( void ) const
{ sotMatrixTwist F; return transpose(F); }

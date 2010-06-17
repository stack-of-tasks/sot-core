/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMatrixTwist.cpp
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

#include <sot-core/matrix-twist.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-force.h>
#include <sot-core/debug.h>

sotMatrixTwist& sotMatrixTwist::
buildFrom( const sotMatrixHomogeneous& M )
{
  ml::Matrix Tx(3,3);
  Tx( 0,0 ) = 0       ;  Tx( 0,1 )=-M( 2,3 );  Tx( 0,2 ) = M( 1,3 );
  Tx( 1,0 ) = M( 2,3 );  Tx( 1,1 )= 0       ;  Tx( 1,2 ) =-M( 0,3 );
  Tx( 2,0 ) =-M( 1,3 );  Tx( 2,1 )= M( 0,3 );  Tx( 2,2 ) = 0       ;
  sotMatrixRotation R; M.extract(R);
  ml::Matrix sk(3,3); Tx.multiply(R,sk);
  
  sotDEBUG(15) << "Tx = " << Tx << std::endl;
  sotDEBUG(15) << "Sk = " << sk << std::endl;

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	elementAt( i,j ) = R(i,j);
	elementAt( i+3,j+3 ) = R(i,j);
	elementAt( i,j+3 ) = sk(i,j);
	elementAt( i+3,j ) = 0.;
      }

  return *this;
}



sotMatrixTwist& sotMatrixTwist::
inverse( sotMatrixTwist& Vi ) const
{
  ml::Matrix Rt(3,3); 
  ml::Matrix Sk(3,3); 
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Rt(i,j)=elementAt( j,i );
	Sk(i,j)=elementAt( i,j+3 );
      }
  ml::Matrix RtS(3,3), RtSRt(3,3); 
  Rt.multiply(Sk,RtS); 
  RtS.multiply(Rt,RtSRt);

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Vi(i,j) = Rt(i,j);
	Vi(i+3,j+3) = Rt(i,j);
	Vi( i+3,j ) = 0.;
	Vi( i,j+3 ) = RtSRt(i,j);
      }

  return Vi;
}



sotMatrixTwist& sotMatrixTwist::
operator=( const ml::Matrix& m2)
{
  if( (m2.nbRows()==6)&&(m2.nbCols()==6) )
    ((ml::Matrix&)*this) = m2;
    
  return *this;

}

sotMatrixForce& sotMatrixTwist::
transpose( sotMatrixForce& Vt ) const
{ return (sotMatrixForce&)ml::Matrix::transpose((ml::Matrix&)Vt);}

sotMatrixForce sotMatrixTwist::
transpose( void ) const
{ sotMatrixForce F; return transpose(F); }

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      MatrixHomogeneous.cpp
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

#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/debug.h>

using namespace sot;


MatrixHomogeneous& MatrixHomogeneous::
buildFrom( const MatrixRotation& rot, const ml::Vector& trans )
{
  for( int i=0;i<3;++i )
    {
      elementAt( i,3 ) = trans(i);
      for( int j=0;j<3;++j )
	elementAt( i,j ) = rot( i,j );
      elementAt( 3,i ) = 0.;
    }
  
  elementAt( 3,3 ) = 1.;

  return *this;
}

ml::Matrix& MatrixHomogeneous::
extract( ml::Matrix& rot ) const
{
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      rot( i,j ) = elementAt( i,j );

  return rot;
}

MatrixRotation& MatrixHomogeneous::
extract( MatrixRotation& rot ) const
{
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      rot( i,j ) = elementAt( i,j );

  return rot;
}

ml::Vector& MatrixHomogeneous::
extract( ml::Vector& trans ) const
{
  for( int i=0;i<3;++i )
    trans( i ) = elementAt( i,3 );

  return trans;
}


MatrixHomogeneous& MatrixHomogeneous::
operator=( const ml::Matrix& m2)
{
  if( (m2.nbRows()==4)&&(m2.nbCols()==4) )
    ((ml::Matrix&)*this) = m2;
    
  return *this;

}


MatrixHomogeneous& MatrixHomogeneous::
inverse( MatrixHomogeneous& invMatrix ) const 
{
  sotDEBUGIN(25);

  ml::Matrix & inv = this->ml::Matrix::inverse( invMatrix );

  sotDEBUGOUT(25);
  return dynamic_cast< MatrixHomogeneous& > (inv);
}


ml::Vector& MatrixHomogeneous::
multiply( const ml::Vector& v1,ml::Vector& res ) const
{
  sotDEBUGIN(15);

  bool translate=true;
  if( 3==v1.size() ) res.resize(3);
  else if( 4==v1.size() )
    { res.resize(4); if(res(3)==0) { translate=false; } else res(3)=1.; }
  else {
    sotERROR << "Error while multiplying HOMOxVECTOR."<<std::endl;
    return res;
  }
  res.fill(0.);
  for( unsigned int i=0;i<3;++i )
    {
      for( unsigned int j=0;j<3;++j )
	{	  res(i)+=matrix(i,j)*v1(j); 	}
      if(translate)res(i)+=matrix(i,3);
    }

  sotDEBUGOUT(15);
  return res;
}

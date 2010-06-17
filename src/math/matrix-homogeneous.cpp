/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMatrixHomogeneous.cpp
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
#include <sot-core/sotDebug.h>

sotMatrixHomogeneous& sotMatrixHomogeneous::
buildFrom( const sotMatrixRotation& rot, const ml::Vector& trans )
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

sotMatrixRotation& sotMatrixHomogeneous::
extract( sotMatrixRotation& rot ) const
{
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      rot( i,j ) = elementAt( i,j );

  return rot;
}

ml::Vector& sotMatrixHomogeneous::
extract( ml::Vector& trans ) const
{
  for( int i=0;i<3;++i )
    trans( i ) = elementAt( i,3 );

  return trans;
}


sotMatrixHomogeneous& sotMatrixHomogeneous::
operator=( const ml::Matrix& m2)
{
  if( (m2.nbRows()==4)&&(m2.nbCols()==4) )
    ((ml::Matrix&)*this) = m2;
    
  return *this;

}


sotMatrixHomogeneous& sotMatrixHomogeneous::
inverse( sotMatrixHomogeneous& invMatrix ) const 
{
  sotDEBUGIN(25);

  ml::Matrix & inv = this->ml::Matrix::inverse( invMatrix );

  sotDEBUGOUT(25);
  return dynamic_cast< sotMatrixHomogeneous& > (inv);
}


ml::Vector& sotMatrixHomogeneous::
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

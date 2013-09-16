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

#include <sot/core/matrix-twist.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-force.hh>
#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;


MatrixTwist& MatrixTwist::
buildFrom( const MatrixHomogeneous& M )
{
  /* ovb = ova + owa x oAB
   * bvb = bRo ova + bRo (owa x oAB)
   *     = bRa ava + bRa (aRo owa) x  (aRo oAB)
   *     = bRa ava + bRa (awa) x (aAB)
   *     = bRa ava + (bRa awa) x (-bAB)
   *     = bRa ava + (bAB) x (bRa awa)
   *     = bRa ava + [bAb]x  bRa awa
   */
  dynamicgraph::Matrix Tx(3,3);
  Tx( 0,0 ) = 0       ;  Tx( 0,1 )=-M( 2,3 );  Tx( 0,2 ) = M( 1,3 );
  Tx( 1,0 ) = M( 2,3 );  Tx( 1,1 )= 0       ;  Tx( 1,2 ) =-M( 0,3 );
  Tx( 2,0 ) =-M( 1,3 );  Tx( 2,1 )= M( 0,3 );  Tx( 2,2 ) = 0       ;
  MatrixRotation R; M.extract(R);
  dynamicgraph::Matrix sk(3,3); sk.noalias() = Tx*R;

  sotDEBUG(15) << "Tx = " << Tx << std::endl;
  sotDEBUG(15) << "Sk = " << sk << std::endl;

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	((dynamicgraph::Matrix&)*this)( i,j ) = R(i,j);
	((dynamicgraph::Matrix&)*this)( i+3,j+3 ) = R(i,j);
	((dynamicgraph::Matrix&)*this)( i,j+3 ) = sk(i,j);
	((dynamicgraph::Matrix&)*this)( i+3,j ) = 0.;
      }

  return *this;
}



MatrixTwist& MatrixTwist::
inverse( MatrixTwist& Vi ) const
{
  dynamicgraph::Matrix Rt(3,3); 
  dynamicgraph::Matrix Sk(3,3); 
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Rt(i,j)=((dynamicgraph::Matrix&)*this)( j,i );
	Sk(i,j)=((dynamicgraph::Matrix&)*this)( i,j+3 );
      }
  dynamicgraph::Matrix RtS(3,3), RtSRt(3,3); 
  RtS.noalias() = Rt*Sk; 
  RtSRt.noalias() = -1 * RtS * Rt;

  //RtSRt.noalias() *= -1;

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



MatrixTwist& MatrixTwist::
operator=( const dynamicgraph::Matrix& m2)
{
  if( (m2.rows()==6)&&(m2.cols()==6) )
    ((dynamicgraph::Matrix&)*this).noalias() = m2;
    
  return *this;

}

MatrixForce& MatrixTwist::
transpose( MatrixForce& Vt ) const
{ 
  Vt.noalias() = ((dynamicgraph::Matrix&)*this).transpose();
  return Vt;
}

MatrixForce MatrixTwist::
transpose( void ) const
{ MatrixForce F; return transpose(F); }

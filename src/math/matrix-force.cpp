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

#include <sot/core/matrix-force.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-twist.hh>
#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;


MatrixForce& MatrixForce::
buildFrom( const MatrixHomogeneous& M )
{
  dynamicgraph::Matrix Tx(3,3);
  Tx( 0,0 )=  0       ;  Tx( 0,1 ) = -M( 2,3 ); Tx( 0,2 ) =  M( 1,3 );
  Tx( 1,0 )=  M( 2,3 );  Tx( 1,1 ) =  0       ; Tx( 1,2 ) = -M( 0,3 );
  Tx( 2,0 )= -M( 1,3 );  Tx( 2,1 ) =  M( 0,3 ); Tx( 2,2 )=   0       ;
  MatrixRotation R; M.extract(R);
  dynamicgraph::Matrix sk(3,3); sk = Tx*R;
  
  sotDEBUG(15) << "Tx = " << Tx << std::endl;
  sotDEBUG(15) << "Sk = " << sk << std::endl;

  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	((dynamicgraph::Matrix&)*this)( i,j ) = R(i,j);
	((dynamicgraph::Matrix&)*this)( i+3,j+3 ) = R(i,j);
	((dynamicgraph::Matrix&)*this)( i+3,j ) = sk(i,j);
	((dynamicgraph::Matrix&)*this)( i,j+3 ) = 0.;
      }

  return *this;
}



MatrixForce& MatrixForce::
inverse( MatrixForce& Vi ) const
{
  dynamicgraph::Matrix Rt(3,3); 
  dynamicgraph::Matrix Sk(3,3); 
  for( int i=0;i<3;++i )
    for( int j=0;j<3;++j )
      {
	Rt(i,j)=((dynamicgraph::Matrix&)*this)( j,i );
	Sk(i,j)=((dynamicgraph::Matrix&)*this)( i+3,j );
      }
  dynamicgraph::Matrix RtS(3,3), RtSRt(3,3); 
  RtS = Rt*Sk; 
  RtSRt = RtS*Rt;

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



MatrixForce& MatrixForce::
operator=( const dynamicgraph::Matrix& m2)
{
  if( (m2.rows()==6)&&(m2.cols()==6) )
    ((dynamicgraph::Matrix&)*this) = m2;
    
  return *this;

}

MatrixTwist& MatrixForce::
transpose( MatrixTwist& Vt ) const
{
  Vt = ((dynamicgraph::Matrix&)*this).transpose(); 
  return Vt;
}

MatrixTwist MatrixForce::
transpose( void ) const
{ MatrixTwist F; return transpose(F); }

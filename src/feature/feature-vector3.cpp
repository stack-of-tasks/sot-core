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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
//#define VP_DEBUG
//#define VP_DEBUG_MODE 45
#include <sot/core/debug.hh>
#include <sot/core/feature-vector3.hh>
#include <sot/core/exception-feature.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace std;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureVector3,"FeatureVector3");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureVector3::
FeatureVector3( const string& pointName )
  : FeatureAbstract( pointName )
    ,vectorSIN( NULL,"sotFeatureVector3("+name+")::input(vector3)::vector" )
    ,positionSIN( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position" )
    ,articularJacobianSIN( NULL,"sotFeatureVector3("+name+")::input(matrix)::Jq" )
    ,positionRefSIN( NULL,"sotFeatureVector3("+name+")::input(vector)::positionRef" )
{
  jacobianSOUT.addDependency( positionSIN );
  jacobianSOUT.addDependency( articularJacobianSIN );

  errorSOUT.addDependency( vectorSIN );
  errorSOUT.addDependency( positionSIN );
  errorSOUT.addDependency( positionRefSIN );

  signalRegistration( vectorSIN<<positionSIN
                      <<articularJacobianSIN<<positionRefSIN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureVector3::
getDimension( unsigned int & dim, int /*time*/ )
{
  sotDEBUG(25)<<"# In {"<<endl;

  return dim=3;
}



/* --------------------------------------------------------------------- */
/** Compute the interaction matrix from a subset of
 * the possible features.
 */
dynamicgraph::Matrix& FeatureVector3::
computeJacobian( dynamicgraph::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const dynamicgraph::Matrix & Jq = articularJacobianSIN(time);
  const dynamicgraph::Vector & vect = vectorSIN(time);
  const MatrixHomogeneous & M = positionSIN(time);
  MatrixRotation R; M.extract(R);

  dynamicgraph::Matrix Skew(3,3);
  Skew( 0,0 ) = 0        ; Skew( 0,1 )=-vect( 2 );  Skew( 0,2 ) = vect( 1 );
  Skew( 1,0 ) = vect( 2 ); Skew( 1,1 )= 0        ;  Skew( 1,2 ) =-vect( 0 );
  Skew( 2,0 ) =-vect( 1 ); Skew( 2,1 )= vect( 0 );  Skew( 2,2 ) = 0        ;

  dynamicgraph::Matrix RSk(3,3); RSk.noalias() = R*Skew;

  J.resize(3,Jq.cols());
  for( unsigned int i=0;i<3;++i )
    for( int j=0;j<Jq.cols();++j )
      {
        J(i,j)=0;
        for( unsigned int k=0;k<3;++k )
          {  J(i,j)-=RSk(i,k)*Jq(k+3,j);   }
      }

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
*a the possible features.
 */
dynamicgraph::Vector&
FeatureVector3::computeError( dynamicgraph::Vector& Mvect3,int time )
{
  sotDEBUGIN(15);

  const MatrixHomogeneous & M = positionSIN(time);
  const dynamicgraph::Vector & vect = vectorSIN(time);
  const dynamicgraph::Vector & vectdes = positionRefSIN(time);

  sotDEBUG(15) << "M = " << M << std::endl;
  sotDEBUG(15) << "v = " << vect << std::endl;
  sotDEBUG(15) << "vd = " << vectdes << std::endl;

  MatrixRotation R; M.extract(R);
  Mvect3.resize(3);
  Mvect3.noalias() = R*vect;
  Mvect3.noalias() -= vectdes;

  sotDEBUGOUT(15);
  return Mvect3 ;
}

void FeatureVector3::
display( std::ostream& os ) const
{
  os <<"Vector3 <"<<name<<">";
}



void FeatureVector3::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeatureVector: "<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else  //FeatureAbstract::
    Entity::commandLine( cmdLine,cmdArgs,os );

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

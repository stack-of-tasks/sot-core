/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Feature1D.cpp
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
#include <sot-core/debug.h>
#include <sot-core/feature-1d.h>
#include <sot-core/exception-feature.h>
using namespace std;

#include <sot-core/factory.h>


using namespace sot;
using namespace dynamicgraph;
SOT_FACTORY_FEATURE_PLUGIN(Feature1D,"Feature1D");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



Feature1D::
Feature1D( const string& pointName )
  : FeatureAbstract( pointName )
    ,errorSIN( NULL,"sotFeature1D("+name+")::input(vector)::errorIN" )
    ,jacobianSIN( NULL,"sotFeature1D("+name+")::input(matrix)::jacobianIN" )
    ,activationSIN( NULL,"sotFeature1D("+name+")::input(matrix)::activationIN" )
{
  jacobianSOUT.addDependency( jacobianSIN );
  errorSOUT.addDependency( errorSIN );
  activationSOUT.addDependency( activationSIN );

  signalRegistration( errorSIN<<jacobianSIN<<activationSIN );
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& Feature1D::
getDimension( unsigned int & dim, int time ) 
{  
  sotDEBUG(25)<<"# In {"<<endl;

  dim = 1;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}


ml::Vector& Feature1D::
computeError( ml::Vector& res,int time )
{ 
  const ml::Vector& err = errorSIN.access(time);
  res.resize(1); res(0)=err.scalarProduct(err)*.5;

  return res; 

}



ml::Matrix& Feature1D::
computeJacobian( ml::Matrix& res,int time )
{ 
  sotDEBUGIN(15);

  const ml::Matrix& Jac = jacobianSIN.access(time);
  const ml::Vector& err = errorSIN.access(time);

  res.resize( 1,Jac.nbCols() );res.fill(0);
  for( unsigned int j=0;j<Jac.nbCols();++j )
    for( unsigned int i=0;i<Jac.nbRows();++i )
      res(0,j)+=err(i)*Jac(i,j);
      
  sotDEBUGOUT(15);
  return res; 
}

ml::Vector& Feature1D::
computeActivation( ml::Vector& res,int time )
{ 
  if( activationSIN )
    {
      const ml::Vector& err = activationSIN.access(time);
      res.resize( 1 ); res(0)=0;
      for( unsigned int i=0;i<err.size();++i ) res(0)+=err(i);
    }
  else
    {
      res.resize( dimensionSOUT(time) );
      res.fill( 1. );
    }
      
  return res; 
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void Feature1D::
display( std::ostream& os ) const
{
  os <<"1D <"<<name<<">: " <<std::endl;

  try{ 
    os << "  error= "<< errorSIN.accessCopy() << endl
       << "  J    = "<< jacobianSIN.accessCopy() << endl
       << "  act  = "<<activationSIN.accessCopy() << endl;
  }  catch(ExceptionAbstract e){ os<< " All SIN not set."; }
}


// void Feature1D::
// commandLine( const std::string& cmdLine,
// 	     std::istringstream& cmdArgs,
// 	     std::ostream& os )
// {
//   if( cmdLine == "help" )
//     {
//       os << "Feature1D: " 
//     }
//   else { Entity::commandLine( cmdLine,cmdArgs,os); }
// }


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

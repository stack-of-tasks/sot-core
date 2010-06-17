/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeature1D.cpp
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
#include <sot-core/sotFeature1D.h>
#include <sot-core/sotExceptionFeature.h>
using namespace std;

#include <sot-core/sotFactory.h>
SOT_FACTORY_FEATURE_PLUGIN(sotFeature1D,"Feature1D");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



sotFeature1D::
sotFeature1D( const string& pointName )
  : sotFeatureAbstract( pointName )
    ,errorSIN( NULL,"sotFeature1D("+name+")::input(vector)::errorIN" )
    ,jacobianSIN( NULL,"sotFeature1D("+name+")::input(matrix)::jacobianIN" )
    ,activationSIN( NULL,"sotFeature1D("+name+")::input(matrix)::activationIN" )
{
  jacobianSOUT.addDependancy( jacobianSIN );
  errorSOUT.addDependancy( errorSIN );
  activationSOUT.addDependancy( activationSIN );

  signalRegistration( errorSIN<<jacobianSIN<<activationSIN );
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& sotFeature1D::
getDimension( unsigned int & dim, int time ) 
{  
  sotDEBUG(25)<<"# In {"<<endl;

  dim = 1;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}


ml::Vector& sotFeature1D::
computeError( ml::Vector& res,int time )
{ 
  const ml::Vector& err = errorSIN.access(time);
  res.resize(1); res(0)=err.scalarProduct(err)*.5;

  return res; 

}



ml::Matrix& sotFeature1D::
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

ml::Vector& sotFeature1D::
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

void sotFeature1D::
display( std::ostream& os ) const
{
  os <<"1D <"<<name<<">: " <<std::endl;

  try{ 
    os << "  error= "<< errorSIN.accessCopy() << endl
       << "  J    = "<< jacobianSIN.accessCopy() << endl
       << "  act  = "<<activationSIN.accessCopy() << endl;
  }  catch(sotExceptionAbstract e){ os<< " All SIN not set."; }
}


// void sotFeature1D::
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

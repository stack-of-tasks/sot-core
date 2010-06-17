/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeaturePoint6d.cpp
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
//#define VP_DEBUG
#define VP_DEBUG_MODE 45
#include <sot-core/sotDebug.h>
#include <sot-core/sotFeaturePoint6d.h>
#include <sot-core/exception-feature.h>

#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/vector-utheta.h>

using namespace std;

#include <sot-core/factory.h>
SOT_FACTORY_FEATURE_PLUGIN(sotFeaturePoint6d,"FeaturePoint6d");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


const sotFeaturePoint6d::ComputationFrameType sotFeaturePoint6d::
COMPUTATION_FRAME_DEFAULT = FRAME_DESIRED;

sotFeaturePoint6d::
sotFeaturePoint6d( const string& pointName )
  : sotFeatureAbstract( pointName )
    ,computationFrame( COMPUTATION_FRAME_DEFAULT )
    ,positionSIN( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position" )
    ,articularJacobianSIN( NULL,"sotFeaturePoint6d("+name+")::input(matrix)::Jq" )
{
  jacobianSOUT.addDependancy( positionSIN );
  jacobianSOUT.addDependancy( articularJacobianSIN );

  errorSOUT.addDependancy( positionSIN );

  activationSOUT.removeDependancy( desiredValueSIN );

  signalRegistration( positionSIN<<articularJacobianSIN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& sotFeaturePoint6d::
getDimension( unsigned int & dim, int time )
{
  sotDEBUG(25)<<"# In {"<<endl;

  const sotFlags &fl = selectionSIN.access(time);

  dim = 0;
  for( int i=0;i<6;++i ) if( fl(i) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}


/** Compute the interaction matrix from a subset of
 * the possible features.
 */
ml::Matrix& sotFeaturePoint6d::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const ml::Matrix & Jq = articularJacobianSIN(time);
  const int & dim = dimensionSOUT(time);
  const sotFlags &fl = selectionSIN(time);

  sotDEBUG(25)<<"dim = "<<dimensionSOUT(time)<<" time:" << time << " "
              << dimensionSOUT.getTime() << " " << dimensionSOUT.getReady() << endl;
  sotDEBUG(25)<<"selec = "<<selectionSIN(time)<<" time:" << time << " "
              << selectionSIN.getTime() << " " << selectionSIN.getReady() << endl;

  sotDEBUG(15) <<"Dimension="<<dim<<std::endl;

  const unsigned int cJ = Jq.nbCols();
  J.resize(dim,cJ) ;
  ml::Matrix LJq(6,cJ);

  if( FRAME_CURRENT==computationFrame )
    {
      /* The Jacobian on rotation is equal to Jr = - hdRh Jr6d.
       * The Jacobian in translation is equalt to Jt = [hRw(wthd-wth)]x Jr - Jt. */
      sotFeatureAbstract * sdesAbs = desiredValueSIN(time);
      sotFeaturePoint6d * sdes = dynamic_cast<sotFeaturePoint6d*>(sdesAbs);

      const sotMatrixHomogeneous& wMh = positionSIN(time);
      sotMatrixRotation wRh;      wMh.extract(wRh);
      sotMatrixRotation wRhd;
      ml::Vector hdth(3),Rhdth(3);

      if(NULL!=sdes)
        {
          const sotMatrixHomogeneous& wMhd = sdes->positionSIN(time);
          wMhd.extract(wRhd);
          for( unsigned int i=0;i<3;++i ) hdth(i)=wMhd(i,3)-wMh(i,3);
        }
      else
        {
          wRhd.setIdentity();
          for( unsigned int i=0;i<3;++i ) hdth(i)=-wMh(i,3);
        }
      wRh.inverse().multiply(hdth,Rhdth);
      sotMatrixRotation hdRh; wRhd.inverse().multiply(wRh,hdRh);

      ml::Matrix Lx(6,6);
      for(unsigned int i=0;i<3;i++)
	{for(unsigned int j=0;j<3;j++)
            {
              if( i==j) { Lx(i,j)=-1; Lx(i,j+3)=0; } else { Lx(i,j)=0; }
              Lx(i,j+3)=0;
              Lx(i+3,j+3)=-hdRh(i,j);
            }
        }
      const double & X=Rhdth(0), &Y=Rhdth(1), &Z=Rhdth(2);
      Lx(0,4) = -Z ;      Lx(0,5) = Y ;       Lx(1,3) = Z ;
      Lx(1,5) = -X ;      Lx(2,3) = -Y ;      Lx(2,4) = X ;
      sotDEBUG(15) << "Lx= "<<Lx<<endl;

      Lx.multiply(Jq,LJq);
    }
  else
    {
      /* The Jacobian in rotation is equal to Jr = hdJ = hdRh Jr.
       * The Jacobian in translation is equal to Jr = hdJ = hdRh Jr. */

      sotFeatureAbstract * sdesAbs = desiredValueSIN(time);
      sotFeaturePoint6d * sdes = dynamic_cast<sotFeaturePoint6d*>(sdesAbs);

      const sotMatrixHomogeneous& wMh = positionSIN(time);
      sotMatrixRotation wRh; wMh.extract(wRh);
      sotMatrixRotation hdRh;

      if( NULL!=sdes )
        {
          const sotMatrixHomogeneous& wMhd = sdes->positionSIN(time);
          sotMatrixRotation wRhd; wMhd.extract(wRhd);
          wRhd.inverse().multiply( wRh,hdRh );
        }
      else
        { hdRh = wRh; }

       LJq.fill(0);
       for(unsigned int i=0;i<3;i++)
         for(unsigned int j=0;j<cJ;j++)
           {
             for(unsigned int k=0;k<3;k++)
               {
                 LJq(i,j)+=hdRh(i,k)*Jq(k,j);
                 LJq(i+3,j)+=hdRh(i,k)*Jq(k+3,j);
               }
           }
    }

  /* Select the active line of Jq. */
  unsigned int rJ = 0;
  for( unsigned int r=0;r<6;++r )
    if( fl(r) )
      {
	for( unsigned int c=0;c<cJ;++c )
	  J(rJ,c)=LJq(r,c);
	rJ ++;
      }

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

#define SOT_COMPUTE_H1MH2(wMh,wMhd,hMhd) {                 \
	sotMatrixHomogeneous hMw; wMh.inverse(hMw);        \
	sotDEBUG(15)<<"hMw = "<<hMw<<endl;                 \
	hMw.multiply( wMhd,hMhd );                         \
	sotDEBUG(15)<<"hMhd = "<<hMhd<<endl;               \
      }


/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeaturePoint6d::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

  const sotFlags &fl = selectionSIN(time);
  sotFeatureAbstract * sdesAbs = desiredValueSIN(time);
  sotFeaturePoint6d * sdes = dynamic_cast<sotFeaturePoint6d*>(sdesAbs);

  const sotMatrixHomogeneous& wMh = positionSIN(time);
  sotDEBUG(15)<<"wMh = "<<wMh<<endl;

  /* Computing only translation:                                        *
   * trans( hMw wMhd ) = htw + hRw wthd                                 *
   *                   = -hRw wth + hrW wthd                            *
   *                   = hRw ( wthd - wth )                             *
   * The second line is obtained by writting hMw as the inverse of wMh. */

  sotMatrixHomogeneous hMhd;
  if(NULL!=sdes)
    {
      const sotMatrixHomogeneous& wMhd = sdes->positionSIN(time);
      sotDEBUG(15)<<"wMhd = "<<wMhd<<endl;
      switch(computationFrame)
        {
        case FRAME_CURRENT:
          SOT_COMPUTE_H1MH2(wMh,wMhd,hMhd); break;
        case FRAME_DESIRED:
          SOT_COMPUTE_H1MH2(wMhd,wMh,hMhd); break; // Compute hdMh indeed.
        };
    }
  else
    {
      switch(computationFrame)
        {
        case FRAME_CURRENT:
          hMhd=wMh.inverse(); break;
        case FRAME_DESIRED:
          hMhd=wMh; break; // Compute hdMh indeed.
        };
    }

  sotDEBUG(25)<<"dim = "<<dimensionSOUT(time)<<" time:" << time << " "
              << dimensionSOUT.getTime() << " " << dimensionSOUT.getReady() << endl;
  sotDEBUG(25)<<"selec = "<<selectionSIN(time)<<" time:" << time << " "
              << selectionSIN.getTime() << " " << selectionSIN.getReady() << endl;

  error.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<3;++i )
    { if( fl(i) ) error(cursor++) = hMhd(i,3); }

  if(fl(3)||fl(4)||fl(5))
    {
      sotMatrixRotation hRhd; hMhd.extract( hRhd );
      sotVectorUTheta hrhd; hrhd.fromMatrix( hRhd );
      for( unsigned int i=0;i<3;++i )
        { if( fl(i+3) ) error(cursor++) = hrhd(i); }
    }

  sotDEBUGOUT(15);
  return error ;
}


/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeaturePoint6d::computeActivation( ml::Vector& act,int time )
{
  selectionSIN(time);
  act.resize(dimensionSOUT(time)) ; act.fill(1);
  return act ;
}


static const char * featureNames  []
= { "X ",
    "Y ",
    "Z ",
    "RX",
    "RY",
    "RZ"  };
void sotFeaturePoint6d::
display( std::ostream& os ) const
{
  os <<"Point6d <"<<name<<">: (" ;

  try{
    const sotFlags &fl = selectionSIN.accessCopy();
    bool first = true;
    for( int i=0;i<6;++i )
      if( fl(i) )
	{
	  if( first ) { first = false; } else { os << ","; }
	  os << featureNames[i];
	}
    os<<") ";
  }  catch(sotExceptionAbstract e){ os<< " selectSIN not set."; }
}



void sotFeaturePoint6d::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeaturePoint: "<<endl
	 << "  - frame [{desired|current}]: get/set the computation frame."<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="frame" )
    {
      cmdArgs>>std::ws;
      if(cmdArgs.good())
	{
	  std::string frameStr; cmdArgs >> frameStr;
	  if( frameStr =="current" ) { computationFrame = FRAME_CURRENT; }
	  else { computationFrame = FRAME_DESIRED; }
	}
      else {
	os << "frame = ";
	if( FRAME_DESIRED==computationFrame ) os << "desired" << std::endl;
	else if( FRAME_CURRENT==computationFrame ) os << "current" << std::endl;
      }
    }
  else  //sotFeatureAbstract::
    Entity::commandLine( cmdLine,cmdArgs,os );

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

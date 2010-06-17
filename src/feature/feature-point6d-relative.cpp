/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFeaturePoint6dRelative.cpp
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
#include <sot-core/feature-point6d-relative.h>
#include <sot-core/exception-feature.h>

#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-twist.h>
#include <dynamic-graph/pool.h>

using namespace std;

#include <sot-core/factory.h>
SOT_FACTORY_FEATURE_PLUGIN(sotFeaturePoint6dRelative,"FeaturePoint6dRelative");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



sotFeaturePoint6dRelative::
sotFeaturePoint6dRelative( const string& pointName )
  : sotFeaturePoint6d( pointName )
  ,positionReferenceSIN( NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::positionRef" )
  ,articularJacobianReferenceSIN( NULL,"sotFeaturePoint6dRelative("+name+")::input(matrix)::JqRef" )
  ,dotpositionSIN(NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::dotposition" )
  ,dotpositionReferenceSIN(NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::dotpositionRef" )
  ,errordotSOUT(boost::bind(&sotFeaturePoint6dRelative::computeErrorDot,this,_1,_2),
		selectionSIN<<desiredValueSIN,
		"sotFeatureAbstract("+name+")::output(vector)::errordot" )
{
  jacobianSOUT.addDependancy( positionReferenceSIN );
  jacobianSOUT.addDependancy( articularJacobianReferenceSIN );

  errorSOUT.addDependancy( positionReferenceSIN );

  errordotSOUT.addDependancy(dotpositionReferenceSIN);

  activationSOUT.removeDependancy( desiredValueSIN );

  signalRegistration( positionReferenceSIN<<articularJacobianReferenceSIN );

  signalRegistration( dotpositionSIN <<
		      dotpositionReferenceSIN <<
		      errordotSOUT );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/** Compute the interaction matrix from a subset of
 * the possible features. 
 */
ml::Matrix& sotFeaturePoint6dRelative::
computeJacobian( ml::Matrix& Jres,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const ml::Matrix & Jq = articularJacobianSIN(time);
  const ml::Matrix & JqRef = articularJacobianReferenceSIN(time);
  const sotMatrixHomogeneous & wMp = positionSIN(time);
  const sotMatrixHomogeneous & wMpref = positionReferenceSIN(time);

  const unsigned int cJ = Jq.nbCols();
  ml::Matrix J(6,cJ);
  {
    sotMatrixHomogeneous pMw;  wMp.inverse(pMw);
    sotMatrixHomogeneous pMpref; pMw.multiply( wMpref,pMpref );
    sotMatrixTwist pVpref; pVpref.buildFrom(pMpref );
    pVpref.multiply( JqRef,J );
    J -= Jq;
  }

  const sotFlags &fl = selectionSIN(time);
  const int dim = dimensionSOUT(time);
  sotDEBUG(15) <<"Dimension="<<dim<<std::endl;
  Jres.resize(dim,cJ) ;

  unsigned int rJ = 0;
  for( unsigned int r=0;r<6;++r )
    if( fl(r) )
      {
	for( unsigned int c=0;c<cJ;++c )
	  Jres(rJ,c)=J(r,c);
	rJ ++;
      }

  sotDEBUG(15)<<"# Out }"<<endl;
  return Jres;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeaturePoint6dRelative::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

//   /* TODO */
//   error.resize(6); error.fill(.0);

  const sotMatrixHomogeneous & wMp = positionSIN(time);
  const sotMatrixHomogeneous & wMpref = positionReferenceSIN(time);

  sotMatrixHomogeneous pMw;  wMp.inverse(pMw);
  sotMatrixHomogeneous pMpref; pMw.multiply( wMpref,pMpref );
  
  sotMatrixHomogeneous Merr;
  try
    {
      sotFeatureAbstract * sdesAbs = desiredValueSIN(time);
    
      sotFeaturePoint6dRelative * sdes = dynamic_cast<sotFeaturePoint6dRelative*>(sdesAbs);
      if( sdes )
	{
	  const sotMatrixHomogeneous & wMp_des = sdes->positionSIN(time);
	  const sotMatrixHomogeneous & wMpref_des = sdes->positionReferenceSIN(time);
	  
	  sotMatrixHomogeneous pMw_des;  wMp_des.inverse(pMw_des);
	  sotMatrixHomogeneous pMpref_des; pMw_des.multiply( wMpref_des,pMpref_des );
	  sotMatrixHomogeneous Minv; pMpref_des.inverse(Minv);
	  pMpref.multiply(Minv,Merr);
	} else {

	  sotFeaturePoint6d * sdes6d = dynamic_cast<sotFeaturePoint6d*>(sdesAbs);
	  if( sdes6d )
	    {
	      const sotMatrixHomogeneous & Mref = sdes6d->positionSIN(time);
	      sotMatrixHomogeneous Minv; Mref.inverse(Minv);
	      pMpref.multiply(Minv,Merr);
	    } else Merr=pMpref;
	}
    } catch( ... ) { Merr=pMpref; }
  
  sotMatrixRotation Rerr; Merr.extract( Rerr );
  sotVectorUTheta rerr; rerr.fromMatrix( Rerr );

  const sotFlags &fl = selectionSIN(time);
  error.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<3;++i )
    { if( fl(i) ) error(cursor++) = Merr(i,3); } 
  for( unsigned int i=0;i<3;++i )
    { if( fl(i+3) ) error(cursor++) = rerr(i); }

  sotDEBUGOUT(15);
  return error ;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 * 
 * This is computed by the desired feature.
 */
ml::Vector&
sotFeaturePoint6dRelative::computeErrorDot( ml::Vector& errordot,int time )
{
  sotDEBUGIN(15);

  //   /* TODO */
  //   error.resize(6); error.fill(.0);
  const sotMatrixHomogeneous & wMp = positionSIN(time);
  const sotMatrixHomogeneous & wMpref = positionReferenceSIN(time);  
  const sotMatrixHomogeneous & wdMp = dotpositionSIN(time);
  const sotMatrixHomogeneous & wdMpref = dotpositionReferenceSIN(time);

  sotDEBUG(15) << "wdMp :" <<wdMp << endl;
  sotDEBUG(15) << "wdMpref :" <<wdMpref << endl;

  sotMatrixRotation dRerr;
  ml::Vector dtrerr;

  try
    {
      sotMatrixRotation wRp;    wMp.extract(wRp);
      sotMatrixRotation wRpref; wMpref.extract(wRpref );
      sotMatrixRotation wdRp; wdMp.extract(wdRp);
      sotMatrixRotation wdRpref; wdMpref.extract(wdRpref );
      
      ml::Vector trp(3); wMp.extract(trp);
      ml::Vector trpref(3); wMpref.extract(trpref);
      ml::Vector trdp(3); wdMp.extract(trdp);
      ml::Vector trdpref(3); wdMpref.extract(trdpref);
      
      sotDEBUG(15) << "Everything is extracted" <<endl;
      sotMatrixRotation wdRpt,wRpt,op1,op2; 
      wdRp.transpose(wdRpt);wdRpt.multiply(wRpref, op1);
      wRp.transpose(wRpt);wRpt.multiply(wdRpref,op2);
      op1.addition(op2,dRerr);

      sotDEBUG(15) << "dRerr" << dRerr << endl;
      ml::Vector trtmp1(3),vop1(3),vop2(3);
      trpref.substraction(trp,trtmp1);
      wdRpt.multiply(trtmp1,vop1);
      trdpref.substraction(trdp,trtmp1);
      wRpt.multiply(trtmp1,vop2);
      vop1.addition(vop2,dtrerr);

      sotDEBUG(15) << "dtrerr" << dtrerr << endl;
      

    } catch( ... ) { sotDEBUG(15) << "You've got a problem with errordot." << std::endl; }
  
  sotVectorUTheta rerr; rerr.fromMatrix( dRerr );

  const sotFlags &fl = selectionSIN(time);
  errordot.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<3;++i )
    { if( fl(i) ) errordot(cursor++) = dtrerr(i); } 
  for( unsigned int i=0;i<3;++i )
    { if( fl(i+3) ) errordot(cursor++) = rerr(i); }

  sotDEBUGOUT(15);
  return errordot ;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
sotFeaturePoint6dRelative::computeActivation( ml::Vector& act,int time )
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
void sotFeaturePoint6dRelative::
display( std::ostream& os ) const
{
  os <<"Point6dRelative <"<<name<<">: (" ;

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


void sotFeaturePoint6dRelative::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeaturePoint6dRelative: "<<endl
	 << "  - initSdes <feature>: init <feature> by copy of the current value."<<endl;
      sotFeaturePoint6d::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="initSdes" )
    {
      cmdArgs>>std::ws; 
      if(cmdArgs.good())
	{
	  std::string nameSdes; cmdArgs >> nameSdes;
	  sotFeaturePoint6dRelative & sdes 
	    = dynamic_cast< sotFeaturePoint6dRelative &> (pool.getEntity( nameSdes ));
	  const int timeCurr = positionSIN.getTime() +1;
	  positionSIN.recompute( timeCurr );
	  positionReferenceSIN.recompute( timeCurr );

	  sdes.positionSIN.setConstant( positionSIN.accessCopy() );
	  sdes.positionReferenceSIN.setConstant( positionReferenceSIN.accessCopy() );

	}
    }
  else  
    sotFeaturePoint6d::commandLine( cmdLine,cmdArgs,os );

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

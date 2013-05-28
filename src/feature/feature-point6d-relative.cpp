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
#include <sot/core/debug.hh>
#include <sot/core/feature-point6d-relative.hh>
#include <sot/core/exception-feature.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-twist.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/all-commands.h>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoint6dRelative,"FeaturePoint6dRelative");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



FeaturePoint6dRelative::
FeaturePoint6dRelative( const string& pointName )
  : FeaturePoint6d( pointName )
  ,positionReferenceSIN( NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::positionRef" )
  ,articularJacobianReferenceSIN( NULL,"sotFeaturePoint6dRelative("+name+")::input(matrix)::JqRef" )
  ,dotpositionSIN(NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::dotposition" )
  ,dotpositionReferenceSIN(NULL,"sotFeaturePoint6dRelative("+name+")::input(matrixHomo)::dotpositionRef" )
{
  jacobianSOUT.addDependency( positionReferenceSIN );
  jacobianSOUT.addDependency( articularJacobianReferenceSIN );

  errorSOUT.addDependency( positionReferenceSIN );

  errordotSOUT.addDependency(dotpositionReferenceSIN);

  signalRegistration( positionReferenceSIN<<articularJacobianReferenceSIN );

  signalRegistration( dotpositionSIN <<
		      dotpositionReferenceSIN <<
		      errordotSOUT );

  errordotSOUT.setFunction (boost::bind (&FeaturePoint6dRelative::computeErrordot,
					 this, _1, _2));
  initCommands();
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/** Compute the interaction matrix from a subset of
 * the possible features.
 */
ml::Matrix& FeaturePoint6dRelative::
computeJacobian( ml::Matrix& Jres,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const ml::Matrix & Jq = articularJacobianSIN(time);
  const ml::Matrix & JqRef = articularJacobianReferenceSIN(time);
  const MatrixHomogeneous & wMp = positionSIN(time);
  const MatrixHomogeneous & wMpref = positionReferenceSIN(time);

  const unsigned int cJ = Jq.nbCols();
  ml::Matrix J(6,cJ);
  {
    MatrixHomogeneous pMw;  wMp.inverse(pMw);
    MatrixHomogeneous pMpref; pMw.multiply( wMpref,pMpref );
    MatrixTwist pVpref; pVpref.buildFrom(pMpref );
    pVpref.multiply( JqRef,J );
    J -= Jq;
  }

  const Flags &fl = selectionSIN(time);
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
FeaturePoint6dRelative::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

//   /* TODO */
//   error.resize(6); error.fill(.0);

  const MatrixHomogeneous & wMp = positionSIN(time);
  const MatrixHomogeneous & wMpref = positionReferenceSIN(time);

  MatrixHomogeneous pMw;  wMp.inverse(pMw);
  MatrixHomogeneous pMpref; pMw.multiply( wMpref,pMpref );

  MatrixHomogeneous Merr;
  try
    {
      if( isReferenceSet() )
	{
	  // TODO: Deal with the case of FeaturePoint6dRelative reference without dcast
	  FeaturePoint6dRelative * sdes6d = dynamic_cast<FeaturePoint6dRelative*>(getReference());
	  if( NULL!=sdes6d )
	    {
	         const MatrixHomogeneous & wMp_des = sdes6d->positionSIN(time);
		 const MatrixHomogeneous & wMpref_des = sdes6d->positionReferenceSIN(time);
		 MatrixHomogeneous pMw_des;  wMp_des.inverse(pMw_des);
		 MatrixHomogeneous pMpref_des; pMw_des.multiply( wMpref_des,pMpref_des );
		 MatrixHomogeneous Minv; pMpref_des.inverse(Minv);
		 pMpref.multiply(Minv,Merr);
	    }
	  else
	    {
	      const MatrixHomogeneous & Mref = getReference()->positionSIN(time);
	      MatrixHomogeneous Minv; Mref.inverse(Minv);
	      pMpref.multiply(Minv,Merr);
	    }
	}
      else
	{
	  Merr=pMpref;
	}
    } catch( ... ) { Merr=pMpref; }

  MatrixRotation Rerr; Merr.extract( Rerr );
  VectorUTheta rerr; rerr.fromMatrix( Rerr );

  const Flags &fl = selectionSIN(time);
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
FeaturePoint6dRelative::computeErrorDot( ml::Vector& errordot,int time )
{
  sotDEBUGIN(15);

  //   /* TODO */
  //   error.resize(6); error.fill(.0);
  const MatrixHomogeneous & wMp = positionSIN(time);
  const MatrixHomogeneous & wMpref = positionReferenceSIN(time);
  const MatrixHomogeneous & wdMp = dotpositionSIN(time);
  const MatrixHomogeneous & wdMpref = dotpositionReferenceSIN(time);

  sotDEBUG(15) << "wdMp :" <<wdMp << endl;
  sotDEBUG(15) << "wdMpref :" <<wdMpref << endl;

  MatrixRotation dRerr;
  ml::Vector dtrerr;

  try
    {
      MatrixRotation wRp;    wMp.extract(wRp);
      MatrixRotation wRpref; wMpref.extract(wRpref );
      MatrixRotation wdRp; wdMp.extract(wdRp);
      MatrixRotation wdRpref; wdMpref.extract(wdRpref );

      ml::Vector trp(3); wMp.extract(trp);
      ml::Vector trpref(3); wMpref.extract(trpref);
      ml::Vector trdp(3); wdMp.extract(trdp);
      ml::Vector trdpref(3); wdMpref.extract(trdpref);

      sotDEBUG(15) << "Everything is extracted" <<endl;
      MatrixRotation wdRpt,wRpt,op1,op2;
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

  VectorUTheta rerr; rerr.fromMatrix( dRerr );

  const Flags &fl = selectionSIN(time);
  errordot.resize(dimensionSOUT(time)) ;
  unsigned int cursor = 0;
  for( unsigned int i=0;i<3;++i )
    { if( fl(i) ) errordot(cursor++) = dtrerr(i); }
  for( unsigned int i=0;i<3;++i )
    { if( fl(i+3) ) errordot(cursor++) = rerr(i); }

  sotDEBUGOUT(15);
  return errordot ;
}

static const char * featureNames  []
= { "X ",
    "Y ",
    "Z ",
    "RX",
    "RY",
    "RZ"  };
void FeaturePoint6dRelative::
display( std::ostream& os ) const
{
  os <<"Point6dRelative <"<<name<<">: (" ;

  try{
    const Flags &fl = selectionSIN.accessCopy();
    bool first = true;
    for( int i=0;i<6;++i )
      if( fl(i) )
	{
	  if( first ) { first = false; } else { os << ","; }
	  os << featureNames[i];
	}
    os<<") ";
  }  catch(ExceptionAbstract e){ os<< " selectSIN not set."; }
}


void FeaturePoint6dRelative::
initCommands( void )
{
  using namespace command;
  addCommand( "initSdes",
	      makeCommandVoid1( *this,&FeaturePoint6dRelative::initSdes,
				docCommandVoid1( "Initialize the desired feature.",
						 "string (desired feature name)")));
}

/* Initialise the reference value: set up the sdes signal of the current feature,
 * and freezes the position and position-reference of the desired feature.
 */
void FeaturePoint6dRelative::
initSdes( const std::string & nameSdes )
{
  FeaturePoint6dRelative & sdes
    = dynamic_cast< FeaturePoint6dRelative &>
    (dg::PoolStorage::getInstance()->getEntity(nameSdes));

  setReference(&sdes);

  const int timeCurr = positionSIN.getTime() +1;
  positionSIN.recompute( timeCurr );
  positionReferenceSIN.recompute( timeCurr );

  sdes.positionSIN.setConstant( positionSIN.accessCopy() );
  sdes.positionReferenceSIN.setConstant( positionReferenceSIN.accessCopy() );
}


void FeaturePoint6dRelative::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeaturePoint6dRelative: "<<endl
	 << "  - initSdes <feature>: init <feature> by copy of the current value."<<endl;
      FeaturePoint6d::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="initSdes" )
    {
      cmdArgs>>std::ws;
      if(cmdArgs.good())
	{
	  std::string nameSdes; cmdArgs >> nameSdes;
	  initSdes( nameSdes );
	}
    }
  else
    FeaturePoint6d::commandLine( cmdLine,cmdArgs,os );

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

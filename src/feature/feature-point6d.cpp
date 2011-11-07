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
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <sot/core/debug.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/exception-feature.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-utheta.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeaturePoint6d,"FeaturePoint6d");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


const FeaturePoint6d::ComputationFrameType FeaturePoint6d::
COMPUTATION_FRAME_DEFAULT = FRAME_DESIRED;

FeaturePoint6d::
FeaturePoint6d( const string& pointName )
  : FeatureAbstract( pointName )
    ,computationFrame_( COMPUTATION_FRAME_DEFAULT )
    ,positionSIN( NULL,"sotFeaturePoint6d("+name+")::input(matrixHomo)::position" )
    ,articularJacobianSIN( NULL,"sotFeaturePoint6d("+name+")::input(matrix)::Jq" )
{
  jacobianSOUT.addDependency( positionSIN );
  jacobianSOUT.addDependency( articularJacobianSIN );

  errorSOUT.addDependency( positionSIN );

  signalRegistration( positionSIN<<articularJacobianSIN );

  // Commands
  //
  {
    using namespace dynamicgraph::command;
    std::string docstring;
    // Set computation frame
    docstring = "    \n"
      "    Set computation frame\n"
      "    \n"
      "      Input:\n"
      "        a string: 'current' or 'desired'\n"
      "    \n";
    addCommand("frame",
	       new dynamicgraph::command::Setter<FeaturePoint6d, std::string>
	       (*this, &FeaturePoint6d::computationFrame, docstring));
    addCommand("getFrame",
	       new dynamicgraph::command::Getter<FeaturePoint6d, std::string>
	       (*this, &FeaturePoint6d::computationFrame, docstring));
    addCommand("keep",
	       makeCommandVoid0(*this,&FeaturePoint6d::servoCurrentPosition,
				docCommandVoid0("modify the desired position to servo at current pos.")));
  }
}

void FeaturePoint6d::
addDependenciesFromReference( void )
{
  assert( isReferenceSet() );
  errorSOUT.addDependency( getReference()->positionSIN );
  jacobianSOUT.addDependency( getReference()->positionSIN );
}

void FeaturePoint6d::
removeDependenciesFromReference( void )
{
  assert( isReferenceSet() );
  errorSOUT.removeDependency( getReference()->positionSIN );
  jacobianSOUT.removeDependency( getReference()->positionSIN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
void FeaturePoint6d::
computationFrame(const std::string& inFrame)
{
  if (inFrame == "current")
    computationFrame_ = FRAME_CURRENT;
  else if (inFrame == "desired")
    computationFrame_ = FRAME_DESIRED;
  else {
    std::string msg("FeaturePoint6d::computationFrame: "
		    + inFrame + ": invalid argument,\n"
		    "expecting 'current' or 'desired'");
    throw ExceptionFeature(ExceptionFeature::GENERIC, msg);
  }
}

/// \brief Get computation frame
std::string FeaturePoint6d::
computationFrame() const 
{
  switch(computationFrame_)
    {
    case FRAME_CURRENT:
      return "current";
    case FRAME_DESIRED:
      return "desired";
    }
  assert( false&&"Case not handled" );
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeaturePoint6d::
getDimension( unsigned int & dim, int time )
{
  sotDEBUG(25)<<"# In {"<<endl;

  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  for( int i=0;i<6;++i ) if( fl(i) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}


/** Compute the interaction matrix from a subset of
 * the possible features.
 */
ml::Matrix& FeaturePoint6d::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const ml::Matrix & Jq = articularJacobianSIN(time);
  const int & dim = dimensionSOUT(time);
  const Flags &fl = selectionSIN(time);

  sotDEBUG(25)<<"dim = "<<dimensionSOUT(time)<<" time:" << time << " "
              << dimensionSOUT.getTime() << " " << dimensionSOUT.getReady() << endl;
  sotDEBUG(25)<<"selec = "<<selectionSIN(time)<<" time:" << time << " "
              << selectionSIN.getTime() << " " << selectionSIN.getReady() << endl;

  sotDEBUG(15) <<"Dimension="<<dim<<std::endl;

  const unsigned int cJ = Jq.nbCols();
  J.resize(dim,cJ) ;
  ml::Matrix LJq(6,cJ);

  if( FRAME_CURRENT==computationFrame_ )
    {
      /* The Jacobian on rotation is equal to Jr = - hdRh Jr6d.
       * The Jacobian in translation is equalt to Jt = [hRw(wthd-wth)]x Jr - Jt. */
      const MatrixHomogeneous& wMh = positionSIN(time);
      MatrixRotation wRh;      wMh.extract(wRh);
      MatrixRotation wRhd;
      ml::Vector hdth(3),Rhdth(3);

      if( isReferenceSet() )
        {
          const MatrixHomogeneous& wMhd = getReference()->positionSIN(time);
          wMhd.extract(wRhd);
          for( unsigned int i=0;i<3;++i ) hdth(i)=wMhd(i,3)-wMh(i,3);
        }
      else
        {
          wRhd.setIdentity();
          for( unsigned int i=0;i<3;++i ) hdth(i)=-wMh(i,3);
        }
      wRh.inverse().multiply(hdth,Rhdth);
      MatrixRotation hdRh; wRhd.inverse().multiply(wRh,hdRh);

      ml::Matrix Lx(6,6);
      for(unsigned int i=0;i<3;i++)
	{for(unsigned int j=0;j<3;j++)
            {
              if( i==j) { Lx(i,j)=-1; } else { Lx(i,j)=0; }
              Lx(i+3,j)=0;
              Lx(i+3,j+3)=-hdRh(i,j);
            }
        }
      const double & X=Rhdth(0), &Y=Rhdth(1), &Z=Rhdth(2);
      Lx(0,4) = -Z ;      Lx(0,5) =  Y ;       Lx(1,3) = Z ;
      Lx(1,5) = -X ;      Lx(2,3) = -Y ;       Lx(2,4) = X ;
      Lx(0,3) =  0 ;      Lx(1,4) =  0 ;       Lx(2,5) = 0 ;
      sotDEBUG(15) << "Lx= "<<Lx<<endl;

      Lx.multiply(Jq,LJq);
    }
  else
    {
      /* The Jacobian in rotation is equal to Jr = hdJ = hdRh Jr.
       * The Jacobian in translation is equal to Jr = hdJ = hdRh Jr. */
      const MatrixHomogeneous& wMh = positionSIN(time);
      MatrixRotation wRh; wMh.extract(wRh);
      MatrixRotation hdRh;

      if( isReferenceSet() )
        {
          const MatrixHomogeneous& wMhd = getReference()->positionSIN(time);
          MatrixRotation wRhd; wMhd.extract(wRhd);
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
	MatrixHomogeneous hMw; wMh.inverse(hMw);        \
	sotDEBUG(15)<<"hMw = "<<hMw<<endl;                 \
	hMw.multiply( wMhd,hMhd );                         \
	sotDEBUG(15)<<"hMhd = "<<hMhd<<endl;               \
      }


/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
FeaturePoint6d::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

  const Flags &fl = selectionSIN(time);
  const MatrixHomogeneous& wMh = positionSIN(time);
  sotDEBUG(15)<<"wMh = "<<wMh<<endl;

  /* Computing only translation:                                        *
   * trans( hMw wMhd ) = htw + hRw wthd                                 *
   *                   = -hRw wth + hrW wthd                            *
   *                   = hRw ( wthd - wth )                             *
   * The second line is obtained by writting hMw as the inverse of wMh. */

  MatrixHomogeneous hMhd;
  if(isReferenceSet())
    {
      const MatrixHomogeneous& wMhd = getReference()->positionSIN(time);
      sotDEBUG(15)<<"wMhd = "<<wMhd<<endl;
      switch(computationFrame_)
        {
        case FRAME_CURRENT:
          SOT_COMPUTE_H1MH2(wMh,wMhd,hMhd); break;
        case FRAME_DESIRED:
          SOT_COMPUTE_H1MH2(wMhd,wMh,hMhd); break; // Compute hdMh indeed.
        };
    }
  else
    {
      switch(computationFrame_)
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
      MatrixRotation hRhd; hMhd.extract( hRhd );
      VectorUTheta hrhd; hrhd.fromMatrix( hRhd );
      for( unsigned int i=0;i<3;++i )
        { if( fl(i+3) ) error(cursor++) = hrhd(i); }
    }

  sotDEBUGOUT(15);
  return error ;
}

/* Modify the value of the reference (sdes) so that it corresponds
 * to the current position. The effect on the servo is to maintain the
 * current position and correct any drift. */
void FeaturePoint6d::
servoCurrentPosition( void )
{
  sotDEBUGIN(15);

  if(! isReferenceSet() )
    {
      sotERROR << "The reference is not set, this function should not be called" <<std::endl;
      throw ExceptionFeature(ExceptionFeature::GENERIC,
			     "The reference is not set, this function should not be called");
    }
  getReference()->positionSIN = positionSIN.accessCopy();

  sotDEBUGOUT(15);
}

static const char * featureNames  []
= { "X ",
    "Y ",
    "Z ",
    "RX",
    "RY",
    "RZ"  };
void FeaturePoint6d::
display( std::ostream& os ) const
{
  os <<"Point6d <"<<name<<">: (" ;

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



void FeaturePoint6d::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "FeaturePoint: "<<endl
	 << "  - frame [{desired|current}]: get/set the computation frame."<<endl
	 << "  - keep: modify the desired position to servo at current pos."<<endl;
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="frame" )
    {
      cmdArgs>>std::ws;
      if(cmdArgs.good())
	{
	  std::string frameStr; cmdArgs >> frameStr;
	  if( frameStr =="current" ) { computationFrame_ = FRAME_CURRENT; }
	  else { computationFrame_ = FRAME_DESIRED; }
	}
      else {
	os << "frame = ";
	if( FRAME_DESIRED==computationFrame_ ) os << "desired" << std::endl;
	else if( FRAME_CURRENT==computationFrame_ ) os << "current" << std::endl;
      }
    }
  else if( cmdLine=="keep" )
    {
      servoCurrentPosition();
    }
  else  //FeatureAbstract::
    Entity::commandLine( cmdLine,cmdArgs,os );

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

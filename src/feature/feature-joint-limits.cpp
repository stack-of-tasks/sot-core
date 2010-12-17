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
#include <sot-core/feature-joint-limits.h>
#include <sot-core/exception-feature.h>
#include <sot-core/debug.h>
using namespace std;

#include <sot-core/factory.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace sot;
SOT_FACTORY_FEATURE_PLUGIN(FeatureJointLimits,"FeatureJointLimits");

const double FeatureJointLimits::THRESHOLD_DEFAULT = .9;


FeatureJointLimits::
FeatureJointLimits( const string& fName )
  : FeatureAbstract( fName )
    ,threshold(THRESHOLD_DEFAULT)
//     ,freeFloatingIndex( FREE_FLOATING_INDEX )
//     ,freeFloatingSize( FREE_FLOATING_SIZE )

    ,jointSIN( NULL,"sotFeatureJointLimits("+name+")::input(vector)::joint" )
    ,upperJlSIN( NULL,"sotFeatureJointLimits("+name+")::input(vector)::upperJl" )
    ,lowerJlSIN( NULL,"sotFeatureJointLimits("+name+")::input(vector)::lowerJl" )
    ,widthJlSINTERN( boost::bind(&FeatureJointLimits::computeWidthJl,this,_1,_2),
		     upperJlSIN<<lowerJlSIN,
		     "sotFeatureJointLimits("+name+")::input(vector)::widthJl" )
{
  errorSOUT.addDependency( jointSIN );
  errorSOUT.addDependency( upperJlSIN );
  errorSOUT.addDependency( lowerJlSIN );

  activationSOUT.addDependency( jointSIN );
  activationSOUT.addDependency( upperJlSIN );
  activationSOUT.addDependency( lowerJlSIN );

  //jacobianSOUT.addDependency( jointSIN );

  errorSOUT.removeDependency( desiredValueSIN );
  jacobianSOUT.removeDependency( desiredValueSIN );
  activationSOUT.removeDependency( desiredValueSIN );

  signalRegistration( jointSIN<<upperJlSIN<<lowerJlSIN<<widthJlSINTERN );
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

unsigned int& FeatureJointLimits::
getDimension( unsigned int & dim, int time ) 
{
  sotDEBUG(25)<<"# In {"<<endl;

  const Flags &fl = selectionSIN.access(time);
  const unsigned int NBJL  = upperJlSIN.access(time).size();

  dim = 0;
  for( unsigned int i=0;i<NBJL;++i )
    if( fl(i) ) dim++;

  sotDEBUG(25)<<"# Out }"<<endl;
  return dim;
}

ml::Vector&
FeatureJointLimits::computeWidthJl( ml::Vector& res,const int& time )
{
  sotDEBUGIN(15);

  const ml::Vector UJL = upperJlSIN.access(time);
  const ml::Vector LJL = lowerJlSIN.access(time);
  const unsigned int SIZE=UJL.size();
  res.resize(SIZE);

  for( unsigned int i=0;i<SIZE;++i )
    {      res(i)=UJL(i)-LJL(i);    }

  sotDEBUGOUT(15);
  return res;
}

/** Compute the interaction matrix from a subset of
 * the possible features. 
 */
ml::Matrix& FeatureJointLimits::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15)<<"# In {"<<endl;

  const unsigned int SIZE=dimensionSOUT.access(time);
  const ml::Vector q = jointSIN.access(time);
  const Flags &fl = selectionSIN(time);
  //const unsigned int SIZE_FF=SIZE+freeFloatingSize;
  const unsigned int SIZE_TOTAL=q.size();
  const ml::Vector WJL = widthJlSINTERN.access(time);
  J.resize( SIZE,SIZE_TOTAL ); J.fill(0.);
 
  unsigned int idx=0;
  for( unsigned int i=0;i<SIZE_TOTAL;++i ) 
    {
      if( fl(i) ) 
	{
	  if( fabs(WJL(i))>1e-3 ) J(idx,i)=1/WJL(i); 
	  else J(idx,i)=1.;
	  idx++;
	}
    }
//   if( 0!=freeFloatingIndex ) 
//     for( unsigned int i=0;i<freeFloatingIndex;++i ) 
//       {
// 	if( fabs(WJL(i))>1e-3 ) J(i,i)=1/WJL(i); else J(i,i)=1.;
//       }

//   if( SIZE!=freeFloatingIndex ) 
//     for( unsigned int i=freeFloatingIndex;i<SIZE;++i ) 
//       {
// 	if( fabs(WJL(i))>1e-3 ) J(i,i+freeFloatingSIZE)=1/WJL(i); 
// 	else J(i,i)=1.;
//       }

  sotDEBUG(15)<<"# Out }"<<endl;
  return J;
}

/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
FeatureJointLimits::computeError( ml::Vector& error,int time )
{
  sotDEBUGIN(15);

  const Flags &fl = selectionSIN(time);
  const ml::Vector q = jointSIN.access(time);
  const ml::Vector UJL = upperJlSIN.access(time);
  const ml::Vector LJL = lowerJlSIN.access(time);
  const ml::Vector WJL = widthJlSINTERN.access(time);
  const unsigned int SIZE=dimensionSOUT.access(time);
  const unsigned int SIZE_TOTAL=q.size();

  sotDEBUG(25) << "q = " << q << endl;
  sotDEBUG(25) << "ljl = " << LJL << endl;
  sotDEBUG(25) << "Wjl = " << WJL << endl;
  sotDEBUG(25) << "dim = " << SIZE << endl;

  assert( UJL.size() == SIZE_TOTAL );
  assert( WJL.size() == SIZE_TOTAL );
  assert( LJL.size() == SIZE_TOTAL );
  assert( SIZE <= SIZE_TOTAL );

  error.resize(SIZE);
  
  unsigned int parcerr = 0;
  for( unsigned int i=0;i<SIZE_TOTAL;++i )
    {
      if( fl(i) )
	{	  error(parcerr++) = (q(i)-LJL(i))/WJL(i)*2-1;	}
    }

  sotDEBUGOUT(15);
  return error ;
}


/** Compute the error between two visual features from a subset
 * a the possible features.
 */
ml::Vector&
FeatureJointLimits::computeActivation( ml::Vector& act,int time )
{
  const ml::Vector err = errorSOUT.access(time);
  //const Flags &fl = selectionSIN(time);
  const unsigned int SIZE=dimensionSOUT.access(time);
  //const unsigned int SIZE_TOTAL=jointSIN.access(time).size();
  act.resize(SIZE);

  //unsigned int parcact = 0;
  for( unsigned int i=0;i<SIZE;++i )
    {
      //if( fl(i) )
      //{
	  const double x = fabs(err(i));
	  if( x<threshold ) act(i)=0.;
	  else if( x>1. ) act(i) = 1.;
	  else act(i) = (x-threshold)/(1-threshold);
	  //parcact++;
	  //}
    }
  return act ;
}


void FeatureJointLimits::
display( std::ostream& os ) const
{
  

  os <<"JointLimits <"<<name<<"> ... TODO";

}




void FeatureJointLimits::
commandLine( const std::string& cmdLine,
	     std::istringstream& cmdArgs,
	     std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "FeatureJointLimits:"<<endl
	 <<"  - setThreshold  <int> " <<endl
	 <<"  - getThreshold  " <<endl;
    }
  else if( cmdLine == "setThreshold" )
    {
      double th;
      cmdArgs >> th; if( th<0. ) th=0.;if( th>1. ) th=1.;
      threshold = th;
    }
  else if( cmdLine == "getThreshold" )
    {
      os << "threshold = " << threshold <<endl; 
    }
  else if( cmdLine == "actuate" )
    {
      Flags fl( 63 );
      selectionSIN =  (! fl);
    }
  else { FeatureAbstract::commandLine( cmdLine,cmdArgs,os ); }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

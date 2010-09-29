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

#include <sot-core/pool.h>
#include <sot-core/signal-cast.h>
#include <iomanip>
#include <sot-core/feature-abstract.h>

using namespace std;
using namespace sot;

SignalCast_sotFeatureAbstractPtr SignalCast<SignalCast_sotFeatureAbstractPtr>::
cast( std::istringstream& iss )
{
  SignalCast_sotFeatureAbstractPtr ref; 
  std::string name; iss >> name; 
  if( name.length())
    {
      ref = &sotPool.getFeature(name); 
    }
  else { ref = NULL; }
  return ref;
}

void SignalCast<SignalCast_sotFeatureAbstractPtr>::
disp(  const SignalCast_sotFeatureAbstractPtr & t,std::ostream& os )
{
  if( t ) { t->display(os); os<<std::endl; }
  else { os << "NULL" << std::endl; }
}



struct timeval SignalCast<struct timeval>::
cast( std::istringstream& iss )
{
  int u,s; iss >> s >> u;
  struct timeval t; t.tv_sec = s; t.tv_usec = u;
  return t;
}
void SignalCast<struct timeval>::
disp( const struct timeval& t,std::ostream& os )
{
  os << t.tv_sec << "s "<< t.tv_usec << "ms";
}

void SignalCast<VectorMultiBound>::
trace( const VectorMultiBound& t,std::ostream& os )
{
  for( VectorMultiBound::const_iterator iter=t.begin();t.end()!=iter;++iter )
    {
      switch( iter->mode )
        {
        case MultiBound::MODE_SINGLE:
          os << iter->getSingleBound() << "\t";
          break;
        case MultiBound::MODE_DOUBLE:
          if( iter->getDoubleBoundSetup(MultiBound::BOUND_INF) )
            os << iter->getDoubleBound(MultiBound::BOUND_INF)<<"\t";
          else os <<"-inf\t";
          if( iter->getDoubleBoundSetup(MultiBound::BOUND_SUP) )
            os << iter->getDoubleBound(MultiBound::BOUND_SUP)<<"\t";
          else os <<"+inf\t";
          break;
        }
    }
}

/* --- VECTOR ---------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */

maal::boost::DisplayType SignalCast<ml::Vector>::displayType = maal::boost::MATLAB;

ml::Vector SignalCast<ml::Vector>::
cast( std::istringstream& iss )
{
   ml::Vector res; 
   iss >> res; 
   return res;
}
void SignalCast<ml::Vector>::
disp( const ml::Vector& v1,std::ostream& os )
{
  switch( displayType )
    {
    case maal::boost::COMPLET:
      for( unsigned int i=0;i<v1.size();++i ) 
	{ if(fabs(v1(i))>1e-6) os << v1(i) << "\t"; else os<<0.00<<"\t"; } 
      break;
    case maal::boost::MATLAB:
      os << "[ ";
      for( unsigned int i=0;i<v1.size();++i )
	{
	  os <<  v1(i);
	  if( v1.size()!=i+1 ) { os << ", "; }
	}
      os << "]" << std::endl; 
      break;
    case maal::boost::SIMPLE:
    default:
      os<<v1.accessToMotherLib() << std::endl;
      break;
    }
}

void SignalCast<ml::Vector>::
trace( const ml::Vector& t,std::ostream& os )

{
  for( unsigned int i=0;i<t.size();++i ) 
    { os << std::setprecision(9)<<t(i) << "\t"; } 
}




/* --- MATRIX --------------------------------------------------- */
/* --- MATRIX --------------------------------------------------- */
/* --- MATRIX --------------------------------------------------- */

ml::Matrix SignalCast<ml::Matrix>::
cast( std::istringstream& iss )
{
   ml::Matrix res; 
   iss >> res; 
   return res;
}

void SignalCast<ml::Matrix>::
disp( const ml::Matrix& m1,std::ostream& os )
{
  switch( SignalCast<ml::Vector>::displayType)
    {
    case ml::COMPLET:
      for( unsigned int i=0;i<m1.nbRows();++i ) 
	{
	  for( unsigned int j=0;j<m1.nbCols();++j )
	    { if(fabs(m1(i,j))>1e-6) os << m1(i,j) << "\t"; else os<<0.00<<"\t"; } 
	  os<<std::endl;
	}
      break;
    case ml::MATLAB:
      os << "[ ";
      for( unsigned int i=0;i<m1.nbRows();++i )
	{
	  for( unsigned int j=0;j<m1.nbCols();++j )
	    {
	      os <<  m1(i,j) << ", ";
	    }
	  if( m1.nbRows()!=i+1 ) { os << ";" << std::endl; }
	  else { os << "]" << std::endl; }
	}
      break;
    case ml::SIMPLE:
    default:
      os<<m1.matrix<<std::endl; 
      break;
    }   
}

void SignalCast<ml::Matrix>::
trace( const ml::Matrix& t,std::ostream& os )
{  
   for( unsigned int i=0;i<t.nbRows();++i ) 
     for( unsigned int j=0;j<t.nbCols();++j )
       { if(fabs(t(i,j))>1e-6) os << t(i,j) << "\t"; else os<<0.00<<"\t"; } 
}

namespace {
	SOT_SIGNAL_CAST_DECLARATION(SignalCast_sotFeatureAbstractPtr);
	SOT_SIGNAL_CAST_DECLARATION(Flags);
	SOT_SIGNAL_CAST_DECLARATION(VectorMultiBound);
	SOT_SIGNAL_CAST_DECLARATION(timeval );
	SOT_SIGNAL_CAST_DECLARATION_NAMED(maal::boost::Vector, maal_boost_Vector);
	SOT_SIGNAL_CAST_DECLARATION_NAMED(maal::boost::Matrix, maal_boost_Matrix);
	SOT_SIGNAL_CAST_DECLARATION(VectorUTheta);
	SOT_SIGNAL_CAST_DECLARATION(VectorQuaternion);
	SOT_SIGNAL_CAST_DECLARATION(VectorRollPitchYaw);
	SOT_SIGNAL_CAST_DECLARATION(MatrixRotation);
	SOT_SIGNAL_CAST_DECLARATION(MatrixHomogeneous);
	SOT_SIGNAL_CAST_DECLARATION(MatrixTwist);
	SOT_SIGNAL_CAST_DECLARATION(MatrixForce);

}

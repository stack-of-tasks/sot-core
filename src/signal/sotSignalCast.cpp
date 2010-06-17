/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      SignalCast.cpp
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

#include <sot-core/sotPool.h>
#include <sot-core/sotSignalCast.h>
#include <iomanip>
#include <sot-core/sotFeatureAbstract.h>

using namespace std;

#ifdef WIN32
#include < Windows.h >
#endif

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

void SignalCast<sotVectorMultiBound>::
trace( const sotVectorMultiBound& t,std::ostream& os )
{
  for( sotVectorMultiBound::const_iterator iter=t.begin();t.end()!=iter;++iter )
    {
      switch( iter->mode )
        {
        case sotMultiBound::MODE_SINGLE:
          os << iter->getSingleBound() << "\t";
          break;
        case sotMultiBound::MODE_DOUBLE:
          if( iter->getDoubleBoundSetup(sotMultiBound::BOUND_INF) )
            os << iter->getDoubleBound(sotMultiBound::BOUND_INF)<<"\t";
          else os <<"-inf\t";
          if( iter->getDoubleBoundSetup(sotMultiBound::BOUND_SUP) )
            os << iter->getDoubleBound(sotMultiBound::BOUND_SUP)<<"\t";
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
	SOT_SIGNAL_CAST_DECLARATION(sotFlags);
	SOT_SIGNAL_CAST_DECLARATION(sotVectorMultiBound);
	SOT_SIGNAL_CAST_DECLARATION(timeval );
	SOT_SIGNAL_CAST_DECLARATION_NAMED(maal::boost::Vector, maal_boost_Vector);
	SOT_SIGNAL_CAST_DECLARATION_NAMED(maal::boost::Matrix, maal_boost_Matrix);
	SOT_SIGNAL_CAST_DECLARATION(sotVectorUTheta);
	SOT_SIGNAL_CAST_DECLARATION(sotVectorQuaternion);
	SOT_SIGNAL_CAST_DECLARATION(sotVectorRollPitchYaw);
	SOT_SIGNAL_CAST_DECLARATION(sotMatrixRotation);
	SOT_SIGNAL_CAST_DECLARATION(sotMatrixHomogeneous);
	SOT_SIGNAL_CAST_DECLARATION(sotMatrixTwist);
	SOT_SIGNAL_CAST_DECLARATION(sotMatrixForce);

}

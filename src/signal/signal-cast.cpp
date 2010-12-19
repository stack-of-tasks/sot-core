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

#include <dynamic-graph/signal-caster.h>
#include <sot-core/pool.h>
#include <sot-core/signal-cast.h>
#include <iomanip>
#include <sot-core/feature-abstract.h>

#include "sot-core/matrix-homogeneous.h"

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

namespace {
	SOT_SIGNAL_CAST_DECLARATION(SignalCast_sotFeatureAbstractPtr);
	SOT_SIGNAL_CAST_DECLARATION(Flags);
	SOT_SIGNAL_CAST_DECLARATION(VectorMultiBound);
	SOT_SIGNAL_CAST_DECLARATION(timeval );
  dynamicgraph::DefaultCastRegisterer <sot::MatrixHomogeneous>
  matrixHomegeneousCastRegisterer;
}

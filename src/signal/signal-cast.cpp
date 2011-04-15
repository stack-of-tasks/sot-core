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
#include <sot/core/pool.hh>
#include <iomanip>

#include <sot/core/feature-abstract.hh>
#include "sot/core/matrix-homogeneous.hh"
#include "sot/core/matrix-rotation.hh"
#include <sot/core/flags.hh>
#include <sot/core/multi-bound.hh>
#include <dynamic-graph/signal-caster.h>
#include <dynamic-graph/signal-cast-helper.h>

# ifdef WIN32
#  include <Windows.h>
# endif

/* Implements a set of caster/displayer for the main types of sot-core. */


namespace dynamicgraph
{
  using namespace std;
  using namespace dynamicgraph::sot;

  /* --- CASTER IMPLEMENTATION ------------------------------------------------ */
  /* --- CASTER IMPLEMENTATION ------------------------------------------------ */
  /* --- CASTER IMPLEMENTATION ------------------------------------------------ */

  DG_SIGNAL_CAST_DEFINITION(sot::Flags);
  DG_ADD_CASTER(sot::Flags,flags);


  template <>
  void
  DefaultCastRegisterer<MatrixHomogeneous>::
  trace(const boost::any& object, std::ostream& os)
  {
    const MatrixHomogeneous & M = boost::any_cast<MatrixHomogeneous>(object);
    for( unsigned int i=0;i<4;++i )
      for( unsigned int j=0;j<4;++j )
	{ os << "\t" << M(i,j); }
  }
  namespace {
    dynamicgraph::DefaultCastRegisterer <sot::MatrixHomogeneous>
    matrixHomegeneousCastRegisterer;

    dynamicgraph::DefaultCastRegisterer <sot::MatrixRotation>
    matrixRotationCastRegisterer;
  }
  /* --- FEATURE ABSTRACT ----------------------------------------------------- */
  /* --- FEATURE ABSTRACT ----------------------------------------------------- */
  /* --- FEATURE ABSTRACT ----------------------------------------------------- */

  DG_SIGNAL_CAST_DEFINITION_HPP(sot::FeatureAbstract*);

  FeatureAbstract* SignalCast<FeatureAbstract*>::
  cast( std::istringstream& iss )
  {
    FeatureAbstract* ref;
    std::string name; iss >> name;
    if( name.length())
      {
	ref = &dynamicgraph::sot::PoolStorage::getInstance()->getFeature(name);
      }
    else { ref = NULL; }
    return ref;
  }

  void SignalCast<FeatureAbstract*>::
  disp( FeatureAbstract* const& t,std::ostream& os )
  {
    if( t ) { t->display(os); os<<std::endl; }
    else { os << "NULL" << std::endl; }
  }

  DG_ADD_CASTER(FeatureAbstract*,FAptr);


  /* --- TIMEVAL -------------------------------------------------------------- */
  /* --- TIMEVAL -------------------------------------------------------------- */
  /* --- TIMEVAL -------------------------------------------------------------- */
  DG_SIGNAL_CAST_DEFINITION_HPP(struct timeval);

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

  DG_ADD_CASTER(struct timeval,tv);

  /* --- MULTI BOUND ---------------------------------------------------------- */
  /* --- MULTI BOUND ---------------------------------------------------------- */
  /* --- MULTI BOUND ---------------------------------------------------------- */

  DG_SIGNAL_CAST_DEFINITION_TRACE(sot::VectorMultiBound);

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
  DG_ADD_CASTER(sot::VectorMultiBound,sotVMB);


}// namespace dynamicgraph

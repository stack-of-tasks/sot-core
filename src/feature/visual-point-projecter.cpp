/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
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

#include <sot/core/visual-point-projecter.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

namespace dynamicgraph
{
  namespace sot
  {

      namespace dg = ::dynamicgraph;
      using namespace dg;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VisualPointProjecter,"VisualPointProjecter");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
    VisualPointProjecter::
      VisualPointProjecter( const std::string & name )
	: Entity(name)

	,CONSTRUCT_SIGNAL_IN(point3D,dynamicgraph::Vector)
	,CONSTRUCT_SIGNAL_IN(transfo,MatrixHomogeneous)

	,CONSTRUCT_SIGNAL_OUT(point3Dgaze,dynamicgraph::Vector,m_point3DSIN<<m_transfoSIN )
	,CONSTRUCT_SIGNAL_OUT(depth,double,m_point3DgazeSOUT )
	,CONSTRUCT_SIGNAL_OUT(point2D,dynamicgraph::Vector,m_point3DgazeSOUT<<m_depthSOUT )
      {
	Entity::signalRegistration( m_point3DSIN );
	Entity::signalRegistration( m_transfoSIN );
	Entity::signalRegistration( m_point3DgazeSOUT );
	Entity::signalRegistration( m_point2DSOUT );
	Entity::signalRegistration( m_depthSOUT );
      }


      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      dynamicgraph::Vector& VisualPointProjecter::
      point3DgazeSOUT_function( dynamicgraph::Vector &p3g, int iter )
      {
	const dynamicgraph::Vector & p3 = m_point3DSIN(iter);
	const MatrixHomogeneous & M = m_transfoSIN(iter);
	MatrixHomogeneous Mi; Mi = M.inverse(Eigen::Affine);
	p3g = Mi.matrix()*p3;
	return p3g;
      }

      dynamicgraph::Vector& VisualPointProjecter::
      point2DSOUT_function( dynamicgraph::Vector &p2, int iter )
      {
	sotDEBUGIN(15);

	const dynamicgraph::Vector & p3 = m_point3DgazeSOUT(iter);
	const double &z =m_depthSOUT(iter);
	assert(z>0);

	p2.resize(2);
	p2(0) = p3(0) / z;
	p2(1) = p3(1) / z; 

	sotDEBUGOUT(15);
	return p2;
      }

      double& VisualPointProjecter::
      depthSOUT_function( double & z, int iter )
      {
	const dynamicgraph::Vector & p3 = m_point3DgazeSOUT(iter);
	assert(p3.size() == 3);
        z=p3(2);
	return z;
      }

      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */
      /* --- ENTITY ----------------------------------------------------------- */

      void VisualPointProjecter::
      display( std::ostream& os ) const
      {
	os << "VisualPointProjecter "<<getName();
      }

  } // namespace sot
} // namespace dynamicgraph


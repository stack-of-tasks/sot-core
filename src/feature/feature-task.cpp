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
#include <sot/core/feature-task.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/task.hh>
#include <dynamic-graph/pool.h>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureTask,"FeatureTask");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



FeatureTask::
FeatureTask( const string& pointName )
  : FeatureGeneric( pointName )
{
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void FeatureTask::
display( std::ostream& os ) const
{
  os << "Feature from task <" << getName();
  if( taskPtr ) os <<  ": from task " << taskPtr->getName();
  os << std::endl;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

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

#include <sot/core/feature-abstract.hh>
#include <sot/core/pool.hh>
#include <dynamic-graph/all-commands.h>

using namespace dynamicgraph::sot;
namespace dg = dynamicgraph;

const std::string 
FeatureAbstract::CLASS_NAME = "FeatureAbstract";


FeatureAbstract::
FeatureAbstract( const std::string& name ) 
  :Entity(name)
   ,selectionSIN(NULL,"sotFeatureAbstract("+name+")::input(flag)::selec")
   ,errorSOUT( boost::bind(&FeatureAbstract::computeError,this,_1,_2),
	       selectionSIN,
	       "sotFeatureAbstract("+name+")::output(vector)::error" )
   ,jacobianSOUT( boost::bind(&FeatureAbstract::computeJacobian,this,_1,_2),
		  selectionSIN,
		  "sotFeatureAbstract("+name+")::output(matrix)::jacobian" )
   ,dimensionSOUT( boost::bind(&FeatureAbstract::getDimension,this,_1,_2),
		   selectionSIN,
		   "sotFeatureAbstract("+name+")::output(uint)::dim" )
{
  selectionSIN = true;
  signalRegistration( selectionSIN
		      <<errorSOUT<<jacobianSOUT<<dimensionSOUT );
  featureRegistration();
  initCommands();
}

void FeatureAbstract::
initCommands( void )
{
  using namespace command;
  addCommand("setReference",
	     new dynamicgraph::command::Setter<FeatureAbstract, std::string>
	     (*this, &FeatureAbstract::setReferenceByName,
	      "Give the name of the reference feature.\nInput: a string (feature name)."));
  addCommand("getReference",
	     new dynamicgraph::command::Getter<FeatureAbstract, std::string>
	     (*this, &FeatureAbstract::getReferenceByName,
	      "Get the name of the reference feature.\nOutput: a string (feature name)."));
}

void FeatureAbstract::
featureRegistration( void )
{
  PoolStorage::getInstance()->registerFeature(name,this);
}

std::ostream& FeatureAbstract::
writeGraph( std::ostream& os ) const
{
  Entity::writeGraph(os);

  if( isReferenceSet() )
    {
      const FeatureAbstract *asotFA = getReferenceAbstract();
      os << "\t\"" << asotFA->getName() << "\" -> \"" << getName() << "\""
	 << "[ color=darkseagreen4 ]" << std::endl;
    }
  else std::cout << "asotFAT : 0" << std::endl;

  return os;
}

void FeatureAbstract::
setReferenceByName( const std::string& name )
{
  setReference( &dynamicgraph::sot::PoolStorage::getInstance()->getFeature(name));
}

std::string FeatureAbstract::
getReferenceByName() const
{
  if( isReferenceSet() ) return getReferenceAbstract()->getName(); else return "none";
}

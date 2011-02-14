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

using namespace dynamicgraph::sot;
namespace dg = dynamicgraph;

const std::string 
FeatureAbstract::CLASS_NAME = "FeatureAbstract";


FeatureAbstract::
FeatureAbstract( const std::string& name ) 
  :Entity(name)
   ,desiredValueSIN(NULL,"sotFeatureAbstract("+name+")::input(feature)::sdes")
   ,selectionSIN(NULL,"sotFeatureAbstract("+name+")::input(flag)::selec")
   ,errorSOUT( boost::bind(&FeatureAbstract::computeError,this,_1,_2),
	       selectionSIN<<desiredValueSIN,
	       "sotFeatureAbstract("+name+")::output(vector)::error" )
   ,jacobianSOUT( boost::bind(&FeatureAbstract::computeJacobian,this,_1,_2),
		  selectionSIN,
		  "sotFeatureAbstract("+name+")::output(matrix)::jacobian" )
   ,activationSOUT( boost::bind(&FeatureAbstract::computeActivation,this,_1,_2),
		    selectionSIN<<desiredValueSIN,
		    "sotFeatureAbstract("+name+")::output(vector)::activation" )
   ,dimensionSOUT( boost::bind(&FeatureAbstract::getDimension,this,_1,_2),
		   selectionSIN,
		   "sotFeatureAbstract("+name+")::output(uint)::dim" )
{
  selectionSIN = true;
  signalRegistration( desiredValueSIN<<selectionSIN
		      <<errorSOUT<<jacobianSOUT<<activationSOUT<<dimensionSOUT );
  featureRegistration();

}


void FeatureAbstract::
featureRegistration( void )
{
  sotPool.registerFeature(name,this);
}


std::ostream& FeatureAbstract::
writeGraph( std::ostream& os ) const
{
  Entity::writeGraph(os);

  if( desiredValueSIN )
    {
      //      const SignalAbstract<int> & sdesAbs = desiredValueSIN;
      const dg::SignalPtr<FeatureAbstract *,int>  & sdesSig = desiredValueSIN;
      
      if (sdesSig!=0)
	{
	  FeatureAbstract *asotFA = sdesSig.accessCopy();
	  if (asotFA!=0)
	    {
	      os << "\t\"" << asotFA->getName() << "\" -> \"" << getName() << "\""
		 << "[ color=darkseagreen4 ]" << std::endl;
	    }
	  else std::cout << "asotFAT : 0" << std::endl;
	}
      else std::cout << "sdesSig : 0" << std::endl;
    }

  return os;
}

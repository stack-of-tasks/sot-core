/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-abstract.h
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

#include <sot-core/feature-abstract.h>
#include <sot-core/pool.h>

using namespace sot;
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

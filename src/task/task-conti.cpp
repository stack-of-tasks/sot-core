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

/* SOT */
#include <sot/core/task-conti.hh>
#include <sot/core/debug.hh>
#include <jrl/mal/boostvector.hh>
#include <sot/core/factory.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskConti,"TaskConti");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


TaskConti::
TaskConti( const std::string& n )
  :Task(n)
   ,timeRef( TIME_REF_UNSIGNIFICANT )
   ,mu(0)
   ,controlPrevSIN( NULL,"sotTaskConti("+n+")::input(double)::q0" )
{
  taskSOUT.setFunction( boost::bind(&TaskConti::computeContiDesiredVelocity,this,_1,_2) );
  signalRegistration( controlPrevSIN );
}


VectorMultiBound& TaskConti::
computeContiDesiredVelocity( VectorMultiBound& desvel2b,const int & timecurr )
{
  sotDEBUG(15) << "# In {" << endl;

  ml::Vector desvel = errorSOUT(timecurr);
  const double &  lambda = controlGainSIN(timecurr);

  try{
    const ml::Matrix & J = jacobianSOUT(timecurr);

    ml::Vector deref( J.nbRows() );
    sotDEBUG(15) << "q0 = " << q0 << std::endl;
    sotDEBUG(25) << "J = " << J << std::endl;
    if( q0.size() != (J.nbCols()-6) ) throw; // TODO
    for( unsigned int i=0;i<J.nbRows();++i )
      {
	deref(i)=0;
	for( unsigned int j=6;j<J.nbCols();++j )
	  deref(i) += J(i,j)*q0(j-6);
      }
    //J.multiply( q0,deref );

    if( timeRef==TIME_REF_TO_BE_SET ) { timeRef = timecurr; }
    if( timeRef<0 ) { sotDEBUG(10) << "Time not used. " << std::endl; throw 1;}

    double dt = timeRef-timecurr; dt*=mu/200.0;
    double contiGain = exp(dt);
    double gain = (contiGain-1)*lambda;

    sotDEBUG(25) << "T: ref=" << timeRef << ", cur=" << timecurr << std::endl;
    sotDEBUG(25) << "Gains: l=" << lambda << ", expmu=" << contiGain << std::endl;
    sotDEBUG(25) << "e = " << deref<<std::endl;

    desvel *= gain;
    sotDEBUG(25) << "dedes: " << desvel <<std::endl;
    deref *= contiGain;
    desvel += deref;
    sotDEBUG(25) << "task: " << desvel <<std::endl;

    desvel2b.resize(desvel.size());
    for( unsigned int i=0;i<desvel.size(); ++i )
      desvel2b[i] = desvel(i);


    sotDEBUG(15) << "# Out }" << endl;
    return desvel2b;
  } catch (...)
    {
      const ml::Vector & desvel = errorSOUT(timecurr);
      const double & gain = controlGainSIN(timecurr);
      desvel2b.resize(desvel.size());
      for( unsigned int i=0;i<desvel.size(); ++i )
        desvel2b[i] = -gain*desvel(i);
      return desvel2b;
    }

}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void TaskConti::
display( std::ostream& os ) const
{
  os << "TaskConti " << name
     << " [t=" << timeRef << "] "
     << ": " << endl;
  os << "--- LIST ---  " << std::endl;

  for(   std::list< FeatureAbstract* >::const_iterator iter = featureList.begin();
	 iter!=featureList.end(); ++iter )
    {
      os << "-> " << (*iter)->getName() <<endl;
    }

}



/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

#include <dynamic-graph/pool.h>

void TaskConti::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "TaskConti: "<<endl
	 << "  - touch <sot.control>"<<endl
	 << "  - timeRef" << endl
	 << "  - mu [<val>]" << endl;

      Task::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="touch" )
    {
      Signal<ml::Vector,int> & sig
	= dynamic_cast< Signal<ml::Vector,int>& >( g_pool.getSignal( cmdArgs ) );
      timeRef = TIME_REF_TO_BE_SET; //sig.getTime();
      q0 = sig.accessCopy();
    }
  else if( cmdLine=="timeRef" )
    {
      os << "timeRef = ";
      if( timeRef == TIME_REF_TO_BE_SET ) os << "to be set.";
      else if( timeRef == TIME_REF_UNSIGNIFICANT ) os << "no signaificant";
      else os << timeRef;
      os << std::endl;
    }
  else if( cmdLine=="mu" )
    {
      cmdArgs >> std::ws; if(! cmdArgs.good() ) os << "mu = " << mu << std::endl;
      else { cmdArgs >> mu; }
    }
  else  Task::commandLine( cmdLine,cmdArgs,os );

}

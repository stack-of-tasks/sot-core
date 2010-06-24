/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet VISTA / IRISA, 2003
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_categorie.cc
 * Project:   Traces
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id: test_boost.cpp,v 1.1.1.1 2006-07-03 05:17:37 nmansard Exp $
 *
 * Description
 * ============
 *
 * Test la classe CategorieTrace.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <sot-core/sot-h.h>
#include <sot-core/feature-visual-point.h>
#include <sot-core/feature-abstract.h>
#include <sot-core/debug.h>
#include <sot-core/task.h>
#include <sot-core/gain-adaptive.h>

using namespace std;
using namespace sot;

namespace ml = maal::boost;

double drand( void ) { return 2*((double)rand())/RAND_MAX-1; }
ml::Matrix& mrand( ml::Matrix& J )
{ 
  for( unsigned int i=0;i<J.nbRows();++i)
    for( unsigned int j=0;j<J.nbCols();++j)
      J(i,j) = drand();
  return J;
}

int main( void )
{
  sotDEBUGF( "# In {" );

  srand(12);
  ml::Matrix Jq(6,6); Jq.setIdentity();

  ml::Vector p1xy(2); p1xy(0)=1.; p1xy(1)=-2;
  
  sotDEBUGF("Create feature");
  FeatureVisualPoint * p1 = new FeatureVisualPoint("p1");
  FeatureVisualPoint * p1des = new FeatureVisualPoint("p1des");

  
  p1->articularJacobianSIN.setReference(&Jq);
  p1->selectionSIN = Flags(true);
  p1->desiredValueSIN = p1des;
  p1->xySIN = p1xy;

  p1des->xySIN = ml::Vector(2);

  
  sotDEBUGF("Create Task");
  sotDEBUG(0) << ml::MATLAB;

  Task * task = new Task("task");
  task->addFeature(*p1);
  task->addFeature(*p1);

  GainAdaptative * lambda = new GainAdaptative("g");
  lambda->errorSIN.plug( &task->errorSOUT );

  task->controlGainSIN.plug( &lambda->gainSOUT );
  task->dampingGainSINOUT = .1;
  task->controlSelectionSIN = Flags(true);

  task->jacobianSOUT.display(cout)<<endl;
  task->jacobianSOUT.displayDependancies(cout)<<endl;

  sotDEBUG(0) << "J"<< task->jacobianSOUT(2);
  sotDEBUG(0) <<"H"<< task->featureActivationSOUT(2)<<endl;
  sotDEBUG(0) <<"e"<< task->errorSOUT(2) <<endl;


  sotDEBUGF( "# Out }" );

  return 0;
}

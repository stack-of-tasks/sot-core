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

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <sot/core/sot.hh>
#include <sot/core/feature-visual-point.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/debug.hh>
#include <sot/core/task.hh>
#include <sot/core/gain-adaptive.hh>

using namespace std;
using namespace dynamicgraph::sot;

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
  srand(12);
  ml::Matrix Jq(6,6); Jq.setIdentity();

  ml::Vector p1xy(2); p1xy(0)=1.; p1xy(1)=-2;

  sotDEBUGF("Create feature");
  FeatureVisualPoint * p1 = new FeatureVisualPoint("p1");
  FeatureVisualPoint * p1des = new FeatureVisualPoint("p1d");

  p1->articularJacobianSIN.setReference(&Jq);
  p1->selectionSIN = Flags(true);
  p1->desiredValueSIN = p1des;
  p1->xySIN = p1xy;

  p1des->xySIN = ml::Vector(2);


  sotDEBUGF("Create Task");
  sotDEBUG(0) << ml::MATLAB;

  Task * task = new Task("t");
  task->addFeature(*p1);
  task->addFeature(*p1);

  GainAdaptive * lambda = new GainAdaptive("g");
  lambda->errorSIN.plug( &task->errorSOUT );

  task->controlGainSIN.plug( &lambda->gainSOUT );
  task->dampingGainSINOUT = .1;
  task->controlSelectionSIN = Flags(true);

  task->jacobianSOUT.display(cout)<<endl;
  task->jacobianSOUT.displayDependencies(cout)<<endl;

  sotDEBUG(0) << ml::MATLAB << "J"<< task->jacobianSOUT(2);
  sotDEBUG(0) <<"H"<< task->featureActivationSOUT(2)<<endl;
  sotDEBUG(0) <<"e"<< task->errorSOUT(2) <<endl;



  return 0;
}

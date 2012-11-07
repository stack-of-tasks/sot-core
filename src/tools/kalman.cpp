/*
 * Copyright 2010,
 * Nicolas Mansard,
 * François Bleibel,
 * Olivier Stasse,
 * Florent Lamiraux
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

/* --- SOT --- */
#include <sot/core/kalman.hh>          /* Header of the class implemented here.   */
#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <sot/core/factory.hh>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/command-setter.h>

namespace dynamicgraph {
  using command::Setter;
  namespace sot {

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Kalman,"Kalman");

    Kalman::Kalman( const std::string& name )
      : Entity(name)
      ,measureSIN (NULL,"Kalman("+name+")::input(vector)::y")
      ,modelTransitionSIN( NULL,"Kalman("+name+")::input(matrix)::F" )
      ,modelMeasureSIN( NULL,"Kalman("+name+")::input(matrix)::H" )
      ,noiseTransitionSIN( NULL,"Kalman("+name+")::input(matrix)::Q" )
      ,noiseMeasureSIN( NULL,"Kalman("+name+")::input(matrix)::R" )

      ,statePredictedSIN (0, "Kalman("+name+")::input(vector)::x_pred")
      ,observationPredictedSIN (0, "Kalman("+name+")::input(vector)::y_pred")
      ,varianceUpdateSOUT ("Kalman("+name+")::output(vector)::P")
      ,stateUpdateSOUT ("Kalman("+name+")::output(vector)::x_est"),
	stateEstimation_ (),
	stateVariance_ ()
    {
      sotDEBUGIN(15);
      varianceUpdateSOUT.setFunction
	(boost::bind(&Kalman::computeVarianceUpdate,
		     this,_1,_2));
      stateUpdateSOUT.setFunction (boost::bind(&Kalman::computeStateUpdate,
					       this, _1, _2));

      signalRegistration( measureSIN << observationPredictedSIN
			  << modelTransitionSIN
			  << modelMeasureSIN << noiseTransitionSIN
			  << noiseMeasureSIN << statePredictedSIN
			  << stateUpdateSOUT << varianceUpdateSOUT );

      std::string docstring =
	"  Set initial state estimation\n"
	"\n"
	"  input:\n"
	"    - a vector: initial state\n";
      addCommand ("setInitialState",
		  new Setter <Kalman, Vector> (*this,
					       &Kalman::setStateEstimation,
					       docstring));

      docstring =
	"  Set variance of initial state estimation\n"
	"\n"
	"  input:\n"
	"    - a matrix: variance covariance matrix\n";
      addCommand ("setInitialVariance",
		  new Setter <Kalman, Matrix> (*this,
					       &Kalman::setStateVariance,
					       docstring));
      sotDEBUGOUT(15);
    }

    Matrix & Kalman::
    computeVarianceUpdate (Matrix& Pk_k,const int& time)
    {
      sotDEBUGIN(15);
      if (time == 0) {
	// First time return variance initial state
	Pk_k = stateVariance_;
	// Set dependency to input signals for latter computations
	varianceUpdateSOUT.addDependency (noiseTransitionSIN);
	varianceUpdateSOUT.addDependency (modelTransitionSIN);
      } else {

	const Matrix &Q = noiseTransitionSIN( time );
	const Matrix& R = noiseMeasureSIN (time);
	const Matrix &F = modelTransitionSIN( time );
	const Matrix& H = modelMeasureSIN (time);
	const Matrix &Pk_1_k_1 = stateVariance_;

	sotDEBUG(15) << "Q=" << Q << std::endl;
	sotDEBUG(15) << "R=" << R << std::endl;
	sotDEBUG(15) << "F=" << F << std::endl;
	sotDEBUG(15) << "H=" << H << std::endl;
	sotDEBUG(15) << "Pk_1_k_1=" << Pk_1_k_1 << std::endl;

	F.multiply(Pk_1_k_1,FP_);
	FP_.multiply (F.transpose(), Pk_k_1_);
	Pk_k_1_ += Q;

	sotDEBUG(15) << "F " <<std::endl << F << std::endl;
	sotDEBUG(15) << "P_{k-1|k-1} " <<std::endl<< Pk_1_k_1 << std::endl;
	sotDEBUG(15) << "F^T " <<std::endl<< F.transpose() << std::endl;
	sotDEBUG(15) << "P_{k|k-1} " << std::endl << Pk_k_1_ << std::endl;

	S_ = H * Pk_k_1_ * H.transpose () + R;
	K_ = Pk_k_1_ * H.transpose () * S_.inverse ();
	Pk_k = Pk_k_1_ - K_ * H * Pk_k_1_;

	sotDEBUG (15) << "S_{k} " << std::endl << S_ << std::endl;
	sotDEBUG (15) << "K_{k} " << std::endl << K_ << std::endl;
	sotDEBUG (15) << "P_{k|k} " << std::endl << Pk_k << std::endl;

	sotDEBUGOUT(15);

	stateVariance_ = Pk_k;
      }
      return Pk_k;
    }

    //               ^
    //   z = y  - h (x     )
    //    k   k       k|k-1
    //
    //   ^     ^
    //   x   = x      + K  z
    //    k|k   k|k-1    k  k
    //
    //                  T
    //   S = H  P      H  + R
    //    k   k  k|k-1  k
    //
    //               T  -1
    //   K = P      H  S
    //    k   k|k-1  k  k
    //
    //   P   = (I - K  H ) P
    //    k|k        k  k   k|k-1

    Vector& Kalman::
    computeStateUpdate (Vector& x_est,const int& time )
    {
      sotDEBUGIN(15);
      if (time == 0) {
	// First time return variance initial state
	x_est = stateEstimation_;
	// Set dependency to input signals for latter computations
	stateUpdateSOUT.addDependency (measureSIN);
	stateUpdateSOUT.addDependency (observationPredictedSIN);
	stateUpdateSOUT.addDependency (modelMeasureSIN);
	stateUpdateSOUT.addDependency (noiseTransitionSIN);
	stateUpdateSOUT.addDependency (noiseMeasureSIN);
	stateUpdateSOUT.addDependency (statePredictedSIN);
	stateUpdateSOUT.addDependency (varianceUpdateSOUT);
      } else {
	varianceUpdateSOUT.recompute (time);
	const Vector & x_pred = statePredictedSIN (time);
	const Vector & y_pred = observationPredictedSIN (time);
	const Vector & y = measureSIN (time);

	sotDEBUG(25) << "K_{k} = "<<std::endl<< K_ << std::endl;
	sotDEBUG(25) << "y = " << y << std::endl;
	sotDEBUG(25) << "h (\\hat{x}_{k|k-1}) = " << y_pred << std::endl;
	sotDEBUG(25) << "y = " << y << std::endl;

	// Innovation: z_ = y - Hx
	z_ = y - y_pred;
	//x_est = x_pred + (K*(y-(H*x_pred)));
	x_est = x_pred + K_ * z_;

	sotDEBUG(25) << "z_{k} = " << z_ << std::endl;
	sotDEBUG(25) << "x_{k|k} = " << x_est << std::endl;

	stateEstimation_ = x_est;
      }
      sotDEBUGOUT (15);
      return x_est;
    }

    /* -------------------------------------------------------------------------- */
    /* --- MODELE --------------------------------------------------------------- */
    /* -------------------------------------------------------------------------- */

    void Kalman::
    display( std::ostream& ) const
    {
    }




    void Kalman::
    commandLine( const std::string&, std::istringstream&, std::ostream& )
    {
    }

  } // namespace sot
} // namespace dynamicgraph

/*!
  \file Kalman.cpp
  \brief  Generic kalman filtering implementation
*/


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

#ifndef __SOT_INTEGRATOR_EULER_H__
#define __SOT_INTEGRATOR_EULER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/integrator-abstract.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
 * \class IntegratorEuler
 * \brief integrates an ODE using a naive Euler integration.
 * TODO: change the integration method. For the moment, the highest
 * derivative of the output signal is computed using the
 * previous values of the other derivatives and the input
 * signal, then integrated n times, which will most certainly
 * induce a huge drift for ODEs with a high order at the denominator.
*/
template<class sigT,class coefT>
class IntegratorEuler
  : public IntegratorAbstract<sigT,coefT>
{

 public: 
  virtual const std::string& getClassName( void ) const { return dg::Entity::getClassName(); }
  static std::string getTypeName( void ) { return "Unknown"; }
  static const std::string CLASS_NAME;

 public:
  using IntegratorAbstract<sigT,coefT>::SIN;
  using IntegratorAbstract<sigT,coefT>::SOUT;
  using IntegratorAbstract<sigT,coefT>::numerator;
  using IntegratorAbstract<sigT,coefT>::denominator;

 public:
  IntegratorEuler( const std::string& name )
    : IntegratorAbstract<sigT,coefT>( name )
  {
    SOUT.addDependency(SIN);
  }

  virtual ~IntegratorEuler( void ) {}

protected:
  std::vector<sigT> inputMemory;
  std::vector<sigT> outputMemory;

public:
  sigT& integrate( sigT& res, int time )
  {
    sotDEBUG(15)<<"# In {"<<std::endl;

    const double dt = 0.005;
    const double invdt = 200;

    sigT sum;
    sigT tmp1, tmp2;
    const std::vector<coefT>& num = numerator;
    const std::vector<coefT>& denom = denominator;

    // Step 1
    tmp1 = inputMemory[0];
    inputMemory[0] = SIN.access(time);
    sum.resize(tmp1.size());
    sum = denom[0] * inputMemory[0];
    // End of step 1. Here, sum is b_0 X

    // Step 2
    int denomsize = denom.size();
    for(int i = 1; i < denomsize; ++i)
    {
      tmp2 = inputMemory[i-1] - tmp1;
      tmp2 *= invdt;
      tmp1 = inputMemory[i];
      inputMemory[i] = tmp2;
      sum += (denom[i] * inputMemory[i]);
    }
    // End of step 2. Here, sum is b_m * d(m)X / dt^m + ... - b_0 X

    // Step 3
    int numsize = num.size() - 1;
    for(int i = 0; i < numsize; ++i)
    {
      sum -= (num[i] * outputMemory[i]);
    }
    // End of step 3. Here, sum is b_m * d(m)X / dt^m + ... - b_0 X - a_0 Y - ... a_n-1 d(n-1)Y / dt^(n-1)

    // Step 4
    outputMemory[numsize] = sum;
    for(int i = numsize - 1; i >= 0; --i)
    {
      outputMemory[i] += (outputMemory[i+1] * dt);
    }
    // End of step 4. The ODE is integrated

    inputMemory[0] = SIN.access(time);
    res = outputMemory[0];

    sotDEBUG(15)<<"# Out }"<<std::endl;
    return res;
  }
};

} /* namespace sot */} /* namespace dynamicgraph */




#endif

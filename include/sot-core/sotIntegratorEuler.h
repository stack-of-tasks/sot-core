/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotIntegratorEuler.h
 * Project:   SOT
 * Author:    Paul Evrard and Nicolas Mansard
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



#ifndef __SOT_INTEGRATOR_EULER_H__
#define __SOT_INTEGRATOR_EULER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/sotIntegratorAbstract.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/*!
 * \class sotIntegratorEuler
 * \brief integrates an ODE using a naive Euler integration.
 * TODO: change the integration method. For the moment, the highest
 * derivative of the output signal is computed using the
 * previous values of the other derivatives and the input
 * signal, then integrated n times, which will most certainly
 * induce a huge drift for ODEs with a high order at the denominator.
*/
template<class sigT,class coefT>
class sotIntegratorEuler
  : public sotIntegratorAbstract<sigT,coefT>
{

 public: 
  virtual const std::string& getClassName( void ) const { return Entity::getClassName(); }
  static std::string getTypeName( void ) { return "Unknown"; }
  static const std::string CLASS_NAME;

 public:
  using sotIntegratorAbstract<sigT,coefT>::SIN;
  using sotIntegratorAbstract<sigT,coefT>::SOUT;
  using sotIntegratorAbstract<sigT,coefT>::numerator;
  using sotIntegratorAbstract<sigT,coefT>::denominator;

 public:
  sotIntegratorEuler( const std::string& name )
    : sotIntegratorAbstract<sigT,coefT>( name )
  {
    SOUT.addDependancy(SIN);
  }

  virtual ~sotIntegratorEuler( void ) {}

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

#endif

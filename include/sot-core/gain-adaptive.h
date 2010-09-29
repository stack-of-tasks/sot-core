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

#ifndef __SOT_GAIN_ADAPTATIVE_HH__
#define __SOT_GAIN_ADAPTATIVE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (gain_adaptive_EXPORTS)
#    define SOTGAINADAPTATIVE_EXPORT __declspec(dllexport)
#  else
#    define SOTGAINADAPTATIVE_EXPORT  __declspec(dllimport)
#  endif
#else
#  define SOTGAINADAPTATIVE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTGAINADAPTATIVE_EXPORT GainAdaptive
: public dg::Entity
{

 public: /* --- CONSTANTS --- */

  /* Default values. */
  static const double ZERO_DEFAULT;   // = 0.1
  static const double INFTY_DEFAULT;  // = 0.1
  static const double TAN_DEFAULT;    // = 1.

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }


 protected:

  /* Parameters of the adaptative-gain function:
   * lambda (x) = a * exp (-b*x) + c. */
  double coeff_a;
  double coeff_b;
  double coeff_c;

 public: /* --- CONSTRUCTORS ---- */

  GainAdaptive( const std::string & name );
  GainAdaptive( const std::string & name,const double& lambda );
  GainAdaptive( const std::string & name,
		     const double& valueAt0,
		     const double& valueAtInfty,
		     const double& tanAt0 );

 public: /* --- INIT --- */

  inline void init( void ) { init( ZERO_DEFAULT,INFTY_DEFAULT,TAN_DEFAULT ); }
  inline void init( const double& lambda ) { init( lambda,lambda,1.); }
  void init( const double& valueAt0,
	     const double& valueAtInfty,
	     const double& tanAt0 );
  void forceConstant( void );

 public:  /* --- SIGNALS --- */
  dg::SignalPtr<ml::Vector,int> errorSIN;
  dg::SignalTimeDependent<double,int> gainSOUT;
 protected:
  double& computeGain( double& res,int t );

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );
};

} // namespace sot



#endif // #ifndef __SOT_GAIN_ADAPTATIVE_HH__

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

#ifndef __SOT_FEATURE_JOINTLIMITS_HH__
#define __SOT_FEATURE_JOINTLIMITS_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/exception-task.h>
#include <dynamic-graph/all-signals.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (joint_limitator_EXPORTS)
#    define SOTJOINTLIMITATOR_EXPORT __declspec(dllexport)
#  else  
#    define SOTJOINTLIMITATOR_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTJOINTLIMITATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


namespace sot {

namespace dg = dynamicgraph;

/*!
  \class JointLimitator
*/
class SOTJOINTLIMITATOR_EXPORT JointLimitator
: public dg::Entity
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  
  double threshold;
  const static double THRESHOLD_DEFAULT; // = .99;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalPtr< ml::Vector,int > jointSIN;
  dg::SignalPtr< ml::Vector,int > upperJlSIN;
  dg::SignalPtr< ml::Vector,int > lowerJlSIN;
  dg::SignalPtr< ml::Vector,int > controlSIN;
  dg::SignalTimeDependent< ml::Vector,int > controlSOUT;
  dg::SignalTimeDependent< ml::Vector,int > widthJlSINTERN;

 public:
  JointLimitator( const std::string& name );
  virtual ~JointLimitator( void ) {}

  virtual ml::Vector& computeControl( ml::Vector& res,int time ); 
  ml::Vector& computeWidthJl( ml::Vector& res,const int& time );

  virtual void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
} ;


} // namespace sot



#endif // #ifndef __SOT_FEATURE_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

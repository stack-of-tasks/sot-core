/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vector-constant.h
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

#include <dynamic-graph/entity.h>

#include <dynamic-graph/all-signals.h>

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace sot{
namespace dg = dynamicgraph;

class VectorConstant
: public dg::Entity
{
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  int rows;
  double color;

public:
  VectorConstant( const std::string& name )
    :Entity( name )
    ,rows(0),color(0.)
    ,SOUT( "sotVectorConstant("+name+")::output(vector)::out" )
    {
      SOUT.setDependencyType( dg::TimeDependency<int>::BOOL_DEPENDENT );
      signalRegistration( SOUT );
    }

  virtual ~VectorConstant( void ){}

  dg::SignalTimeDependent<ml::Vector,int> SOUT;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};
    
} // namespace sot






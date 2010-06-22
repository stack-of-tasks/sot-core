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

class VectorConstant
: public Entity
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
      SOUT.setDependancyType( TimeDependancy<int>::BOOL_DEPENDANT );
      signalRegistration( SOUT );
    }

  virtual ~VectorConstant( void ){}

  SignalTimeDependant<ml::Vector,int> SOUT;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};
    
} // namespace sot






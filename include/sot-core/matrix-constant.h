/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      matrix-constant.h
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
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class sotMatrixConstant
: public Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  int rows,cols;
  double color;

public:
  sotMatrixConstant( const std::string& name )
    :Entity( name )
    ,rows(0),cols(0),color(0.)
    ,SOUT( "sotMatrixConstant("+name+")::output(matrix)::out" )
    {
      SOUT.setDependancyType( TimeDependancy<int>::BOOL_DEPENDANT );
      signalRegistration( SOUT );
    }

  virtual ~sotMatrixConstant( void ){}

  SignalTimeDependant<ml::Matrix,int> SOUT;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};
    







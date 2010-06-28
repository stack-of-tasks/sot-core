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

namespace sot {
namespace dg = dynamicgraph;

class MatrixConstant
: public dg::Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  int rows,cols;
  double color;

public:
  MatrixConstant( const std::string& name )
    :Entity( name )
    ,rows(0),cols(0),color(0.)
    ,SOUT( "sotMatrixConstant("+name+")::output(matrix)::out" )
    {
      SOUT.setDependencyType( dg::TimeDependency<int>::BOOL_DEPENDENT );
      signalRegistration( SOUT );
    }

  virtual ~MatrixConstant( void ){}

  dg::SignalTimeDependent<ml::Matrix,int> SOUT;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};
    
} // namespace sot






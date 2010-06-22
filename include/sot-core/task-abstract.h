/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      task-abstract.h
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



#ifndef __SOT_TASKABSTRACT_H__
#define __SOT_TASKABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
#include <MatrixAbstractLayer/boostMatrixSvd.h>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/multi-bound.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOT_CORE_EXPORT TaskAbstract
: public Entity
{
 public:

  /* Use a derivative of this class to store computational memory. */
  class sotMemoryTaskAbstract
  {
  public:
    int timeLastChange;
  public:
  sotMemoryTaskAbstract( void ) : timeLastChange(0) {};
    virtual ~sotMemoryTaskAbstract( void ) {};
  public:
    virtual void commandLine( const std::string& cmdLine
                              ,std::istringstream& cmdArgs
                              ,std::ostream& os ) = 0;
    virtual void display( std::ostream& os ) const = 0;
    friend std::ostream&
      operator<<( std::ostream& os,const sotMemoryTaskAbstract& tcm )
      {tcm.display(os); return os;}
  };

 public:
  sotMemoryTaskAbstract * memoryInternal;

 protected:
  void taskRegistration( void );

 public:
  TaskAbstract( const std::string& n );

 public: /* --- SIGNALS --- */

  SignalTimeDependant< sotVectorMultiBound,int > taskSOUT;
  SignalTimeDependant< ml::Matrix,int > jacobianSOUT;
  SignalTimeDependant< ml::Vector,int > featureActivationSOUT;

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine
			    ,std::istringstream& cmdArgs
			    ,std::ostream& os ) ;
 public:
};

} // namespace sot





#endif /* #ifndef __SOT_TASKABSTRACT_H__ */



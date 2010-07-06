#ifndef __SOT_Control_PD_HH__
#define __SOT_Control_PD_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (control_pd_EXPORTS)
#    define ControlPD_EXPORT __declspec(dllexport)
#  else  
#    define ControlPD_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define ControlPD_EXPORT
#endif

namespace sot{
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class ControlPD_EXPORT ControlPD
: public dg::Entity
{

 public: /* --- CONSTRUCTOR ---- */

  ControlPD( const std::string & name );

 public: /* --- INIT --- */

  void init( const double& step);

 public: /* --- CONSTANTS --- */

  /* Default values. */
  static const double TIME_STEP_DEFAULT;   // = 0.001

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const; 
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }


 protected: 
  
  /* Parameters of the torque-control function: 
   * tau = kp * (qd-q) + kd* (dqd-dq) */
  double TimeStep;
  double _dimension;

 public:  /* --- SIGNALS --- */

  dg::SignalPtr<ml::Vector,int> KpSIN;
  dg::SignalPtr<ml::Vector,int> KdSIN;
  dg::SignalPtr<ml::Vector,int> positionSIN;
  dg::SignalPtr<ml::Vector,int> desiredpositionSIN;
  dg::SignalPtr<ml::Vector,int> velocitySIN;
  dg::SignalPtr<ml::Vector,int> desiredvelocitySIN;
  dg::SignalTimeDependent<ml::Vector,int> controlSOUT;

 protected:

  double& setsize(int dimension);
  ml::Vector& computeControl( ml::Vector& tau,int t );

};



} // namespace sot



#endif // #ifndef __SOT_Control_PD_HH__

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotIntegratorAbstract.h
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



#ifndef __SOT_INTEGRATOR_ABSTRACT_H__
#define __SOT_INTEGRATOR_ABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <sot-core/sotFlags.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/sotDebug.h>
#include <dynamic-graph/interpreter.h>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! \brief integrates an ODE. If Y is the output and X the input, the
 * following equation is integrated:
 * a_p * d(p)Y / dt^p + .... + a_0 Y = b_m * d(m)X / dt^m + ... . b_0 X
 * a_i are the coefficients of the denominator of the associated transfer
 * function between X and Y, while the b_i are those of the numerator.
*/
template<class sigT, class coefT>
class sotIntegratorAbstract
:public Entity
{
 public:
  virtual const std::string& getClassName() const { return Entity::getClassName(); }
  static std::string getTypeName( void ) { return "Unknown"; }
  static const std::string CLASS_NAME;

 public:
  sotIntegratorAbstract ( const std::string& name )
    :Entity(name)
     ,SIN(NULL,"sotIntegratorAbstract("+name+")::input(vector)::in")
     ,SOUT(boost::bind(&sotIntegratorAbstract<sigT,coefT>::integrate,this,_1,_2),
		 SIN,
		 "sotIntegratorAbstract("+name+")::output(vector)::out")
  {
    signalRegistration( SIN<<SOUT );
  }

  virtual ~sotIntegratorAbstract() {}

  virtual sigT& integrate( sigT& res,int time ) = 0;

 public:  /* --- INHERITANCE --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,std::ostream& os )
  {
    if( cmdLine == "pushNumCoef" )
    {
      std::string objname, signame;
      Interpreter::objectNameParser(cmdArgs, objname, signame);
      Entity& obj = pool.getEntity(objname);
      SignalBase<int>& sig = obj.getSignal(signame);
      try {
	Signal<coefT,int>& sigc = dynamic_cast<Signal<coefT,int>&>(sig);
	pushNumCoef(sigc.accessCopy());
      }
      catch(std::bad_cast& bc) {
	os << "Command ignored: bad_cast exception...";
      }
      cmdArgs >> std::ws;
    }
    else if( cmdLine == "pushDenomCoef" )
    {
      std::string objname, signame;
      Interpreter::objectNameParser(cmdArgs, objname, signame);
      Entity& obj = pool.getEntity(objname);
      SignalBase<int>& sig = obj.getSignal(signame);
      try {
	Signal<coefT,int>& sigc = dynamic_cast<Signal<coefT,int>&>(sig);
	pushDenomCoef(sigc.accessCopy());
      }
      catch(std::bad_cast& bc) {
	os << "Command ignored: bad_cast exception...";
      }
      cmdArgs >> std::ws;
    }
    else if( cmdLine == "popNumCoef" )
    {
      popNumCoef();
    }
    else if( cmdLine == "popDenomCoef" )
    {
      popDenomCoef();
    }
    else 
    {
    	Entity::commandLine(cmdLine, cmdArgs, os);
    }
  }

 public:
  void pushNumCoef(const coefT& numCoef) { numerator.push_back(numCoef); }
  void pushDenomCoef(const coefT& denomCoef) { denominator.push_back(denomCoef); }
  void popNumCoef() { numerator.pop_back(); }
  void popDenomCoef() { denominator.pop_back(); }

 public:
  SignalPtr<sigT, int> SIN;

  SignalTimeDependant<sigT, int> SOUT;

 protected:
  std::vector<coefT> numerator;
  std::vector<coefT> denominator;
};

#endif

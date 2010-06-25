/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sot-h.h
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


#ifndef __SOT_sotSOTH_H__
#define __SOT_sotSOTH_H__


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <sot-core/sot.h>
#include <sot-core/solver-hierarchical-inequalities.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef SOTSOTH_EXPORT 
# if defined (WIN32)
#  if defined (sotSOTH_EXPORTS)
#    define SOTSOTH_EXPORT __declspec(dllexport)
#  else
#    define SOTSOTH_EXPORT __declspec(dllimport)
#  endif
# else
#  define SOTSOTH_EXPORT
# endif
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTSOTH_EXPORT SotH
:public Sot
{
 public:
  /*! \brief Specify the name of the class entity. */
  static const std::string CLASS_NAME;
  /*! \brief Returns the name of this class. */
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  /* --- SPECIFIC MEM -------------------------------------------------- */
  class MemoryTaskSOTH
    : public TaskAbstract::MemoryTaskAbstract,public Entity
    {
    public:
      const SotH * referenceKey;
      SolverHierarchicalInequalities solver;
      ml::Matrix JK,Jff,Jact;
    public:
    MemoryTaskSOTH( const std::string & name,
                       const SotH * ref,
                       unsigned int nJ,
                       sotRotationComposedInExtenso& Qh,
                       bubMatrix &Rh,
                       SolverHierarchicalInequalities::ConstraintList &cH );

    public: // Entity heritage
      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const;
      virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
      dg::Signal< ml::Matrix,int > jacobianConstrainedSINOUT;
      dg::Signal< ml::Vector,int > diffErrorSINOUT;
      virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
                                std::ostream& os );

    };
  /* --- \ SPECIFIC MEM ------------------------------------------------ */


 protected:
  //typedef std::vector<SolverHierarchicalInequalities *> SolversList;
  //SolversList solvers;
  sotRotationComposedInExtenso Qh;
  bubMatrix Rh;
  SolverHierarchicalInequalities::ConstraintList constraintH;
  SolverHierarchicalInequalities solverNorm;
  SolverHierarchicalInequalities * solverPrec;
  bool fillMemorySignal;

 public:

  /*! \brief Default constructor */
  SotH( const std::string& name );
  ~SotH( void );

 public: /* --- CONTROL --- */

  /*! \name Methods to compute the control law following the
   recursive definition of the stack of tasks.
   @{
  */

  /*! \brief Compute the control law. */
  virtual ml::Vector& computeControlLaw( ml::Vector& control,const int& time );

  /*! @} */


 public: /* --- DISPLAY --- */

 public: /* --- COMMANDS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
                            std::ostream& os );
  virtual void defineNbDof( const unsigned int& nbDof );

};

} // namespace sot



#endif // #ifndef __SOT_sotSOTH_H__

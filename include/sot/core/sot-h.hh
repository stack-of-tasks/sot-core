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

#ifndef __SOT_sotSOTH_H__
#define __SOT_sotSOTH_H__


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <sot/core/sot.hh>
#include <sot/core/solver-hierarchical-inequalities.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef SOTSOTH_EXPORT
# if defined (WIN32)
#  if defined (sot_h_EXPORTS)
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

namespace dynamicgraph { namespace sot {
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

} /* namespace sot */} /* namespace dynamicgraph */



#endif // #ifndef __SOT_sotSOTH_H__

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

#ifndef __SOT_SOT_HH
#define __SOT_SOT_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/malv2.hh>
DECLARE_MAL_NAMESPACE(ml);

/* Classes standards. */
#include <list>                    /* Classe std::list   */

/* SOT */
#include <sot/core/task-abstract.hh>
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <sot/core/constraint.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef SOTSOT_CORE_EXPORT
# if defined (WIN32)
#  if defined (sot_EXPORTS)
#    define SOTSOT_CORE_EXPORT __declspec(dllexport)
#  else
#    define SOTSOT_CORE_EXPORT __declspec(dllimport)
#  endif
# else
#  define SOTSOT_CORE_EXPORT
# endif
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {
    namespace dg = dynamicgraph;

    /*! @ingroup stackoftasks
      \brief This class implements the Stack of Task.
      It allows to deal with the priority of the controllers
      through the shell. The controllers can be either constraints
      either tasks.


    */
    class SOTSOT_CORE_EXPORT Sot
      :public Entity
    {
    public:
      /*! \brief Specify the name of the class entity. */
      static const std::string CLASS_NAME;
    public:
      /*! \brief Returns the name of this class. */
      virtual const std::string& getClassName() const {
	return CLASS_NAME;
      }

    protected:

      /*! \brief Defines a type for a list of tasks */
      typedef std::list<TaskAbstract*> StackType;


      /*! \brief This field is a list of controllers
	managed by the stack of tasks. */
      StackType stack;

      /*! \brief Defines a type for a list of constraints */
      typedef std::list<Constraint*> ConstraintListType;
      /*! \brief This field is a list of constraints
	managed by the stack of tasks. */
      ConstraintListType constraintList;

      /*! \brief Defines an interval in the state vector of the robot
	which is the free flyer. */
      unsigned int ffJointIdFirst,ffJointIdLast;
      /*! \brief Defines a default joint. */
      static const unsigned int FF_JOINT_ID_DEFAULT = 0;

      /*   double directionalThreshold; */
      /*   bool useContiInverse; */

      /*! \brief Store the number of joints to be used in the
	command computed by the stack of tasks. */
      unsigned int nbJoints;

      /*! \brief Store a pointer to compute the gradient */
      TaskAbstract* taskGradient;

      /*! Projection used to compute the control law. */
      ml::Matrix Proj;

      /*! Force the recomputation at each step. */
      bool recomputeEachTime;

    public:

      /*! \brief Threshold to compute the dumped pseudo inverse. */
      static const double INVERSION_THRESHOLD_DEFAULT; // = 1e-4;

      /*   static const double DIRECTIONAL_THRESHOLD_DEFAULT = 1e-2; */
      /*   static const bool USE_CONTI_INVERSE_DEFAULT = false; */

      /*! \brief Number of joints by default. */
      static const unsigned int NB_JOINTS_DEFAULT; // = 48;

      static ml::Matrix & computeJacobianConstrained( const ml::Matrix& Jac,
						      const ml::Matrix& K,
						      ml::Matrix& JK,
						      ml::Matrix& Jff,
						      ml::Matrix& Jact );
      static ml::Matrix & computeJacobianConstrained( const TaskAbstract& task,
						      const ml::Matrix& K );
      static ml::Vector
	taskVectorToMlVector(const VectorMultiBound& taskVector);

    public:

      /*! \brief Default constructor */
      Sot( const std::string& name );
      ~Sot( void ) { /* TODO!! */ }

      /*! \name Methods to handle the stack.
	@{
      */

      /*! \brief Push the task in the stack.
	It has a lowest priority than the previous ones.
	If this is the first task, then it has the highest
	priority. */
      virtual void push( TaskAbstract& task );
      /*! \brief Pop the task from the stack.
	This method removes the task with the smallest
	priority in the task. The other are projected
	in the null-space of their predecessors. */
      virtual TaskAbstract& pop( void );

      /*! \brief This method allows to know if a task exists or not */
      virtual bool exist( const TaskAbstract& task );

      /*! \brief Remove a task regardless to its position in the stack.
	It removes also the signals connected to the output signal of this
	stack.*/
      virtual void remove( const TaskAbstract& task );

      /*! \brief This method removes the output signals depending on
          this task. */
      virtual void removeDependency( const TaskAbstract& key );

      /*! \brief This method makes the task to swap with the task having the
	immediate superior priority. */
      virtual void up( const TaskAbstract& task );

      /*! \brief This method makes the task to swap with the task having the
	immediate inferior priority. */
      virtual void down( const TaskAbstract& task );

      /*! \brief Remove all the tasks from the stack. */
      virtual void clear( void );
      /*! @} */


      /*! \name Methods to handle the constraints.
	@{
      */
      /*! \brief Add a constraint to the stack with the current level
	of priority. */
      virtual void addConstraint( Constraint& constraint );
      /*! \brief Remove a constraint from the stack. */
      virtual void removeConstraint( const Constraint& constraint );
      /*! \brief Remove all the constraints from the stack. */
      virtual void clearConstraint( void );

      /*! @} */

      /*! \brief This method defines the part of the state vector
	which correspond to the free flyer of the robot. */
      virtual void
	defineFreeFloatingJoints(const unsigned int& jointIdFirst,
				 const unsigned int& jointIdLast = -1);
      virtual void defineNbDof( const unsigned int& nbDof );

      /*! @} */
    public: /* --- CONTROL --- */

      /*! \name Methods to compute the control law following the
	recursive definition of the stack of tasks.
	@{
      */

      /*! \brief Compute the control law. */
      virtual ml::Vector& computeControlLaw(ml::Vector& control,
					    const int& time);

      /*! \brief Compute the projector of the constraint. */
      virtual ml::Matrix& computeConstraintProjector(ml::Matrix& Proj,
						     const int& time );

      /*! @} */

    public: /* --- DISPLAY --- */

      /*! \name Methods to display the stack of tasks.
	@{
      */
      /*! Display the stack of tasks in text mode as a tree. */
      virtual void display( std::ostream& os ) const;
      /*! Wrap the previous method around an operator. */
      SOTSOT_CORE_EXPORT friend std::ostream&
	operator<< (std::ostream& os,const Sot& sot);
      /*! @} */
    public: /* --- SIGNALS --- */

      /*! \name Methods to handle signals
	@{
      */
      /*! \brief Intrinsec velocity of the robot, that is used to initialized
       * the recurence of the SOT (e.g. velocity comming from the other
       * OpenHRP plugins).
       */
      SignalPtr<ml::Vector,int> q0SIN;
      /*! \brief This signal allow to change the threshold for the
	damped pseudo-inverse on-line */
      SignalPtr<double,int> inversionThresholdSIN;
      /*! \brief Allow to get the result of the Constraint projector. */
      SignalTimeDependent<ml::Matrix,int> constraintSOUT;
      /*! \brief Allow to get the result of the computed control law. */
      SignalTimeDependent<ml::Vector,int> controlSOUT;
      /*! @} */

    public: /* --- COMMANDS --- */
      /*! \brief This method deals with the command line.
	The command given in argument is send to the stack of tasks by the shell.
	The command understood by sot are:
	<ul>
	<li> Tasks
	<ul>
	<li> push <task> : Push a task in the stack (FILO).
	<li> pop : Remove the task push in the stack.
	<li> down <task> : Make the task have a higher priority, i.e.
	swap with the task immediatly superior in priority.
	<li> up <task> : Make the task have a lowest priority, i.e.
	swap with the task immediatly inferior in priority.
	<li> rm <task> : Remove the task from the stack.
	</ul>
	<li> Constraints
	<ul>
	<li> addConstraint <constraint> : Add the constraint in the stack (FILO).
	<li> rmConstraint <constraint> : Remove the constraint.
	<li> clearConstraint : Remove all the constraints.
	<li> printConstraint :
	</ol>
	</ul>
      */
      virtual void commandLine(const std::string& cmdLine,
			       std::istringstream& cmdArgs,
			       std::ostream& os );

      /*! \brief This method write the priority between tasks in the output stream os. */
      virtual std::ostream & writeGraph(std::ostream & os) const;
    };
  } // namespace sot
} // namespace dynamicgraph

#endif /* #ifndef __SOT_SOT_HH */

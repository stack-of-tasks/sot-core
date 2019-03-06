/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_SOT_HH
#define __SOT_SOT_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

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

      /*! \brief Defines a type for a list of tasks */
      typedef std::list<TaskAbstract*> StackType;

    protected:

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

      //Eigen::MatrixXd<double,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> Proj;
      /*! Force the recomputation at each step. */
      bool recomputeEachTime;

    public:

      /*! \brief Threshold to compute the dumped pseudo inverse. */
      static const double INVERSION_THRESHOLD_DEFAULT; // = 1e-4;

      /*   static const double DIRECTIONAL_THRESHOLD_DEFAULT = 1e-2; */
      /*   static const bool USE_CONTI_INVERSE_DEFAULT = false; */

      /*! \brief Number of joints by default. */
      static dg::Matrix & computeJacobianConstrained( const dg::Matrix& Jac,
						      const dg::Matrix& K,
						      dg::Matrix& JK);
      static dg::Matrix & computeJacobianConstrained( const TaskAbstract& task,
						      const dg::Matrix& K );
      static void
	taskVectorToMlVector(const VectorMultiBound& taskVector, Vector& err);

    public:

      /*! \brief Default constructor */
      Sot( const std::string& name );
      ~Sot( void ) { /* TODO!! */ }

      /*! \name Methods to handle the stack.
	@{
      */
      virtual const StackType& tasks () const { return stack; }

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
      virtual const unsigned int& getNbDof() const { return nbJoints; }

      /*! @} */
    public: /* --- CONTROL --- */

      /*! \name Methods to compute the control law following the
	recursive definition of the stack of tasks.
	@{
      */

      /*! \brief Compute the control law. */
      virtual dg::Vector& computeControlLaw(dg::Vector& control,
					    const int& time);

      /*! \brief Compute the projector of the constraint. */
      virtual dg::Matrix& computeConstraintProjector(dg::Matrix& Proj,
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
      SignalPtr<dg::Vector,int> q0SIN;
      /*! \brief This signal allow to change the threshold for the
	damped pseudo-inverse on-line */
      SignalPtr<double,int> inversionThresholdSIN;
      /*! \brief Allow to get the result of the Constraint projector. */
      SignalTimeDependent<dg::Matrix,int> constraintSOUT;
      /*! \brief Allow to get the result of the computed control law. */
      SignalTimeDependent<dg::Vector,int> controlSOUT;
      /*! @} */

      /*! \brief This method write the priority between tasks in the output stream os. */
      virtual std::ostream & writeGraph(std::ostream & os) const;
    };
  } // namespace sot
} // namespace dynamicgraph

#endif /* #ifndef __SOT_SOT_HH */

/*
 * Copyright 2013,
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

#ifndef SOT_JOINT_TRAJECTORY_ENTITY_HH
#define SOT_JOINT_TRAJECTORY_ENTITY_HH


// Maal
#include <jrl/mal/boost.hh>
namespace ml= maal::boost;
// SOT 
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/trajectory.hh>
#include <list>

// API

#if defined (WIN32)
#  if defined (sot_joint_trajectory_entity_EXPORTS)
#    define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllexport)
#  else
#    define SOTJOINT_TRAJECTORY_ENTITY_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTJOINT_TRAJECTORY_ENTITY_EXPORT
#endif

// Class

namespace dynamicgraph { 
namespace sot {

  
    class SOTJOINT_TRAJECTORY_ENTITY_EXPORT SotJointTrajectoryEntity
      :public dynamicgraph::Entity
    {
    public:
      DYNAMIC_GRAPH_ENTITY_DECL();


      /// \brief Constructor
      SotJointTrajectoryEntity( const std::string& name );
      virtual ~SotJointTrajectoryEntity( ) { }

      void loadFile( const std::string& name );

      /// \brief Return the next pose for the legs.
      ml::Vector& getNextPosition( ml::Vector& pos, const int& time );

      /// \brief Return the next com.
      ml::Vector& getNextCoM( ml::Vector& com, const int& time );

      /// \brief Return the next cop.
      ml::Vector& getNextCoP( ml::Vector& cop, const int& time );

      /// \brief Return the next waist.
      ml::Vector& getNextWaist( ml::Vector& waist, const int& time );

      /// \brief Perform one update of the signals.
      int & OneStepOfUpdate(int &dummy, const int& time);

      /// @name Display
      /// @{
      virtual void display( std::ostream& os ) const;
      SOTJOINT_TRAJECTORY_ENTITY_EXPORT 
        friend std::ostream& operator<< 
        ( std::ostream& os,const SotJointTrajectoryEntity& r )
      { r.display(os); return os;}
      /// @}
  
    public: 
      typedef int Dummy;

      /// @name Signals
      /// @{
      /// \brief Internal signal for synchronisation.
      dynamicgraph::SignalTimeDependent<int,int> refresherSINTERN;
      
      /// \brief Internal signal to trigger one step of the algorithm. 
      SignalTimeDependent<Dummy,int> OneStepOfUpdateS;

      /// \brief Publish pose for each evaluation of the graph.
      dynamicgraph::SignalTimeDependent<ml::Vector,int> positionSOUT;

      /// \brief Publish com for each evaluation of the graph.
      dynamicgraph::SignalTimeDependent<ml::Vector,int> comSOUT;
      
      /// \brief Publish zmp for each evaluation of the graph.
      dynamicgraph::SignalTimeDependent<ml::Vector,int> zmpSOUT;

      /// \brief Publish waist for each evaluation of the graph.
      dynamicgraph::SignalTimeDependent<ml::Vector,int> waistSOUT;
      
      /// \brief Read a trajectory.
      dynamicgraph::SignalPtr<Trajectory,int> trajectorySIN;
      ///@}

    public: 
      /// @name Commands
      /// @{
      virtual void commandLine( const std::string& cmdLine,
                                std::istringstream& cmdArgs,
                                std::ostream& os );
      /// @}
    protected:
      
      /// \brief Index on the point along the trajectory.
      unsigned int index_;

      /// \brief Keep the starting time as an identifier of the trajector
      timestamp traj_timestamp_;

      /// \brief Store the pos;
      ml::Vector pose_;

      /// \brief Store the center of mass.
      ml::Vector com_;
      
      /// \brief Store the center of pressure ZMP.
      ml::Vector cop_;

      /// \brief Store the waist position.
      ml::Vector waist_;
      
    };


  } /* namespace sot */
} /* namespace dynamicgraph */

#endif // SOT_JOINT_TRAJECTORY_ENTITY_HH 





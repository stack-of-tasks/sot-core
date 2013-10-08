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

#include <sot/core/trajectory.hh>

/************************/
/* JointTrajectoryPoint */
/************************/

/**************/
/* Trajectory */
/**************/
namespace dynamicgraph {
  namespace sot {

    Trajectory::Trajectory(void)
    {
    }
    Trajectory::Trajectory(const Trajectory &copy)
    {
      header_ = copy.header_;
      time_from_start_ = copy.time_from_start_;
      points_ = copy.points_;
    }
    
    Trajectory::~Trajectory(void)
    {
    }
    
    void Trajectory::display(std::ostream &os)
    {
      unsigned int index=0;

      for (std::vector<std::string>::iterator it_joint_name = 
             joint_names_.begin();
           it_joint_name != joint_names_.end();
           it_joint_name++,index++)
        os << "Joint("<<index<<")="<< *(it_joint_name) << std::endl;
      
      for(std::vector<JointTrajectoryPoint>::iterator 
            it_point = points_.begin();
            it_point != points_.end();
          it_point++)
        {
          it_point->display(os);
        }
      
    }

  } /* ! namespace sot */
}   /* ! namespace dynamicgraph */




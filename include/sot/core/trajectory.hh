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

#ifndef __SOT_TRAJECTORY_H__
#define __SOT_TRAJECTORY_H__


/* --- Matrix --- */
#include <jrl/mal/boost.hh>
#include <sot/core/api.hh>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
  namespace sot {
    
    class SOT_CORE_EXPORT Header
    {
      unsigned int seq;
      double stamp;
      std::string frame_id;
    };


    class SOT_CORE_EXPORT JointTrajectoryPoint
    {
      std::vector<double> positions;
      std::vector<double> velocities;
      std::vector<double> accelerations;
      std::vector<double> effort;
      
      void transfert(std::vector<double> &src, unsigned int vecId)
      {
        std::vector<double> &dst;
        switch(vecId)
          {
          case(0): dst= positions;
            break;
          case(1): dst= velocities;
            break;
          case(2): dst= accelerations;
            break;
          case(3): dst= effort;
            break;
          }
        dst=src;
#if 0
        dst.resize(src.size());
        for(std::vector<double>::sizetype id=0;
            id<src.size();id++)
          dst[id] = src[id];
#endif
      }
    };

    class SOT_CORE_EXPORT Trajectory
    {
      
    public: 
      
      Trajectory( void );
      Trajectory( const Trajectory & copy );
      virtual ~Trajectory( void ) { }

      std::vector<std::string> joint_names_;

      Header header_;
      double time_from_start_;
      
      std::vector<JointTrajectoryPoint> points_;
      
    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef __SOT_TRAJECTORY_H__ */







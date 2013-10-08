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

    class SOT_CORE_EXPORT timestamp
    {
    public:
      unsigned long int secs_;
      unsigned long int nsecs_;      
      timestamp() : secs_(0),nsecs_(0)
        {}
      timestamp(const timestamp &ats)
        { secs_ = ats.secs_; nsecs_ = ats.nsecs_; }
      timestamp(unsigned long int lsecs, 
                unsigned long int lnsecs)
        { secs_ = lsecs; nsecs_ = lnsecs;}
      bool operator==(const timestamp &other) const 
      { 
        if ((secs_!=other.secs_) || (nsecs_!=other.nsecs_))
          return false;
        return true;   
      }
    };

    class SOT_CORE_EXPORT Header
    {
    public:
      unsigned int seq_;
      timestamp stamp_;
      std::string frame_id_;
    };


    class SOT_CORE_EXPORT JointTrajectoryPoint
    {
      
    public:
      std::vector<double> positions_;
      std::vector<double> velocities_;
      std::vector<double> accelerations_;
      std::vector<double> effort_;

      typedef std::vector<double> vec_ref;


      void display(std::ostream &os)
      {
        std::string names[4] = { "Positions", 
                                 "Velocities", 
                                 "Accelerations",
                                 "Effort"};
        
        std::vector<double> *points;

        for(unsigned int arrayId=0;arrayId<4;arrayId++)
          {
            switch(arrayId)
              {
              case(0):points = &positions_;
                break;
              case(1):points = &velocities_;
                break;
              case(2):points = &accelerations_;
                break;
              case(3):points = &effort_;
                break;
              }

            std::vector<double>::iterator it_db;
            os << names[arrayId] << std::endl
               << "---------" << std::endl;
            for(it_db = points->begin();
                it_db != points->end();
                it_db++)
              {
                os << *it_db << std::endl;
              }
          }
      }
      
      void transfert(const std::vector<double> &src, unsigned int vecId)
      {
        switch(vecId)
          {
          case(0): positions_ = src;
            break;
          case(1): velocities_ = src;
            break;
          case(2): accelerations_ = src;
            break;
          case(3): effort_ =src;
            break;
          }
      }
    };

    class SOT_CORE_EXPORT Trajectory
    {
      
    public: 
      
      Trajectory( void );
      Trajectory( const Trajectory & copy );
      virtual ~Trajectory( void );

      std::vector<std::string> joint_names_;

      Header header_;
      double time_from_start_;
      
      std::vector<JointTrajectoryPoint> points_;
      
      void display(std::ostream &);

    };
  } // namespace sot
} // namespace dynamicgraph


#endif /* #ifndef __SOT_TRAJECTORY_H__ */







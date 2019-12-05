/*
 * Copyright 2013,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */
#define VP_DEBUG
#define VP_DEBUG_MODE 45
#include <iostream>
#include <sot/core/debug.hh>
// #ifdef VP_DEBUG
//  class sotJTE__INIT
//  {
//  public:sotJTE__INIT( void ) { dynamicgraph::sot::DebugTrace::openFile(); }
//  };
//  sotJTE__INIT sotJTE_initiator;
// #endif //#ifdef VP_DEBUG

#include <sot/core/trajectory.hh>

/************************/
/* JointTrajectoryPoint */
/************************/

/**************/
/* Trajectory */
/**************/
namespace dynamicgraph {
namespace sot {

RulesJointTrajectory::RulesJointTrajectory(Trajectory &aTrajectoryToFill)
    : TrajectoryToFill_(aTrajectoryToFill), dbg_level(0),
      float_str_re("[-0-9]+\\.[0-9]*"),

      // Header Regular Expression
      seq_str_re("([0-9]+)"),
      timestamp_str_re("(" + float_str_re + "),(" + float_str_re + ")"),
      frame_id_str_re("[a-zA-z_0-9]*"),
      header_str_re("\\(" + seq_str_re + "\\,\\(" + timestamp_str_re + "\\),(" +
                    frame_id_str_re + ")\\)\\,\\("),

      // List of Joint Names
      joint_name_str_re("([a-zA-Z0-9_]+)"),
      list_of_jn_str_re(joint_name_str_re + "(\\,|\\))"),

      // Point
      point_value_str_re("(" + float_str_re + "+)|(?:)"),
      list_of_pv_str_re(point_value_str_re + "(\\,|\\))"), bg_pt_str_re("\\("),
      end_pt_str_re("\\)"), comma_pt_str_re("\\,\\("),

      // Liste of points
      bg_liste_of_pts_str_re("\\,\\("),

      // Reg Exps
      header_re(header_str_re), list_of_jn_re(list_of_jn_str_re),
      list_of_pv_re(list_of_pv_str_re), bg_pt_re(bg_pt_str_re),
      end_pt_re(end_pt_str_re), comma_pt_re(comma_pt_str_re),
      bg_liste_of_pts_re(bg_liste_of_pts_str_re) {}

bool RulesJointTrajectory::search_exp_sub_string(
    std::string &text, boost::match_results<std::string::const_iterator> &what,
    boost::regex &e, std::string &sub_text) {
  unsigned nb_failures = 0;

  boost::match_flag_type flags = boost::match_extra;
  if (boost::regex_search(text, what, e, flags)) {

    if (dbg_level > 5) {
      std::cout << "** Match found **\n   Sub-Expressions:" << what.size()
                << std::endl;
      for (unsigned int i = 0; i < what.size(); ++i)
        std::cout << "      $" << i << " = \"" << what[i] << "\" "
                  << what.position(i) << " " << what.length(i) << "\n";
    }
    if (what.size() >= 1) {
      unsigned int all_text = 0;
      boost::match_results<std::string::const_iterator>::difference_type pos =
          what.position(all_text);
      boost::match_results<std::string::const_iterator>::difference_type len =
          what.length(all_text);
      sub_text = text.substr(pos + len);
      return true;
    }
  } else {
    if (dbg_level > 5)
      std::cout << "** No Match found **\n";
    sub_text = text;
    nb_failures++;
    if (nb_failures > 100)
      return false;
  }
  return false;
}

void RulesJointTrajectory::parse_header(std::string &trajectory,
                                        std::string &sub_text1) {
  std::istringstream is;
  boost::match_results<std::string::const_iterator> what;

  if (search_exp_sub_string(trajectory, what, header_re, sub_text1)) {
    is.str(what[1]);
    is >> TrajectoryToFill_.header_.seq_;
    is.str(what[2]);
    is >> TrajectoryToFill_.header_.stamp_.secs_;
    is.str(what[3]);
    is >> TrajectoryToFill_.header_.stamp_.nsecs_;
    TrajectoryToFill_.header_.frame_id_ = what[4];

    if (dbg_level > 5) {
      std::cout << "seq: " << TrajectoryToFill_.header_.seq_ << std::endl;
      std::cout << "ts:" << TrajectoryToFill_.header_.stamp_.secs_ << " "
                << what[2] << " " << TrajectoryToFill_.header_.stamp_.nsecs_
                << " " << is.str() << std::endl;
      std::cout << "frame_id:" << TrajectoryToFill_.header_.frame_id_
                << std::endl;

      std::cout << "sub_text1:" << sub_text1 << std::endl;
    }
  }
}

void RulesJointTrajectory::parse_joint_names(
    std::string &trajectory, std::string &sub_text1,
    std::vector<std::string> &joint_names) {
  std::istringstream is;
  boost::match_results<std::string::const_iterator> what;
  bool joint_names_loop = true;
  do {
    if (search_exp_sub_string(trajectory, what, list_of_jn_re, sub_text1)) {
      std::string joint_name;
      is.str(what[1]);
      joint_name = what[1];
      joint_names.push_back(joint_name);

      std::string sep_char;
      sep_char = what[2];

      if (sep_char == ")")
        joint_names_loop = false;
      if (dbg_level > 5) {
        std::cout << "joint_name:" << joint_name << " " << sep_char
                  << std::endl;
        std::cout << "sub_text1:" << sub_text1 << std::endl;
      }
    }
    trajectory = sub_text1;

  } while (joint_names_loop);
}

bool RulesJointTrajectory::parse_seq(std::string &trajectory,
                                     std::string &sub_text1,
                                     std::vector<double> &seq) {
  boost::match_results<std::string::const_iterator> what;
  bool joint_seq_loop = true;
  std::istringstream is;
  std::string sub_text2 = trajectory;
  sub_text1 = trajectory;
  do {
    if (search_exp_sub_string(sub_text2, what, list_of_pv_re, sub_text1)) {
      std::string sep_char;
      if (dbg_level > 5) {
        std::cout << "size:" << what.size() << std::endl;
      }

      if (what.size() == 3) {
        std::string aString(what[1]);
        if (aString.size() > 0) {
          is.clear();
          is.str(aString);
          double aValue;
          is >> aValue;
          if (dbg_level > 5) {
            std::cout << aString << " | " << aValue << std::endl;
          }

          seq.push_back(aValue);
        }
        sep_char = what[2];
      } else if (what.size() == 1)
        sep_char = what[0];

      if (sep_char == ")")
        joint_seq_loop = false;

    } else {
      return true;
    }
    sub_text2 = sub_text1;
  } while (joint_seq_loop);
  return true;
}

bool RulesJointTrajectory::parse_point(std::string &trajectory,
                                       std::string &sub_text1) {
  std::vector<double> position, velocities, acceleration, effort;
  std::string sub_text2;
  boost::match_results<std::string::const_iterator> what;
  JointTrajectoryPoint aJTP;

  if (!search_exp_sub_string(trajectory, what, bg_pt_re, sub_text1))
    return false;
  sub_text2 = sub_text1;

  if (!parse_seq(sub_text2, sub_text1, aJTP.positions_))
    return false;
  sub_text2 = sub_text1;

  if (!search_exp_sub_string(sub_text2, what, comma_pt_re, sub_text1))
    return false;
  sub_text2 = sub_text1;

  if (!parse_seq(sub_text2, sub_text1, aJTP.velocities_))
    return false;
  sub_text2 = sub_text1;

  if (!search_exp_sub_string(sub_text2, what, comma_pt_re, sub_text1))
    return false;
  sub_text2 = sub_text1;

  if (!parse_seq(sub_text2, sub_text1, aJTP.accelerations_))
    return false;
  sub_text2 = sub_text1;

  if (!search_exp_sub_string(sub_text2, what, comma_pt_re, sub_text1))
    return false;
  sub_text2 = sub_text1;
  if (!parse_seq(sub_text2, sub_text1, aJTP.efforts_))
    return false;

  TrajectoryToFill_.points_.push_back(aJTP);
  return true;
}

bool RulesJointTrajectory::parse_points(std::string &trajectory,
                                        std::string &sub_text1) {
  boost::match_results<std::string::const_iterator> what;
  bool joint_points_loop = true;
  std::istringstream is;

  if (!search_exp_sub_string(trajectory, what, bg_liste_of_pts_re, sub_text1))
    return false;
  std::string sub_text2 = sub_text1;

  do {
    if (!search_exp_sub_string(sub_text2, what, bg_pt_re, sub_text1))
      return false;
    sub_text2 = sub_text1;

    if (!parse_point(sub_text2, sub_text1))
      return false;
    sub_text2 = sub_text1;

    if (!search_exp_sub_string(sub_text2, what, end_pt_re, sub_text1))
      return false;
    sub_text2 = sub_text1;

    if (!search_exp_sub_string(sub_text2, what, list_of_pv_re, sub_text1))
      return false;
    sub_text2 = sub_text1;
    std::string sep_char;
    sep_char = what[1];

    if (sep_char == ")")
      joint_points_loop = false;

  } while (joint_points_loop);

  return true;
}

void RulesJointTrajectory::parse_string(std::string &atext) {
  std::string sub_text1, sub_text2;
  parse_header(atext, sub_text2);
  sub_text1 = sub_text2;

  parse_joint_names(sub_text1, sub_text2, TrajectoryToFill_.joint_names_);

  if (dbg_level > 5) {
    for (std::vector<std::string>::size_type i = 0; i < joint_names.size(); i++)
      std::cout << joint_names[i] << std::endl;
  }

  sub_text1 = sub_text2;
  parse_points(sub_text1, sub_text2);
}

Trajectory::Trajectory(void) {}

Trajectory::Trajectory(const Trajectory &copy) {
  header_ = copy.header_;
  time_from_start_ = copy.time_from_start_;
  points_ = copy.points_;
}

Trajectory::~Trajectory(void) {}

int Trajectory::deserialize(std::istringstream &is) {
  std::string aStr = is.str();
  RulesJointTrajectory aRJT(*this);
  aRJT.parse_string(aStr);

  return 0;
}

void Trajectory::display(std::ostream &os) const {
  unsigned int index = 0;
  os << "-- Trajectory --" << std::endl;
  for (std::vector<std::string>::const_iterator it_joint_name =
           joint_names_.begin();
       it_joint_name != joint_names_.end(); it_joint_name++, index++)
    os << "Joint(" << index << ")=" << *(it_joint_name) << std::endl;

  os << " Number of points: " << points_.size() << std::endl;
  for (std::vector<JointTrajectoryPoint>::const_iterator it_point =
           points_.begin();
       it_point != points_.end(); it_point++) {
    it_point->display(os);
  }
}

} // namespace sot
} // namespace dynamicgraph

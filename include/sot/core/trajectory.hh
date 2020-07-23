/*
 * Copyright 2013,
 * Olivier Stasse,
 *
 * CNRS
 *
 */

#ifndef SOT_TRAJECTORY_H__
#define SOT_TRAJECTORY_H__

// Matrix
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/api.hh>

#include <boost/array.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>

namespace dynamicgraph {
namespace sot {

class Trajectory;

class RulesJointTrajectory {
protected:
  Trajectory &TrajectoryToFill_;

public:
  unsigned int dbg_level;

  /// \brief Strings specifying the grammar of the structure.
  std::string float_str_re, seq_str_re, timestamp_str_re, frame_id_str_re,
      header_str_re, joint_name_str_re, list_of_jn_str_re, point_value_str_re,
      list_of_pv_str_re, bg_pt_str_re, end_pt_str_re, comma_pt_str_re,
      bg_liste_of_pts_str_re;
  /// \brief Boost regular expressions implementing the grammar.
  boost::regex header_re, list_of_jn_re, list_of_pv_re, bg_pt_re, end_pt_re,
      comma_pt_re, bg_liste_of_pts_re;
  std::vector<std::string> joint_names;

  /// \brief Constructor TrajectoryToFill is the structure where to store the
  /// parsed information.
  RulesJointTrajectory(Trajectory &TrajectoryToFill);

  /// \brief parse_string will fill TrajectoryToFill with string atext.
  void parse_string(std::string &atext);

protected:
  /// \brief General parsing method of text with regexp e. The results are given
  /// in what. The remaining text is left in sub_text.
  bool
  search_exp_sub_string(std::string &text,
                        boost::match_results<std::string::const_iterator> &what,
                        boost::regex &e, std::string &sub_text);
  /// \brief Find and store the header.
  /// This method is looking for:
  /// unsigned int seq.
  /// unsigned int sec, unsigned int nsec.
  /// string format_id
  void parse_header(std::string &text, std::string &sub_text1);

  /// \brief Understand joint_names.
  /// Extract a list of strings.
  void parse_joint_names(std::string &text, std::string &sub_text1,
                         std::vector<std::string> &joint_names);

  /// \brief Extract a sequence of doubles.
  /// To be used for position, velocities, accelerations and effort.
  bool parse_seq(std::string &text, std::string &sub_text1,
                 std::vector<double> &seq);

  /// \brief Extract a point description.
  bool parse_point(std::string &trajectory, std::string &sub_text1);

  /// \brief Extract a sequence of points.
  bool parse_points(std::string &trajectory, std::string &sub_text1);
};

class SOT_CORE_EXPORT timestamp {
public:
  unsigned long int secs_;
  unsigned long int nsecs_;
  timestamp() : secs_(0), nsecs_(0) {}
  timestamp(const timestamp &ats) {
    secs_ = ats.secs_;
    nsecs_ = ats.nsecs_;
  }
  timestamp(unsigned long int lsecs, unsigned long int lnsecs) {
    secs_ = lsecs;
    nsecs_ = lnsecs;
  }
  bool operator==(const timestamp &other) const {
    if ((secs_ != other.secs_) || (nsecs_ != other.nsecs_))
      return false;
    return true;
  }
  friend std::ostream &operator<<(std::ostream &stream, const timestamp &ats) {
    stream << ats.secs_ + 0.000001 * (long double)ats.nsecs_;
    return stream;
  }
};

class SOT_CORE_EXPORT Header {
public:
  unsigned int seq_;
  timestamp stamp_;
  std::string frame_id_;
  Header() : seq_(0), stamp_(0, 0), frame_id_("initial_trajectory") {}
};

class SOT_CORE_EXPORT JointTrajectoryPoint {

public:
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> accelerations_;
  std::vector<double> efforts_;

  typedef std::vector<double> vec_ref;

  void display(std::ostream &os) const {
    boost::array<std::string, 4> names = boost::assign::list_of("Positions")(
        "Velocities")("Accelerations")("Effort");

    const std::vector<double> *points = 0;

    for (std::size_t arrayId = 0; arrayId < names.size(); ++arrayId) {
      switch (arrayId) {
      case (0):
        points = &positions_;
        break;
      case (1):
        points = &velocities_;
        break;
      case (2):
        points = &accelerations_;
        break;
      case (3):
        points = &efforts_;
        break;
      default:
        assert(0);
      }

      std::vector<double>::const_iterator it_db;
      os << names[arrayId] << std::endl << "---------" << std::endl;
      for (it_db = points->begin(); it_db != points->end(); it_db++) {
        os << *it_db << std::endl;
      }
    }
  }

  void transfer(const std::vector<double> &src, unsigned int vecId) {
    switch (vecId) {
    case (0):
      positions_ = src;
      break;
    case (1):
      velocities_ = src;
      break;
    case (2):
      accelerations_ = src;
      break;
    case (3):
      efforts_ = src;
      break;
    default:
      assert(0);
    }
  }
};

class SOT_CORE_EXPORT Trajectory {

public:
  Trajectory();
  Trajectory(const Trajectory &copy);
  virtual ~Trajectory();

  std::vector<std::string> joint_names_;

  Header header_;
  double time_from_start_;

  std::vector<JointTrajectoryPoint> points_;

  int deserialize(std::istringstream &is);
  void display(std::ostream &) const;
};
} // namespace sot
} // namespace dynamicgraph

#endif /* #ifndef SOT_TRAJECTORY_H__ */

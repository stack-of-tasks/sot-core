/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      ClampWorkspace.h
 * Project:   SOT
 * Author:    Evrard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 * Homotopy parameter for leader/follower switching to clamp
 * the workspace of the robot.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __SOT_CLAMP_WORKSPACE_HH__
#define __SOT_CLAMP_WORKSPACE_HH__

/* STL */
#include <utility>

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/exception-task.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/vector-roll-pitch-yaw.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-twist.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (clamp_workspace_EXPORTS)
#    define SOTCLAMPWORKSPACE_EXPORT __declspec(dllexport)
#  else  
#    define SOTCLAMPWORKSPACE_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTCLAMPWORKSPACE_EXPORT
#endif

namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTCLAMPWORKSPACE_EXPORT ClampWorkspace
  : public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalPtr< MatrixHomogeneous,int > positionrefSIN;
  dg::SignalPtr< MatrixHomogeneous,int > positionSIN;
  dg::SignalTimeDependent< ml::Matrix,int > alphaSOUT;
  dg::SignalTimeDependent< ml::Matrix,int > alphabarSOUT;
  dg::SignalTimeDependent< MatrixHomogeneous,int > handrefSOUT;

 public:

  ClampWorkspace( const std::string& name );
  virtual ~ClampWorkspace( void ) {}

  void update( int time );

  virtual ml::Matrix& computeOutput( ml::Matrix& res, int time );
  virtual ml::Matrix& computeOutputBar( ml::Matrix& res, int time );
  virtual MatrixHomogeneous& computeRef( MatrixHomogeneous& res, int time );

  virtual void display( std::ostream& ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );

 private:

  int timeUpdate;

  ml::Matrix alpha;
  ml::Matrix alphabar;
  MatrixHomogeneous prefMp;
  ml::Vector pd;
  MatrixRotation Rd;
  MatrixHomogeneous handref;

  double beta;
  double scale;
  double dm_min;
  double dm_max;
  double dm_min_yaw;
  double dm_max_yaw;
  double theta_min;
  double theta_max;
  int mode;

  enum {
    FRAME_POINT,
    FRAME_REF
  } frame;

  std::pair<double,double> bounds[3];
};


} // namespace sot


#endif

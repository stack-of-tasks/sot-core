/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_CLAMP_WORKSPACE_HH__
#define __SOT_CLAMP_WORKSPACE_HH__

/* STL */
#include <utility>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/exception-task.hh>
#include <dynamic-graph/all-signals.h>
#include <sot/core/matrix-geometry.hh>


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

namespace dynamicgraph { namespace sot {
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
  dg::SignalTimeDependent< dg::Matrix,int > alphaSOUT;
  dg::SignalTimeDependent< dg::Matrix,int > alphabarSOUT;
  dg::SignalTimeDependent< MatrixHomogeneous,int > handrefSOUT;

 public:

  ClampWorkspace( const std::string& name );
  virtual ~ClampWorkspace( void ) {}

  void update( int time );

  virtual dg::Matrix& computeOutput( dg::Matrix& res, int time );
  virtual dg::Matrix& computeOutputBar( dg::Matrix& res, int time );
  virtual MatrixHomogeneous& computeRef( MatrixHomogeneous& res, int time );

  virtual void display( std::ostream& ) const;

 private:

  int timeUpdate;

  dg::Matrix alpha;
  dg::Matrix alphabar;
  MatrixHomogeneous prefMp;
  dg::Vector pd;
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


} /* namespace sot */} /* namespace dynamicgraph */


#endif

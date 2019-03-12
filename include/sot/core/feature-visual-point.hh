/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_VISUALPOINT_HH__
#define __SOT_FEATURE_VISUALPOINT_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/exception-task.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_visual_point_EXPORTS)
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREVISUALPOINT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREVISUALPOINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureVisualPoint
  \brief Class that defines 2D visualPoint visual feature
*/
class SOTFEATUREVISUALPOINT_EXPORT FeatureVisualPoint
  : public FeatureAbstract, public FeatureReferenceHelper<FeatureVisualPoint>
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  dg::Matrix L;



  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< dg::Vector,int > xySIN;
  /** FeatureVisualPoint depth (required to compute the interaction matrix)
   * default Z = 1m. */
  dg::SignalPtr< double,int > ZSIN;
  dg::SignalPtr< dg::Matrix,int > articularJacobianSIN;

  using FeatureAbstract::selectionSIN;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;

  DECLARE_REFERENCE_FUNCTIONS(FeatureVisualPoint);

 public:
  FeatureVisualPoint( const std::string& name );
  virtual ~FeatureVisualPoint( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual dg::Vector& computeError( dg::Vector& res,int time );
  virtual dg::Matrix& computeJacobian( dg::Matrix& res,int time );

  /** Static Feature selection. */
  inline static Flags selectX( void ) { return FLAG_LINE_1; }
  inline static Flags selectY( void ) { return FLAG_LINE_2; }

  virtual void display( std::ostream& os ) const;


} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_VISUALPOINT_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

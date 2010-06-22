/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, Laas, CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      task-multi-bound.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_MultiBound_H__
#define __SOT_MultiBound_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>
#include <vector>
#include <iostream>

/* SOT */
#include <sot-core/sot-core-api.h>
#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOT_CORE_EXPORT MultiBound
{
 public:
  enum MultiBoundModeType { MODE_SINGLE, MODE_DOUBLE };
  enum SupInfType { BOUND_SUP,BOUND_INF };

 public:// protected:
  MultiBoundModeType mode;
  double boundSingle;
  double boundSup,boundInf;
  bool boundSupSetup,boundInfSetup;

 public:
  MultiBound( const double x = 0.);
  MultiBound( const double xi,const double xs );
  MultiBound( const double x,const SupInfType bound );
  MultiBound( const MultiBound& clone );

 public: // Acessors
  MultiBoundModeType getMode( void ) const;
  double getSingleBound( void ) const;
  double getDoubleBound( const SupInfType bound ) const;
  bool getDoubleBoundSetup( const SupInfType bound ) const;

 public: // Modifiors
  void setDoubleBound( SupInfType boundType,double boundValue );
  void unsetDoubleBound( SupInfType boundType );
  void setSingleBound( double boundValue );

 public:
  SOT_CORE_EXPORT friend std::ostream& operator<< ( std::ostream& os, const MultiBound & m  );
  SOT_CORE_EXPORT friend std::istream& operator>> ( std::istream& is, MultiBound & m  );
};

/* --------------------------------------------------------------------- */
typedef std::vector< MultiBound > sotVectorMultiBound;
SOT_CORE_EXPORT std::ostream& operator<< (std::ostream& os, const sotVectorMultiBound& v );
SOT_CORE_EXPORT std::istream& operator>> (std::istream& os, sotVectorMultiBound& v );

} // namespace sot


#endif // #ifndef __SOT_MultiBound_H__

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, Laas, CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMultiBound.h
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



#ifndef __SOT_sotMultiBound_H__
#define __SOT_sotMultiBound_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>
#include <vector>
#include <iostream>

/* SOT */
#include <sot-core/sot-core-api.h>
#include <sot-core/sotExceptionTask.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOT_CORE_EXPORT sotMultiBound
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
  sotMultiBound( const double x = 0.);
  sotMultiBound( const double xi,const double xs );
  sotMultiBound( const double x,const SupInfType bound );
  sotMultiBound( const sotMultiBound& clone );

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
  SOT_CORE_EXPORT friend std::ostream& operator<< ( std::ostream& os, const sotMultiBound & m  );
  SOT_CORE_EXPORT friend std::istream& operator>> ( std::istream& is, sotMultiBound & m  );
};

/* --------------------------------------------------------------------- */
typedef std::vector< sotMultiBound > sotVectorMultiBound;
SOT_CORE_EXPORT std::ostream& operator<< (std::ostream& os, const sotVectorMultiBound& v );
SOT_CORE_EXPORT std::istream& operator>> (std::istream& os, sotVectorMultiBound& v );



#endif // #ifndef __SOT_sotMultiBound_H__

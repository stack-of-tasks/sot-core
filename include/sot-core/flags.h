/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      flags.h
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



#ifndef __SOT_FLAGS_H
#define __SOT_FLAGS_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <vector>
#include <iostream>

/* SOT */
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOT_CORE_EXPORT Flags
{
 protected:
  std::vector<char> flags;
  bool reverse;

  char operator[]( const unsigned int& i ) const;
    
  
 public:
  
  Flags( const bool& b=false ) ;
  Flags( const char& c ) ;
  Flags( const int& c4 ) ;

  void add( const char& c ) ;
  void add( const int& c4 ) ;
    
  Flags operator! (void) const;
  SOT_CORE_EXPORT friend Flags operator& ( const Flags& f1,const Flags& f2 ) ;
  SOT_CORE_EXPORT friend Flags operator| ( const Flags& f1,const Flags& f2 ) ;
  Flags& operator&= ( const Flags& f2 ) ;
  Flags& operator|= ( const Flags& f2 ) ;
   
  SOT_CORE_EXPORT friend Flags operator& ( const Flags& f1,const bool& b ) ;
  SOT_CORE_EXPORT friend Flags operator| ( const Flags& f1,const bool& b ) ;
  Flags& operator&= ( const bool& b );
  Flags& operator|= ( const bool& b );
  
  SOT_CORE_EXPORT friend std::ostream& operator<< (std::ostream& os, const Flags& fl );
  SOT_CORE_EXPORT friend char operator>> (const Flags& flags, const int& i);
  SOT_CORE_EXPORT friend std::istream& operator>> (std::istream& is, Flags& fl );
  bool operator() (const int& i) const;

  operator bool (void) const;

  void unset( const unsigned int & i );
  void set( const unsigned int & i );

 public:  /* Selec "matlab-style" : 1:15, 1:, :45 ... */
  
  static void readIndexMatlab( std::istream & iss,
			       unsigned int & indexStart,
			       unsigned int & indexEnd,
			       bool& unspecifiedEnd );
  static Flags readIndexMatlab( std::istream & iss );

	  


};

SOT_CORE_EXPORT extern const Flags FLAG_LINE_1;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_2;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_3;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_4;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_5;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_6;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_7;
SOT_CORE_EXPORT extern const Flags FLAG_LINE_8;

} // namespace sot

#endif /* #ifndef __SOT_FLAGS_H */

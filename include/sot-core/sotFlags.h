/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotFlags.h
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


class SOT_CORE_EXPORT sotFlags
{
 protected:
  std::vector<char> flags;
  bool reverse;

  char operator[]( const unsigned int& i ) const;
    
  
 public:
  
  sotFlags( const bool& b=false ) ;
  sotFlags( const char& c ) ;
  sotFlags( const int& c4 ) ;

  void add( const char& c ) ;
  void add( const int& c4 ) ;
    
  sotFlags operator! (void) const;
  SOT_CORE_EXPORT friend sotFlags operator& ( const sotFlags& f1,const sotFlags& f2 ) ;
  SOT_CORE_EXPORT friend sotFlags operator| ( const sotFlags& f1,const sotFlags& f2 ) ;
  sotFlags& operator&= ( const sotFlags& f2 ) ;
  sotFlags& operator|= ( const sotFlags& f2 ) ;
   
  SOT_CORE_EXPORT friend sotFlags operator& ( const sotFlags& f1,const bool& b ) ;
  SOT_CORE_EXPORT friend sotFlags operator| ( const sotFlags& f1,const bool& b ) ;
  sotFlags& operator&= ( const bool& b );
  sotFlags& operator|= ( const bool& b );
  
  SOT_CORE_EXPORT friend std::ostream& operator<< (std::ostream& os, const sotFlags& fl );
  SOT_CORE_EXPORT friend char operator>> (const sotFlags& flags, const int& i);
  SOT_CORE_EXPORT friend std::istream& operator>> (std::istream& is, sotFlags& fl );
  bool operator() (const int& i) const;

  operator bool (void) const;

  void unset( const unsigned int & i );
  void set( const unsigned int & i );

 public:  /* Selec "matlab-style" : 1:15, 1:, :45 ... */
  
  static void readIndexMatlab( std::istream & iss,
			       unsigned int & indexStart,
			       unsigned int & indexEnd,
			       bool& unspecifiedEnd );
  static sotFlags readIndexMatlab( std::istream & iss );

	  


};

SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_1;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_2;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_3;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_4;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_5;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_6;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_7;
SOT_CORE_EXPORT extern const sotFlags FLAG_LINE_8;


#endif /* #ifndef __SOT_FLAGS_H */

/*
 * Copyright 2010,
 * François Bleibel,
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

#ifndef __SOT_FLAGS_H
#define __SOT_FLAGS_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <vector>
#include <iostream>

/* SOT */
#include "sot/core/api.hh"

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {

    class SOT_CORE_EXPORT Flags
    {
    protected:
      std::vector<char> flags;
      bool reverse;

      char operator[](const unsigned int& i) const;


    public:

      Flags(const bool& b=false);
      Flags(const char& c);
      Flags(const int& c4);

      void add(const char& c);
      void add(const int& c4);

      Flags operator!(void) const;
      SOT_CORE_EXPORT friend Flags operator&(const Flags& f1,const Flags& f2);
      SOT_CORE_EXPORT friend Flags operator|(const Flags& f1,const Flags& f2);
      Flags& operator&=(const Flags& f2);
      Flags& operator|=(const Flags& f2);

      SOT_CORE_EXPORT friend Flags operator&(const Flags& f1,const bool& b);
      SOT_CORE_EXPORT friend Flags operator|(const Flags& f1,const bool& b);
      Flags& operator&=(const bool& b);
      Flags& operator|=(const bool& b);

      SOT_CORE_EXPORT friend std::ostream& operator<<(std::ostream& os, const Flags& fl);
      SOT_CORE_EXPORT friend char operator>>(const Flags& flags, const int& i);
      SOT_CORE_EXPORT friend std::istream& operator>>(std::istream& is, Flags& fl);
      bool operator()(const int& i) const;

      operator bool(void) const;

      void unset(const unsigned int & i);
      void set(const unsigned int & i);

    public:  /* Selec "matlab-style" : 1:15, 1:, :45 ... */

      static void readIndexMatlab(std::istream & iss,
				  unsigned int & indexStart,
				  unsigned int & indexEnd,
				  bool& unspecifiedEnd);
      static Flags readIndexMatlab(std::istream & iss);




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
} // namespace dynamicgraph

#endif /* #ifndef __SOT_FLAGS_H */

/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*! System framework */
#include <list>
#include <stdlib.h>

/*! Local Framework */
#include <sot/core/debug.hh>
#include <sot/core/flags.hh>

using namespace std;
using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static void displaybool(ostream &os, const char c, const bool reverse = false) {
  for (std::size_t j = sizeof(char) * 8; j-- > 0;) {
    // os<<i<<","<<j<<": "<<c+0<<std::endl;
    if (reverse)
      os << (!((c >> j) & 0x1));
    else
      os << (((c >> j) & 0x1)); //?"1":"0");
  }
}
static string displaybool(const char c, const bool reverse = false) {
  stringstream oss;
  for (std::size_t j = sizeof(char) * 8; j-- > 0;) {
    // os<<i<<","<<j<<": "<<c+0<<std::endl;
    if (reverse)
      oss << (!((c >> j) & 0x1));
    else
      oss << (((c >> j) & 0x1)); //?"1":"0");
  }
  return oss.str();
}
/* --------------------------------------------------------------------- */

Flags::Flags(const bool &b) : flags(), reverse(b) {}

Flags::Flags(const char &c) : flags(), reverse(false) { add(c); }

Flags::Flags(const int &c4) : flags(), reverse(false) { add(c4); }

Flags::operator bool(void) const {
  if (reverse)
    return true;
  for (unsigned int i = 0; i < flags.size(); ++i)
    if (flags[i])
      return true;
  return false;
}

/* --------------------------------------------------------------------- */
char Flags::operator[](const unsigned int &i) const {
  char res;
  if (i < flags.size())
    res = flags[i];
  else
    res = 0;
  // cout<<"["<<i<<"] "<<res+0<<"||"<<(!res)+0<<std::endl;
  if (reverse)
    return static_cast<char>(~res); //(!res);
  return res;
}

namespace dynamicgraph {
namespace sot {
char operator>>(const Flags &f, const int &i) {
  const div_t q = div(i, 8);

  char res = static_cast<char>(f[q.quot] >> q.rem);
  res = static_cast<char>(res | f[q.quot + 1] << (8 - q.rem));

  return res;
}
} /* namespace sot */
} /* namespace dynamicgraph */

bool Flags::operator()(const int &i) const { return ((*this) >> i) & 0x01; }

/* --------------------------------------------------------------------- */
void Flags::add(const char &c) { flags.push_back(c); }

void Flags::add(const int &c4) {
  const char *c4p = (const char *)&c4;
  for (unsigned int i = 0; i < sizeof(int); ++i)
    add(c4p[i]);
}

/* --------------------------------------------------------------------- */
void Flags::set(const unsigned int &idx) {
  unsigned int d = (idx / 8), m = (idx % 8);

  char brik = static_cast<char>((reverse) ? (255 - (1 << m)) : (1 << m));

  if (flags.size() > d) {
    sotDEBUG(45) << "List long enough. Modify." << std::endl;
    char &el = flags[d];
    if (reverse)
      el &= brik;
    else
      el |= brik;
  } else {
    sotDEBUG(45) << "List not long enough. Add " << flags.size() << " " << d
                 << std::endl;
    if (!reverse) {
      for (std::vector<char>::size_type i = flags.size(); i < d; ++i)
        add((char)0);
      add(brik);
    }
  }
  sotDEBUG(45) << "New flag: " << *this << endl;
}

void Flags::unset(const unsigned int &idx) {
  unsigned int d = (idx / 8), m = (idx % 8);

  char brik = static_cast<char>((reverse) ? (1 << m) : (255 - (1 << m)));
  if (flags.size() > d) {
    sotDEBUG(45) << "List long enough. Modify." << std::endl;
    char &el = flags[d];
    if (reverse)
      el |= brik;
    else
      el &= brik;
  } else {
    sotDEBUG(45) << "List not long enough. Add." << std::endl;
    if (reverse) {
      for (std::vector<char>::size_type i = flags.size(); i < d; ++i)
        add((char)255);
      add(brik);
    }
  }
  sotDEBUG(45) << "New flag: " << *this << endl;
}

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
Flags Flags::operator!(void) const {
  Flags res = *this;
  res.reverse = !reverse;
  return res;
}

Flags operator&(const Flags &f1, const Flags &f2) {
  Flags res = f1;
  res &= f2;
  return res;
}

Flags operator|(const Flags &f1, const Flags &f2) {
  Flags res = f1;
  res |= f2;
  return res;
}

Flags &Flags::operator&=(const Flags &f2) {
  Flags &f1 = *this;
  const std::vector<char>::size_type max =
      std::max(flags.size(), f2.flags.size());
  if (flags.size() < max) {
    flags.resize(max);
  }
  bool revres = reverse && f2.reverse;
  char c;
  int pos = 0;
  for (unsigned int i = 0; i < max; ++i) {
    c = f1[i] & f2[i];
    if (revres)
      c = static_cast<char>(0xff - c);
    flags[i] = c;
    if (c)
      pos = i + 1;
  }
  flags.resize(pos);
  reverse = revres;
  return *this;
}

Flags &Flags::operator|=(const Flags &f2) {
  Flags &f1 = *this;
  const std::vector<char>::size_type max =
      std::max(flags.size(), f2.flags.size());
  if (flags.size() < max) {
    flags.resize(max);
  }
  bool revres = reverse || f2.reverse;
  char c;
  int pos = 0;
  for (unsigned int i = 0; i < max; ++i) {
    // cout<<"DGi ";displaybool(cout,f1[i],false); cout<<" ";
    // displaybool(cout,f2[i],false); cout<<endl;
    c = f1[i] | f2[i];
    // cout<<"DGr ";displaybool(cout,c,false); cout<<" ";
    // displaybool(cout,c,revres); cout<<endl;
    if (revres)
      c = static_cast<char>(0xff - c);
    flags[i] = c;
    if (c)
      pos = i + 1;
  }
  flags.resize(pos);
  reverse = revres;
  return *this;
}

Flags operator&(const Flags &f1, const bool &b) {
  if (b)
    return f1;
  else
    return Flags();
}
Flags operator|(const Flags &f1, const bool &b) {
  if (b)
    return Flags(true);
  else
    return f1;
}
Flags &Flags::operator&=(const bool &b) {
  if (!b) {
    flags.clear();
    reverse = false;
  }
  return *this;
}
Flags &Flags::operator|=(const bool &b) {
  if (b) {
    flags.clear();
    reverse = true;
  }
  return *this;
}

/* --------------------------------------------------------------------- */
std::ostream &operator<<(std::ostream &os, const Flags &fl) {
  if (fl.reverse)
    os << "...11111 ";
  std::vector<char>::size_type s = fl.flags.size();
  for (unsigned int i = 0; i < fl.flags.size(); ++i) {
    char c = fl.flags[s - i - 1];
    displaybool(os, c, fl.reverse);
    os << " ";
  }
  return os;
}

static unsigned char MASK[] = {0, 1, 3, 7, 15, 31, 63, 127, 255};

std::istream &operator>>(std::istream &is, Flags &fl) {
  sotDEBUGIN(15);
  std::list<char> listing;
  unsigned char count = 0, total = 0;
  char c, cur = 0;
  bool reverse = false, contin = true;
  do {
    is.get(c);
    sotDEBUG(25) << "Read " << total + 0 << endl;

    total++;
    if (!is.good())
      break;
    switch (c) {
    case '.': {
      if (total <= 3) {
        if (total == 3) {
          reverse = true;
          sotDEBUG(20) << " Reverse" << endl;
        }
      } else
        total = 10;
      break;
    }
    case '0':
      cur = static_cast<char>(cur & ~(0x01 << (7 - count++)));
      break;
    case '1':
      cur = static_cast<char>(cur | 0x01 << (7 - count++));
      break;
    case '#': {
      char cnot;
      is.get(cnot);
      if (cnot == '!')
        fl = (!Flags::readIndexMatlab(is));
      else {
        is.unget();
        fl = Flags::readIndexMatlab(is);
      }

      return is;
    }
    case '&': {
      char cnot;
      is.get(cnot);
      if (cnot == '!')
        fl &= (!Flags::readIndexMatlab(is));
      else {
        is.unget();
        fl &= Flags::readIndexMatlab(is);
      }
      return is;
    }
    case '|': {
      char cnot;
      is.get(cnot);
      if (cnot == '!')
        fl |= (!Flags::readIndexMatlab(is));
      else {
        is.unget();
        fl |= (Flags::readIndexMatlab(is));
      }
      return is;
    }
    default:
      is.unget();
      contin = false;
    }
    sotDEBUG(25) << "Get cur= " << displaybool(cur) << endl;

    if (count == 8) {
      sotDEBUG(20) << "Store " << displaybool(cur) << endl;
      count = 0;
      listing.push_front(cur);
      cur = 0;
    }
  } while (contin);

  sotDEBUG(20) << "finish with " << displaybool(cur) << " (" << 0 + count << ")"
               << endl;
  char insert = 0;
  fl.flags.resize(listing.size() + ((count > 0) ? 1 : 0));
  total = 0;
  for (std::list<char>::iterator iter = listing.begin(); iter != listing.end();
       ++iter) {
    insert = static_cast<char>((*iter) << count);
    insert = static_cast<char>(insert & ~MASK[count]);
    cur = static_cast<char>((MASK[count]) & (cur >> (8 - count)));

    insert |= cur;
    cur = *iter;

    if (reverse)
      fl.flags[total++] = static_cast<char>(~insert);
    else
      fl.flags[total++] = insert;
    sotDEBUG(25) << "Insert " << displaybool(insert) << endl;
  }

  if (count > 0) {
    sotDEBUG(25) << "Cur fin " << displaybool(cur) << endl;
    cur = static_cast<char>((MASK[count]) & (cur >> (8 - count)));

    sotDEBUG(25) << "Cur fin >> " << 8 - count << ":" << displaybool(cur)
                 << endl;
    sotDEBUG(25) << "Mask fin " << 0 + count << ": " << displaybool(MASK[count])
                 << endl;

    cur = static_cast<char>(cur & MASK[count]);
    if (reverse) {
      cur = static_cast<char>(cur | ~MASK[count]);
      fl.flags[total++] = static_cast<char>(~cur);
    } else {
      cur = static_cast<char>(cur & MASK[count]);
      fl.flags[total++] = cur;
    }
    sotDEBUG(25) << "Insert fin " << displaybool(cur) << endl;
  }

  fl.reverse = reverse;

  sotDEBUGOUT(15);
  return is;
}

/* --------------------------------------------------------------------- */
const Flags FLAG_LINE_1((char)0x1);
const Flags FLAG_LINE_2((char)0x2);
const Flags FLAG_LINE_3((char)0x4);
const Flags FLAG_LINE_4((char)0x8);
const Flags FLAG_LINE_5((char)0x10);
const Flags FLAG_LINE_6((char)0x20);
const Flags FLAG_LINE_7((char)0x40);
const Flags FLAG_LINE_8((char)0x80);

} /* namespace sot */
} /* namespace dynamicgraph */

/* --------------------------------------------------------------------- */

void Flags::readIndexMatlab(std::istream &cmdArgs, unsigned int &idx_beg,
                            unsigned int &idx_end, bool &no_end) {
  char col;

  cmdArgs >> ws;
  if (!cmdArgs.good()) {
    idx_end = idx_beg = 0;
    no_end = false;
    return;
  }
  cmdArgs.get(col);
  if (col == ':') {
    idx_beg = 0;
    cmdArgs >> ws;
  } else {
    cmdArgs.putback(col);
    cmdArgs >> idx_beg >> ws;
    cmdArgs.get(col);
    if (col != ':') {
      idx_end = idx_beg;
      no_end = false;
      return;
    }
  }
  cmdArgs >> ws;
  if (cmdArgs.good()) {
    sotDEBUG(15) << "Read end" << endl;
    cmdArgs >> idx_end;
    no_end = false;
  } else
    no_end = true;

  sotDEBUG(25) << "Selec: " << idx_beg << " : " << idx_end << "(" << no_end
               << ")" << endl;
}

Flags Flags::readIndexMatlab(std::istream &cmdArgs) {
  sotDEBUGIN(15);
  unsigned int idxEnd, idxStart;
  bool idxUnspec;

  readIndexMatlab(cmdArgs, idxStart, idxEnd, idxUnspec);

  Flags newFlag(idxUnspec);
  if (idxUnspec) {
    for (unsigned int i = 0; i < idxStart; ++i)
      newFlag.unset(i);
  } else {
    for (unsigned int i = idxStart; i <= idxEnd; ++i)
      newFlag.set(i);
  }

  sotDEBUG(25) << newFlag << std::endl;
  sotDEBUGOUT(15);
  return newFlag;
}

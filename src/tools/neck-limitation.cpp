/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/pool.h>

#include <sot/core/debug.hh>
#include <sot/core/exception-tools.hh>
#include <sot/core/factory.hh>
#include <sot/core/neck-limitation.hh>
#include <sot/core/sot.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NeckLimitation, "NeckLimitation");

const double NeckLimitation::COEFF_LINEAR_DEFAULT = -25.0 / 42.0;
const double NeckLimitation::COEFF_AFFINE_DEFAULT = 0.6981;  // 40DG
const double NeckLimitation::SIGN_TILT_DEFAULT = 1;
const unsigned int NeckLimitation::PAN_RANK_DEFAULT = 14;
const unsigned int NeckLimitation::TILT_RANK_DEFAULT = 15;

NeckLimitation::NeckLimitation(const std::string &name)
    : Entity(name),
      panRank(PAN_RANK_DEFAULT),
      tiltRank(TILT_RANK_DEFAULT),
      coeffLinearPan(COEFF_LINEAR_DEFAULT),
      coeffAffinePan(COEFF_AFFINE_DEFAULT),
      signTilt(SIGN_TILT_DEFAULT)

      ,
      jointSIN(NULL, "NeckLimitation(" + name + ")::input(vector)::joint"),
      jointSOUT(
          boost::bind(&NeckLimitation::computeJointLimitation, this, _1, _2),
          jointSIN,
          "NeckLimitation(" + name + ")::output(dummy)::jointLimited") {
  sotDEBUGIN(5);

  signalRegistration(jointSIN << jointSOUT);

  sotDEBUGOUT(5);
}

NeckLimitation::~NeckLimitation(void) {
  sotDEBUGIN(5);

  sotDEBUGOUT(5);
  return;
}

/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */
/* --- SIGNALS -------------------------------------------------------------- */

dynamicgraph::Vector &NeckLimitation::computeJointLimitation(
    dynamicgraph::Vector &jointLimited, const int &timeSpec) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector &joint = jointSIN(timeSpec);
  jointLimited = joint;

  const double &pan = joint(panRank);
  const double &tilt = joint(tiltRank);
  double &panLimited = jointLimited(panRank);
  double &tiltLimited = jointLimited(tiltRank);

  if (fabs(pan) < 1e-3)  // pan == 0
  {
    sotDEBUG(15) << "Pan = 0" << std::endl;
    if (tilt * signTilt > coeffAffinePan) {
      tiltLimited = coeffAffinePan;
    } else {
      tiltLimited = tilt;
    }
    panLimited = pan;
  } else if (pan > 0) {
    sotDEBUG(15) << "Pan > 0" << std::endl;
    if (signTilt * tilt > (pan * coeffLinearPan + coeffAffinePan)) {
      // Orthogonal projection .
      // 	  if( (tilt-coeffAffinePan)*coeffLinearPan<-1*pan )
      // 	    {
      // 	      panLimited=0; tiltLimited=coeffAffinePan;
      // 	    }
      // 	  else
      // 	    {
      // 	      double tmp = 1/(1+coeffLinearPan*coeffLinearPan);
      // 	      double tmp2=pan+coeffLinearPan*tilt;

      // 	      panLimited=(tmp2-coeffAffinePan*coeffLinearPan)*tmp;
      // 	      tiltLimited=(coeffLinearPan*tmp2+coeffAffinePan)*tmp;
      // 	    }
      tiltLimited = (pan * coeffLinearPan + coeffAffinePan);
      panLimited = pan;
    } else {
      tiltLimited = tilt;
      panLimited = pan;
    }
  } else  // pan<0
  {
    sotDEBUG(15) << "Pan < 0" << std::endl;
    sotDEBUG(15) << tilt - coeffAffinePan << "<?" << (-1 * pan * coeffLinearPan)
                 << std::endl;
    if (tilt * signTilt > (-pan * coeffLinearPan + coeffAffinePan)) {
      // 	  sotDEBUG(15) << "Below" << std::endl;
      // 	  if( (tilt-coeffAffinePan)*coeffLinearPan<pan )
      // 	    {
      // 	      sotDEBUG(15) << "Proj on 0" << std::endl;
      // 	      panLimited=0; tiltLimited=coeffAffinePan;
      // 	    }
      // 	  else
      // 	    {
      // 	      double tmp = 1/(1+coeffLinearPan*coeffLinearPan);
      // 	      double tmp2=pan-coeffLinearPan*tilt;

      // 	      panLimited=(tmp2+coeffAffinePan*coeffLinearPan)*tmp;
      // 	      tiltLimited=(-coeffLinearPan*tmp2+coeffAffinePan)*tmp;
      // 	    }
      tiltLimited = (-pan * coeffLinearPan + coeffAffinePan);
      panLimited = pan;
    } else {
      tiltLimited = tilt;
      panLimited = pan;
    }
  }

  sotDEBUGOUT(15);
  return jointLimited;
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */

void NeckLimitation::display(std::ostream &os) const {
  os << "NeckLimitation " << getName() << "." << std::endl;
}

#include <sot-core/sotRotationSimple.h>


bool MATLAB::fullPrec = false;
MATLAB::MATLAB( const sotRotationSimple& Qh,const unsigned int nJ)
{

  bubMatrix eye(nJ,nJ); eye.assign(bub::identity_matrix<double>(nJ));
  Qh.multiplyRight(eye);
  initFromBubMatrix(eye);
}

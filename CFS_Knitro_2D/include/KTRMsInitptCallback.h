/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>

namespace knitro {

class KTRISolver;

/**
 * Abstract base class for a multistart initial point callback, used to override KNITRO's default multistart initial points.
 * If this callback is used, CallbackFunction will be called in the multistart procedure every time a new initial
 * point for the multistart solve is needed.
 */
class KTRMSInitptCallback {
 public:

  /**
   * Virtual destructor required for virtual classes. 
   */
  virtual ~KTRMSInitptCallback() {
  }

  /**
   * Redefines the initial primal and dual variable values for a single run of the multistart solve procedure.
   * @param[in] nSolveNumber Indicates the number of times that the KNITRO multistart has solved with a new point.
   * @param[in] xLoBnds Lower bounds of primal variables.
   * @param[in] xUpBnds Upper bounds of primal variables.
   * @param[in,out] x Input value is KNITRO's default value, chosen randomly for each solve. The values of x
   * set by this function will be used as the initial values for the next solve of the multistart procedures.
   * @param[in,out] lambda Input value is KNITRO's default value, chosen randomly for each solve. The values of lambda
   * set by this function will be used as the initial values for the next solve of the multistart procedures.
   * @param[in] solver Pointer to the KTRSolver that originated the call to this function.
   * @return 0 if no error is encountered, and KTR_RC_CALLBACK_ERR if an error is encountered.
   * If an error is returned, KNITRO will continue the multistart solve with its default (random) initial values.
   */
  virtual int CallbackFunction(int nSolveNumber, const std::vector<double>& xLoBnds, const std::vector<double>& xUpBnds,
                               std::vector<double>& x, std::vector<double>& lambda, KTRISolver* solver) = 0;
};
}


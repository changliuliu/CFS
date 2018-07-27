/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include "knitro.h"

namespace knitro {

class KTRISolver;

/**
 * Abstract base class for a new point callback. 
 * The CallbackFunction is called after each new estimate of the solution point.
 */
class KTRNewptCallback {
 public:

  /**
   * Virtual destructor required for virtual classes. 
   */
  virtual ~KTRNewptCallback() {
  }

  /**
   *
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param solver
   * @return
   */
  virtual int CallbackFunction(const std::vector<double>& x, const std::vector<double>& lambda, double obj,
                               const std::vector<double>& c, const std::vector<double>& objGrad,
                               const std::vector<double>& jac, KTRISolver * solver) = 0;

  /**
   * @param x
   * @param obj
   * @param objGrad
   * @param jac
   * @param solver
   * @return
   */ 
  virtual int CallbackFunctionLSQ(const std::vector<double>& x, double obj, const std::vector<double>& objGrad,
                                  const std::vector<double>& jac, KTRISolver * solver) { return -1; }

};

}


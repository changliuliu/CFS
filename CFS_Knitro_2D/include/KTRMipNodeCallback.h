/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>

namespace knitro {

class KTRSolver;

/**
 * Abstract base class for a KNITRO MIP node callback.
 * If a MIP node callback is used, CallbackFunction will be called after each node is processed
 * in the branch-and-bound tree. The callback
 */
class KTRMipNodeCallback {
 public:

  /**
   * Virtual destructor required for virtual classes.
   */
  virtual ~KTRMipNodeCallback() {
  }

  /**
   *
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param solver
   * @return
   */
  virtual int CallbackFunction(const std::vector<double>& x, const std::vector<double>& lambda, const double obj,
                               const std::vector<double>& c, KTRSolver* solver) = 0;
};
}


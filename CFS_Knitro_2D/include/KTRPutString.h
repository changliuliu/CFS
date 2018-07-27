/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

namespace knitro {

class KTRISolver;

/**
 * Abstract base class for redirecting KNITRO output. By default, KNITRO prints to stdout,
 * or a file named knitro.log (set by seting parameter KTR_PARAM_OUTMODE).
 * CallbackFunction can be implemented to redefine how KNITRO output is printed,
 * and will be called when KNITROSolver::solve() is called and output is created.
 */
class KTRPutString {
 public:

  /**
   * VirutalVirtual destructor required for virtual classes. 
   */
  virtual ~KTRPutString() {}

  /**
   * Redirects KNITRO output.
   * @param[in] str Output from the KNITRO solver.
   * @param[in] solver Pointer to the KTRSolver that originated the call to this function.
   * @return The number of characters printed.
   */
  virtual int CallbackFunction(const std::string & str, KTRISolver* solver) = 0;
};
}

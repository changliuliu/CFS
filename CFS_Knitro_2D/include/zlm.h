/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"
#include "KTRException.h"

namespace knitro {

class ZLM {
 public:
  /**
   * Constructor for ZLM class.
   * Calls ZLM_checkout_license() from the KNITRO C API.
   */
  ZLM();

  /**
   * Releases the Ziena high volume license checked out by the constructor.
   * This function should not be called manually.
   */
  ~ZLM();

  /**
   * Called by the KNITROSolver constructor when validating KNITRO license and registering problem data with KNITRO. This function does not need to be called manually.
   * @return The ZLM_context_ptr pointing to the allocated memory for the Ziena high volume license checked out in the class constructor.
   */
  ZLM_context * getLicenseManager() const;

 private:
  /**
   * Verifies that the KNITRO network license is valid.
   * @throws KTRException if no valid KNITRO network license is found.
   */
  void checkLicenseValidity() const;

  /**
   * Pointer to a structure holding Ziena License Manager information.
   */
  ZLM_context_ptr _zlm;

  /**
   * The ZLM copy constructor is unimplemented so that copies of a ZLM object cannot be made, as each ZLM object holds a unique ZLM_context_ptr.
   * @param other
   */
  ZLM(const ZLM& other);

  /**
   * The ZLM copy operator is unimplemented so that copies of a ZLM object cannot be made, as each ZLM object holds a unique ZLM_context_ptr.
   * @param
   * @return
   */
  ZLM& operator=(const ZLM&);
};

}

#include "impl/zlm.hxx"


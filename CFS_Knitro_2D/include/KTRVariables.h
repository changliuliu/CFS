/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "KTRElements.h"

namespace knitro {

/**
 * Class for storing decision variable input information. See KTRElements for more information.
 */
class KTRVariables : public KTRElements {
 public:

  /**
   * Calls the KTRElements constructor with parameter n.
   * @param n Number of variables for which to hold information in the object.
   */
  explicit KTRVariables(int n);
};
}

#include "impl/KTRVariables.hxx"


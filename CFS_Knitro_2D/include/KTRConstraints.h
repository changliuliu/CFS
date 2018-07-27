/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include "KTRElements.h"
#include "KTRFunctionTypes.h"

namespace knitro {

/**
 * Class holding all input information for a set of constraints: bounds, constraint type, and function type.
 */
class KTRConstraints : public KTRElements {
 public:

  /**
   * Initializes the bounds, type and fnType
   * members with n zeroes and calls the parent KTRElements constructor.
   * @param n
   */
  explicit KTRConstraints(int n);

  /**
   * 
   * @return A vector of the function types of all constraints.
   */
  const std::vector<int>& getFnTypes() const;

  /**
   * Set the function type for a single constraint.
   * @param id Index of the constraint.
   * @param val Function type value for the constraint.
   */
  void setFnTypes(int id, int val);

  /**
   * Set the function type for all constraints.
   * @param fnTypes new values for function types.
   * @throws KTRException if fnTypes.size() is not equal to the number of constraints in the problem. 
   */
  void setFnTypes(const std::vector<int>& fnTypes);

  /**
   * Set all constraint function types to a single value.
   * @param fnTypes Value for all constraint function types.
   */
  void setFnTypes(int fnTypes);

 private:
  /**
   * Holds the constraint function types.
   */
  KTRFunctionTypes _fnTypes;
};
}

#include "impl/KTRConstraints.hxx"


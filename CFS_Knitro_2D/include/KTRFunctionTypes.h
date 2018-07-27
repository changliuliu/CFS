/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include "knitro.h"

namespace knitro {

/**
 * Class holding function types. An object of this class is contained in KTRConstraint objects.
 * Function type is used to indicate convexity of a function, and passed to the solver when solving
 * a MIP problem.
 */
class KTRFunctionTypes {
 public:
  /**
   * Initializes the fnType private member with all values set to KTREnums::FunctionType::Uncertain.
   * @param n The number of function types to hold.
   */
  explicit KTRFunctionTypes(int n);

  /**
   * @return A vector of the function types of all values.
   */
  const std::vector<int>& getFnTypes() const;

  /**
   * Set a single function type value.
   * @param id The index of the item to set.
   * @param val The function type value to set.
   * @param n The total number of function types held. Used by other classes if this value needs to be resized.
   */
  void setFnTypes(int id, int val, int n);

  /**
   * Set the function type for all values.
   * @param fnTypes new values for function types.
   * Should have size equal to the number of values.
   */
  void setFnTypes(const std::vector<int>& fnTypes);

  /**
   * Set all function types to a single value.
   * @param fnTypes Value for all function types.
   * @param n The total number of function types held. Used by other classes if this value needs to be resized.
   */
  void setFnTypes(int fnTypes, int n);

 private:
  /**
   * Function types held in an object of this class.
   */
  std::vector<int> _fnTypes;
};
}

#include "impl/KTRFunctionTypes.hxx"


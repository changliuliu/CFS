/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>

#include "knitro.h"

namespace knitro {

/**
 * Class holding variable or constraint type. An object of this class is contained in KTRVariable and KTRConstraint objects.
 */
class KTRTypes {
 public:
  /**
   * Initializes the type private member with all values set to zero.
   * @param n The number of types to hold.
   */
  explicit KTRTypes(int n);

  /**
   *
   * @return A vector of the types of all values.
   */
  const std::vector<int>& getTypes() const;

  /**
   * Set a single type value.
   * @param id The index of the item to set.
   * @param val The type value to set.
   * @param n The total number of types held. Used by other classes if this value needs to be resized.
   */
  void setTypes(int id, int val, int n);

  /**
   * Set all types.
   * @param types New values for all types.
   * Should have size equal to the number of values.
   */
  void setTypes(const std::vector<int>& types);

  /**
   * Sets all types to a single value.
   * @param types Value to set all types to.
   * @param n The total number of types held. Used by other classes if this value needs to be resized.
   */
  void setTypes(int types, int n);

 private:

  /**
   * Types held in an object of this class.
   */
  std::vector<int> _types;
};

}

#include "impl/KTRTypes.hxx"


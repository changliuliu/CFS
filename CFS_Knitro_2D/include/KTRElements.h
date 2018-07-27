/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>

#include "KTRBounds.h"
#include "KTRTypes.h"

namespace knitro {

/**
 * Base class for KTRVariables and KTRConstraints. Store bound and type values.
 * KTRVariables and KTRConstraints should be used directly instead of this class.
 */
class KTRElements {
 public:
  /**
   * Initializes a KTRElements object. Bound and type members are initialized to size 0 (empty).
   * Empty bounds is equivalent to all values being unbounded.
   * @param n Number of elements to be held in the object.
   */
  explicit KTRElements(int n);

  /**
   *
   * @return Number of elements held in the object.
   */
  int size() const;

  /**
   * Get lower bounds for all elements.
   * @return All element lower bounds
   */
  const std::vector<double>& getLoBnds() const;

  /**
   * Set a single lower bound value.
   * @param id The index of the item to set.
   * @param val The lower bound value. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   */
  void setLoBnds(int id, double val);

  /**
   * Set lower bounds to values in the arguments. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param loBnds Low bounds values to set.
   * @throws KTRException if loBnds parameter has the wrong size.
   */
  void setLoBnds(const std::vector<double>& loBnds);

  /**
   * Sets all lower bounds to a single value. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param loBnds Value to set all lower bounds to.
   */
  void setLoBnds(double loBnds);

  /**
   * Get upper bounds for all elements.
   * @return All element upper bounds
   */
  const std::vector<double>& getUpBnds() const;

  /**
   * Set a single upper bound value.
   * @param id The index of the item to set.
   * @param val The upper bound value. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   */
  void setUpBnds(int id, double val);

  /**
   * Set upper bounds to values in the arguments. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param upBnds Upper bounds values to set.
   * @throws KTRException if upBnds parameter has the wrong size.
   */
  void setUpBnds(const std::vector<double>& upBnds);

  /**
   * Sets all upper bounds to a single value. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param upBnds Value to set all upper bounds to.
   */
  void setUpBnds(double upBnds);

  /**
   * Get all element types.
   * @return All element types.
   */
  const std::vector<int>& getTypes() const;

  /**
   * Set the type of all elements.
   * @param types New values for all element types.
   * @throws KTRException if types parameter has the wrong size.
   */
  void setTypes(const std::vector<int>& types);

  /**
   * Set a single type value.
   * @param id The index of the item to set.
   * @param val The type value to set.
   */
  void setTypes(int id, int val);

  /**
   * Sets all types to a single value.
   * @param types Value to set all types to.
   */
  void setTypes(int types);

 private:
  /**
   * Lower bounds of all elements.
   */
  KTRBounds _bounds;

  /**
   * Type of all elements.
   */
  KTRTypes _type;

  /**
   * Number of elements held.
   */
  int _size;
};
}

#include "impl/KTRElements.hxx"


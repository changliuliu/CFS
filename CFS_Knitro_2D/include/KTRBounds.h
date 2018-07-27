/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include "knitro.h"

namespace knitro {

/**
 * Used to store the upper and lower bounds of variables or constraints for a problem.
 * Used in the KTRProblem class; this class need not directly be used.
 */
class KTRBounds {
 public:
  /**
   * Create a KTRBounds object, with lower bounds set to -KTR_INFBOUND and upper bounds set to KTR_INFBOUND.
   * @param n the number of lower and upper bounds to hold.
   */
  explicit KTRBounds(int n);

  /**
   * @return A vector of the lower bounds stored in this class.
   */
  const std::vector<double>& getLoBnds() const;

  /**
   * Sets a lower bound for one element.
   * @param id Index of the lower bound to set.
   * @param val Lower bound value to set. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param n Used to resize the number of bounds if the lower bounds vector member is empty.
   * The member is empty if all variables are unbounded. When the bound is resized to n elements, all
   * elements except for id are unbounded below.
   */
  void setLoBnds(int id, double val, int n);

  /**
   * Set the lower bounds to a copy of the parameter. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param loBnds std::vector containing lower bounds.
   */
  void setLoBnds(const std::vector<double>& loBnds);

  /**
   * Resizes the lower bounds to have n elements all with the value val.
   * This function is used by the KTRElements class to set lower bounds.
   * @param val The new lower bound value for all elements. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param n The number of elements that will be held in the lower bound.
   */
  void setLoBnds(double val, int n);

  /**
   * @return A vector of the upper bounds stored in this class.
   */
  const std::vector<double>& getUpBnds() const;

  /**
   * Sets an upper bound for one element.
   * @param id Index of the upper bound to set.
   * @param val Upper bound value to set. Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * @param n Used to resize the number of bounds if the upper bounds vector member is empty.
   * The member is empty if all variables are unbounded. When the bound is resized to n elements, all
   * elements except for id are unbounded above.
   */
  void setUpBnds(int id, double val, int n);

  /**
   * Set the upper bounds to a copy of the parameter upBnds.
   * @param upBnds std::vector containing upper bounds.
   */
  void setUpBnds(const std::vector<double>& upBnds);

  /**
   * Resizes the upper bounds to have n elements all with the value val.
   * Valid values are within [-KTR_INFBOUND, KTR_INFBOUND].
   * This function is used by the KTRElements class to set upper bounds.
   * @param val The new upper bound value for all elements.
   * @param n The number of elements that will be held in the upper bound.
   */
  void setUpBnds(double val, int n);
 private:
  /**
   * Holds lower bounds.
   */
  std::vector<double> _loBnds;

  /**
   * Holds upper bounds.
   */
  std::vector<double> _upBnds;
};
}

#include "impl/KTRBounds.hxx"



/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"

namespace knitro {

/**
 * Class for storing objective function information. Used by the KTRProblem class.
 */
class KTRObjective {
 public:

  /**
   * Initializes the objective to general, the objective function type (convexity) to uncertain, and the objective function goal to minimization.
   */
  KTRObjective();

  /**
   * Called by the KTRProblem object when registering the problem with KNITRO.
   * @return The objective function goal (minimization or maximization).
   */
  int getGoal() const;

  /**
   * Sets the objective function goal. Called by the KTRProblem setObjGoal function.
   * @param goal Objective function goal (minimization or maximization).
   */
  void setGoal(int goal);

  /**
   * Called by the KTRProblem object when registering the problem with KNITRO.
   * @return The objective function type (function convexity information).
   */
  int getFnType() const;

  /**
   *
   * @param fnType
   */
  void setFnType(int fnType);

  /**
   * Called by the KTRProblem object when registering the problem with KNITRO.
   * @return The objective type (whether the objective function is linear, quadratic, or neither).
   */
  int getType() const;

  /**
   *
   * @param type
   */
  void setType(int type);

 private:
  /**
   * The objective function type (whether the objective function is linear, quadratic, or neither).
   */
  int _type;

  /**
   * The objective function type (function convexity information).
   */
  int _fnType;

  /**
   * The objective function goal (minimize or maximize).
   */
  int _goal;
};
}

#include "impl/KTRObjective.hxx"


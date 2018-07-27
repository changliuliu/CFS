/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"
#include "KTRObjective.h"
#include "KTREnums.h"

namespace knitro {

inline KTRObjective::KTRObjective()
    : _type(KTREnums::ObjectiveType::ObjGeneral),
      _fnType(KTREnums::FunctionType::Uncertain),
      _goal(KTREnums::ObjectiveGoal::Minimize) {
}

inline int KTRObjective::getGoal() const {
  return _goal;
}

inline void KTRObjective::setGoal(int goal) {
  _goal = goal;
}

inline int KTRObjective::getFnType() const {
  return _fnType;
}

inline void KTRObjective::setFnType(int fnType) {
  _fnType = fnType;
}

inline int KTRObjective::getType() const {
  return _type;
}

inline void KTRObjective::setType(int type) {
  _type = type;
}
}


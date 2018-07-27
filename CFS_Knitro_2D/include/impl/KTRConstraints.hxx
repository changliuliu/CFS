/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <sstream>

#include "KTRConstraints.h"

namespace knitro {

inline KTRConstraints::KTRConstraints(int m)
    : KTRElements(m),
      _fnTypes(0) {
}

inline const std::vector<int>& KTRConstraints::getFnTypes() const {
  return _fnTypes.getFnTypes();
}

inline void KTRConstraints::setFnTypes(int id, int val) {
  return _fnTypes.setFnTypes(id, val, size());
}

inline void KTRConstraints::setFnTypes(const std::vector<int>& fnTypes) {

  if (this->size() != fnTypes.size()) {
    std::ostringstream ss;

    ss << "error setting function types. Expected length = " << this->size();
    ss << ", given length " + fnTypes.size();

    throw KTRException(ss.str(), __FILE__, __LINE__);
  }

  this->_fnTypes.setFnTypes(fnTypes);
}

inline void KTRConstraints::setFnTypes(int fnTypes) {
  this->_fnTypes.setFnTypes(fnTypes, size());
}

}
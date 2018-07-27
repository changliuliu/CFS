/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"
#include "KTREnums.h"

namespace knitro {

inline KTRFunctionTypes::KTRFunctionTypes(int n)
    : _fnTypes(n, KTREnums::FunctionType::Uncertain) {
}

inline const std::vector<int>& KTRFunctionTypes::getFnTypes() const {
  return _fnTypes;
}

inline void KTRFunctionTypes::setFnTypes(int id, int val, int n) {
  if (_fnTypes.empty()) {
    _fnTypes.resize(n, KTREnums::FunctionType::Uncertain);
  }

#ifndef NDEBUG
  _fnTypes.at(id) = val;
#else
  _fnTypes[id] = val;
#endif
}

inline void KTRFunctionTypes::setFnTypes(const std::vector<int>& fnTypes) {
  _fnTypes = fnTypes;
}

inline void KTRFunctionTypes::setFnTypes(int fnTypes, int n) {
  this->_fnTypes.assign(n, fnTypes);
}
}


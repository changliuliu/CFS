/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "KTRTypes.h"
#include "knitro.h"

namespace knitro {
inline KTRTypes::KTRTypes(int n)
    : _types(n) {
}

inline const std::vector<int>& KTRTypes::getTypes() const {
  return _types;
}

inline void KTRTypes::setTypes(int id, int val, int n) {
  if (_types.empty()) {
    _types.resize(n, 0);
  }

#ifndef NDEBUG
  _types.at(id) = val;
#else
  _types[id] = val;
#endif
}

inline void KTRTypes::setTypes(const std::vector<int>& types) {
  _types = types;
}

inline void KTRTypes::setTypes(int types, int n) {
  this->_types.assign(n, types);
}

}


/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"
#include "KTRBounds.h"

namespace knitro {
inline KTRBounds::KTRBounds(int n)
    : _loBnds(n, -KTR_INFBOUND),
      _upBnds(n, KTR_INFBOUND) {
}

inline const std::vector<double>& KTRBounds::getLoBnds() const {
  return _loBnds;
}

inline void KTRBounds::setLoBnds(int id, double val, int n) {
  if (_loBnds.empty()) {
    _loBnds.resize(n, -KTR_INFBOUND);
  }

#ifndef NDEBUG
  _loBnds.at(id) = val;
#else
  _loBnds[id] = val;
#endif
}

inline void KTRBounds::setLoBnds(const std::vector<double>& loBnds) {
  _loBnds = loBnds;
}

inline void KTRBounds::setLoBnds(double val, int n) {
  this->_loBnds.assign(n, val);
}

inline const std::vector<double>& KTRBounds::getUpBnds() const {
  return _upBnds;
}

inline void KTRBounds::setUpBnds(double val, int n) {
  this->_upBnds.assign(n, val);
}

inline void KTRBounds::setUpBnds(int id, double val, int n) {
  if (_upBnds.empty()) {
    _upBnds.resize(n, KTR_INFBOUND);
  }

#ifndef NDEBUG
  _upBnds.at(id) = val;
#else
  _upBnds[id] = val;
#endif
}

inline void KTRBounds::setUpBnds(const std::vector<double>& upBnds) {
  _upBnds = upBnds;
}

}


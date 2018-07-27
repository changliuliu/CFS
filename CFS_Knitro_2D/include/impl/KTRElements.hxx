/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <sstream>

#include "KTRElements.h"

namespace knitro {

inline KTRElements::KTRElements(int numElements)
    : _bounds(0),
      _type(0),
      _size(numElements) {
}

inline int KTRElements::size() const {
  return _size;
}

inline const std::vector<double>& KTRElements::getLoBnds() const {
  return _bounds.getLoBnds();
}

inline void KTRElements::setLoBnds(int id, double val) {
  _bounds.setLoBnds(id, val, size());
}

inline void KTRElements::setLoBnds(const std::vector<double>& loBnds) {
  if (this->size() != loBnds.size()) {
    std::ostringstream ss;

    ss << "error setting lower bounds. Expected length = " << this->size();
    ss << ", given length " + loBnds.size();

    throw KTRException(ss.str(), __FILE__, __LINE__);
  }

  this->_bounds.setLoBnds(loBnds);
}

inline void KTRElements::setLoBnds(double loBnds) {
  this->_bounds.setLoBnds(loBnds, size());
}

inline const std::vector<double>& KTRElements::getUpBnds() const {
  return _bounds.getUpBnds();
}

inline void KTRElements::setUpBnds(int id, double val) {
  return _bounds.setUpBnds(id, val, size());
}

inline void KTRElements::setUpBnds(const std::vector<double>& upBnds) {
  if (this->size() != upBnds.size()) {
    std::ostringstream ss;

    ss << "error setting upper bounds. Expected length = " << this->size();
    ss << ", given length " + upBnds.size();

    throw KTRException(ss.str(), __FILE__, __LINE__);
  }

  this->_bounds.setUpBnds(upBnds);
}

inline void KTRElements::setUpBnds(double upBnds) {
  this->_bounds.setUpBnds(upBnds, size());
}

inline const std::vector<int>& KTRElements::getTypes() const {
  return _type.getTypes();
}

inline void KTRElements::setTypes(const std::vector<int>& types) {
  if (this->size() != types.size()) {
    std::ostringstream ss;

    ss << "error setting types. Expected length = " << this->size();
    ss << ", given length " + types.size();

    throw KTRException(ss.str(), __FILE__, __LINE__);
  }

  this->_type.setTypes(types);
}

inline void KTRElements::setTypes(int id, int val) {
  _type.setTypes(id, val, size());
}

inline void KTRElements::setTypes(int types) {
  this->_type.setTypes(types, size());
}
}


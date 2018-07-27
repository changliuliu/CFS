/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <iostream>

#include "KTRException.h"

namespace knitro {

inline IKTRException::IKTRException(const std::string& message, const std::string& fileName, int lineNum,
                                    const std::string& type)
    : _message(message),
      _fileName(fileName),
      _lineNum(lineNum),
      _type(type.c_str()) {
}

inline void IKTRException::printMessage() const {
  std::cout << "Exception of type: " << _type << " in file ";
  std::cout << _fileName << " at line " << _lineNum << std::endl << "Exception message: " << _message << std::endl;
}

inline std::string const & IKTRException::message() const {
  return _message;
}

/**
 * Implementation of IKTRException.
 */
class KTRException : public IKTRException {
 public:
  KTRException(const std::string& message, const std::string& functionName, int line);
};

/**
 * Implementation of IKTRException constructor, with exception type set tp "KNITROException" by default.
 */
inline KTRException::KTRException(const std::string& message, const std::string& fileName, int lineNum)
    : IKTRException(message, fileName, lineNum, "KnitroException") {
}

}

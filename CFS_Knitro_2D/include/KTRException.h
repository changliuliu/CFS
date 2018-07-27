/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <string>
#include <iostream>

namespace knitro {

/**
 * Exception thrown when the object-oriented interface encounters a problem, such as an invalid KNITRO license,
 * invalid variable and constraint bounds, or invalid input parameters.
 */
class IKTRException {
 public:
  /**
   * Create a new exception.
   * @param message Error message.
   * @param fileName File in which the exception was thrown.
   * @param lineNum Line number in the file in which the exception was thrown.
   * @param type The type of exception thrown.
   */
  IKTRException(const std::string& message, const std::string& fileName, int lineNum, const std::string& type);

  /**
   * Virutal destructor.
   */
  virtual ~IKTRException() {
  }

  /**
   * Print the error message to stdout, and includes the line number and file in which the exception was thrown.
   */
  void printMessage() const;

  /**
   *
   * @return The exception message set in the constructor.
   */
  std::string const & message() const;

 private:

  /**
   * The exception message set in the constructor.
   */
  std::string _message;

  /**
   * File in which the exception was thrown.
   */
  std::string _fileName;

  /**
   * Line number in the file in which the exception was thrown.
   */
  int _lineNum;

  /**
   * The type of exception thrown.
   */
  std::string _type;
};
}

#include "impl/KTRException.hxx"

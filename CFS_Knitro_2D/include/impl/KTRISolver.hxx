/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <cstring>

#include "KTRIProblem.h"
#include "KTRException.h"

namespace knitro {

inline char *convert(const std::string & s) {
  char *pc = new char[s.size() + 1];
  std::memcpy(pc, s.c_str(), s.size() + 1);
  return pc;
}

inline KTRISolver::KTRISolver(KTRIProblem * problem, int gradopt, int hessopt)
    : _problem(problem),
      _kc(NULL),
      _obj(0),
      _xValues(problem->getNumVars()),
      _lambdaValues(problem->getNumVars() + problem->getNumCons()),
      _useMSInitptCallback(false),
      _useMSProcessCallback(false),
      _useNewptCallback(false),
      _initialSolveCall(true) {
  if (_problem->putStringFunctionWrapper("", this) != KTR_RC_CALLBACK_ERR) {
    _kc = KTR_new_puts(callbackRedirectOutput, static_cast<void*>(this));
  } else {
    _kc = KTR_new();
  }

  construct(gradopt, hessopt);
}

inline KTRISolver::KTRISolver(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt)
    : _problem(problem),
      _kc(NULL),
      _obj(0),
      _xValues(_problem->getNumVars()),
      _lambdaValues(_problem->getNumVars() + _problem->getNumCons()),
      _useMSInitptCallback(false),
      _useMSProcessCallback(false),
      _useNewptCallback(false),
      _initialSolveCall(true) {
  if (_problem->putStringFunctionWrapper("", this) != KTR_RC_CALLBACK_ERR) {
    _kc = KTR_new_zlm(callbackRedirectOutput, static_cast<void*>(this), zlm->getLicenseManager());
  } else {
    _kc = KTR_new_zlm(NULL, static_cast<void*>(this), zlm->getLicenseManager());
  }

  construct(gradopt, hessopt);
}

inline KTRISolver::KTRISolver(KTRIProblem * problem)
    : _problem(problem),
      _kc(NULL),
      _obj(0),
      _xValues(problem->getNumVars()),
      _lambdaValues(problem->getNumVars() + problem->getNumCons()),
      _useMSInitptCallback(false),
      _useMSProcessCallback(false),
      _useNewptCallback(false),
      _initialSolveCall(true) {
  if (_problem->putStringFunctionWrapper("", this) != KTR_RC_CALLBACK_ERR) {
    _kc = KTR_new_puts(callbackRedirectOutput, static_cast<void*>(this));
  } else {
    _kc = KTR_new();
  }

  construct();
}

inline KTRISolver::KTRISolver(const ZLM * zlm, KTRIProblem * problem)
    : _problem(problem),
      _kc(NULL),
      _obj(0),
      _xValues(_problem->getNumVars()),
      _lambdaValues(_problem->getNumVars() + _problem->getNumCons()),
      _useMSInitptCallback(false),
      _useMSProcessCallback(false),
      _useNewptCallback(false),
      _initialSolveCall(true) {
  if (_problem->putStringFunctionWrapper("", this) != KTR_RC_CALLBACK_ERR) {
    _kc = KTR_new_zlm(callbackRedirectOutput, static_cast<void*>(this), zlm->getLicenseManager());
  } else {
    _kc = KTR_new_zlm(NULL, static_cast<void*>(this), zlm->getLicenseManager());
  }

  construct();
}

inline KTRISolver::~KTRISolver() {
  KTR_free(&_kc);
}

inline KTRIProblem* KTRISolver::getProblem() {
  return _problem;
}

inline void KTRISolver::useMSInitptCallback() {
  setParam(KTR_PARAM_MULTISTART, KTR_MULTISTART_YES);
  _useMSInitptCallback = true;
}

inline void KTRISolver::useMSProcessCallback() {
  setParam(KTR_PARAM_MULTISTART, KTR_MULTISTART_YES);
  _useMSProcessCallback = true;
}

inline void KTRISolver::useNewptCallback() {
  setParam(KTR_PARAM_NEWPOINT, KTR_NEWPOINT_USER);
  _useNewptCallback = true;
}

inline void KTRISolver::resetParamsToDefault() {
  int retCode = KTR_reset_params_to_defaults(_kc);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_reset_params_to_defaults";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::loadParamFile(const std::string& filename) {
  int retCode = KTR_load_param_file(_kc, filename.c_str());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_load_param_file; filename: ";
    message << filename;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::saveParamFile(const std::string& filename) const {
  int retCode = KTR_save_param_file(_kc, filename.c_str());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_save_param_file; filename: ";
    message << filename;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setVarScaling(const std::vector<double>& xScaleFactors, const std::vector<double>& xScaleCenters){
  int retCode = KTR_set_var_scalings(_kc, xScaleFactors.data(), xScaleCenters.data());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_var_scalings; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setObjScaling(const double objScaleFactor){
  int retCode = KTR_set_obj_scaling(_kc, objScaleFactor);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_obj_scalings; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setHonorBounds(const std::vector<int>& honorBnds) {
  int retCode = KTR_set_honorbnds(_kc, honorBnds.data());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_honorbnds; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  } 
}

inline void KTRISolver::setParam(int paramID, int value) {
  int retCode = KTR_set_int_param(_kc, paramID, value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_int_param; ";
    message << "paramID: " + paramID;
    message << "; value " + value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setParam(const std::string& name, int value) {
  int retCode = KTR_set_int_param_by_name(_kc, name.c_str(), value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_int_param_by_name";
    message << "; name: ";
    message << name;
    message << "; value " + value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setParam(int paramID, const std::string& value) {
  int retCode = KTR_set_char_param(_kc, paramID, value.c_str());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_char_param";
    message << "; paramID: " + paramID;
    message << "; value ";
    message << value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setParam(const std::string& name, const std::string& value) {
  int retCode = KTR_set_char_param_by_name(_kc, name.c_str(), value.c_str());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_char_param_by_name";
    message << "; name: ";
    message << name;
    message << "; value: ";
    message << value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setParam(int paramID, double value) {
  int retCode = KTR_set_double_param(_kc, paramID, value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_double_param";
    message << "; paramID: " + paramID;
    message << "; value: ";
    message << value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setParam(const std::string& name, double value) {
  int retCode = KTR_set_double_param_by_name(_kc, name.c_str(), value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_double_param_by_name";
    message << "; name: " << name;
    message << "; value " << value;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRISolver::getIntParam(int paramID) const {
  int value;
  int retCode = KTR_get_int_param(_kc, paramID, &value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_int_param";
    message << "; paramID: " + paramID;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return value;
}

inline int KTRISolver::getIntParam(const std::string& name) const {
  int value;
  int retCode = KTR_get_int_param_by_name(_kc, name.c_str(), &value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_int_param_by_name";
    message << "; name: " << name;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return value;
}

inline double KTRISolver::getDoubleParam(int paramID) const {
  double value;
  int retCode = KTR_get_double_param(_kc, paramID, &value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_double_param";
    message << "; paramID: " + paramID;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return value;
}

inline double KTRISolver::getDoubleParam(const std::string& name) const {
  double value;
  int retCode = KTR_get_double_param_by_name(_kc, name.c_str(), &value);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_double_param_by_name";
    message << "; name: " << name;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return value;
}

inline std::string KTRISolver::getRelease() const {
  char buffer[21];

  KTR_get_release(20, buffer);

  return std::string(buffer);
}

inline void KTRISolver::loadTunerFile(const std::string& filename) {
  int retCode = KTR_load_tuner_file(_kc, filename.c_str());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_load_tuner_file";
    message << "; filename: " << filename;
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::setFindiffRelstepsizes(const std::vector<double>& relStepSizes) {
  if (relStepSizes.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xFeasTols has length " << relStepSizes.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_set_findiff_relstepsizes(_kc, &relStepSizes[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_findiff_relstepsizes";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline const std::vector<double>& KTRISolver::getXValues() const {
  return _xValues;
}

inline const std::vector<double>& KTRISolver::getLambdaValues() const {
  return _lambdaValues;
}

inline double KTRISolver::getXValues(int id) const {
  return _xValues[id];
}

inline double KTRISolver::getLambdaValues(int id) const {
  return _lambdaValues[id];
}

inline double KTRISolver::getObjValue() const {
  return _obj;
}

inline int KTRISolver::getNumberIters() const {
  int numIters = KTR_get_number_iters(_kc);
  if (numIters < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_number_iters";
    message << "; value: " << numIters;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numIters;
}

inline int KTRISolver::getNumberCGIters() const {
  int numIters = KTR_get_number_cg_iters(_kc);
  if (numIters < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_number_cg_iters";
    message << "; value: " << numIters;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numIters;
}

inline double KTRISolver::getAbsOptError() const {
  double error = KTR_get_abs_opt_error(_kc);
  if (error < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_abs_opt_error";
    message << "; value: " << error;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return error;
}

inline double KTRISolver::getRelOptError() const {
  double error = KTR_get_rel_opt_error(_kc);
  if (error < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_rel_opt_error";
    message << "; value: " << error;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return error;
}

inline std::vector<double> KTRISolver::getObjGradValues() const {
  std::vector<double> objGrad(_problem->getNumVars());
  int retCode = KTR_get_objgrad_values(_kc, &objGrad[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_objgrad_values";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return objGrad;
}

inline std::string KTRISolver::getParamName(int paramID) const {
  char buffer[25];

  int retCode = KTR_get_param_name(_kc, paramID, buffer, 25);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_paramName; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  std::string paramName(buffer);

  return paramName;
}

inline std::string KTRISolver::getParamDoc(int paramID) const {
  char buffer[700];

  int retCode = KTR_get_param_doc(_kc, paramID, buffer, 700);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_param_value_docget_param_doc; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return std::string(buffer);
}

inline int KTRISolver::getParamType(int paramID) const {
  int type;
  int retCode = KTR_get_param_type(_kc, paramID, &type);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_paramType; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return type;
}

inline int KTRISolver::getNumParamValues(int paramID) const {
  int numParamValues;
  int retCode = KTR_get_num_param_values(_kc, paramID, &numParamValues);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_numParamValues; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return numParamValues;
}

inline std::string KTRISolver::getParamValueDoc(int paramID, int answerId) const {
  char buffer[700];

  int retCode = KTR_get_param_value_doc(_kc, paramID, answerId, buffer, 700);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_param_value_doc; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return std::string(buffer);
}

inline int KTRISolver::getParamID(const std::string& name) const {
  int paramID;

  int retCode = KTR_get_param_id(_kc, name.c_str(), &paramID);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_param_id; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return paramID;
}

inline void KTRISolver::chgVarBnds(const std::vector<double>& xLoBnds, const std::vector<double>& xUpBnds) {
  if (xLoBnds.size() != 0 && xLoBnds.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xLoBnds has length " << xLoBnds.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (xUpBnds.size() != 0 && xUpBnds.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xUpBnds has length " << xUpBnds.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_chgvarbnds(_kc, xLoBnds.size() == 0 ? NULL : &xLoBnds[0], xUpBnds.size() == 0 ? NULL : &xUpBnds[0]);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_chgvarbnds; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::checkValidKNITROLicense() const {
  if (_kc == NULL) {
    std::ostringstream message;
    message << "Error validating KNITRO license";
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRISolver::construct(int gradopt, int hessopt) {
  checkValidKNITROLicense();
  setParam(KTR_PARAM_GRADOPT, gradopt); 
  setParam(KTR_PARAM_HESSOPT, hessopt);
}

inline int KTRISolver::callbackRedirectOutput(const char* const str, void* userParams) {
  KTRISolver* solver = static_cast<KTRISolver*>(userParams);
  return solver->getProblem()->putStringFunctionWrapper(str, solver);
}

inline std::string KTRISolver::CreateErrorMessage(int errorCode) {
  return "Call to a native KNITRO method returned returned error code " + errorCode;
}

inline double* KTRISolver::var() {
  if (&_xValues.front() == NULL) {
    std::ostringstream message;
    message << "Error: primal variable array sent to solver is null";
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return &_xValues.front();
}

inline double* KTRISolver::dual() {
  if (&_lambdaValues.front() == NULL) {
    std::ostringstream message;
    message << "Error: dual variable array sent to solver is null";
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return &_lambdaValues.front();
}

inline double* KTRISolver::obj() {
  return &_obj;
}

inline int KTRISolver::solve(bool resetVariableBounds) {
  if (_initialSolveCall) {
    initCallbacks();
    _initialSolveCall = false;
  }

  return resolve(resetVariableBounds);
}


}


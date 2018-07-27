/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#ifndef NDEBUG
# define VERIFY_LENGTH(expectedSize, actualSize, functionName, elementType, fileName, line) verifyLength(expectedSize, actualSize, functionName, elementType, fileName, line)
#else
# define VERIFY_LENGTH(expectedSize, actualSize, functionName, elementType, fileName, line)
#endif

#include <sstream>

#include "knitro.h"
#include "KTRProblem.h"
#include "KTRMipNodeCallback.h"
#include "KTRMSProcessCallback.h"
#include "KTRMsInitptCallback.h"
#include "KTRNewptCallback.h"
#include "KTRPutString.h"

namespace knitro {

inline void verifyLength(int expectedSize, int actualSize, std::string const & functionName, std::string const & elementType,
                         std::string const &fileName, int line) {
  if (actualSize != expectedSize) {
    std::ostringstream message;
    message << "Error returned from " << functionName;
    message << "; attempted to copy values into " << elementType;
    message << "; source size: " << actualSize;
    message << "; destination size: " << expectedSize;
    throw KTRException(message.str(), fileName.c_str(), line);
  }
}  
 
inline KTRProblem::KTRProblem(int n, int m)
    : _lambdaInitial(0),
      _hessIndexCols(0),
      _hessIndexRows(0),
      _jacIndexCons(n * m),
      _jacIndexVars(n * m),
      _xInitial(0),
      _complementarityIndexList1(0),
      _complementarityIndexList2(0),
      _msProcessCallback(NULL),
      _mipNodeCallback(NULL),
      _newPointCallback(NULL),
      _msInitPtCallback(NULL),
      _putStringFunction(NULL),
      _objective(),
      _variables(n),
      _constraints(m) {
  /// If the user does not provide derivative size information
  /// use dense Jacobian. For Hessian, keep nnzH = 0 and NULL sparsity structure.
  setDenseJacobian();
}

inline KTRProblem::KTRProblem(int n, int m, int nnzJ)
    : _lambdaInitial(0),
      _hessIndexCols(0),
      _hessIndexRows(0),
      _jacIndexCons(nnzJ),
      _jacIndexVars(nnzJ),
      _xInitial(0),
      _complementarityIndexList1(0),
      _complementarityIndexList2(0),
      _msProcessCallback(NULL),
      _mipNodeCallback(NULL),
      _newPointCallback(NULL),
      _msInitPtCallback(NULL),
      _putStringFunction(NULL),
      _objective(),
      _variables(n),
      _constraints(m) {
}

inline KTRProblem::KTRProblem(int n, int m, int nnzJ, int nnzH)
    : _lambdaInitial(0),
      _hessIndexCols(nnzH),
      _hessIndexRows(nnzH),
      _jacIndexCons(nnzJ),
      _jacIndexVars(nnzJ),
      _xInitial(0),
      _complementarityIndexList1(0),
      _complementarityIndexList2(0),
      _msProcessCallback(NULL),
      _mipNodeCallback(NULL),
      _newPointCallback(NULL),
      _msInitPtCallback(NULL),
      _putStringFunction(NULL),
      _objective(),
      _variables(n),
      _constraints(m) {
}

inline const std::vector<int>& KTRProblem::complementarityIndexList1() const {
  return _complementarityIndexList1;
}

inline const std::vector<int>& KTRProblem::complementarityIndexList2() const {
  return _complementarityIndexList2;
}

inline void KTRProblem::setComplementarity(const std::vector<int>& indexList1, const std::vector<int>& indexList2) {
  if (indexList1.size() != indexList2.size()) {
    std::ostringstream msg;
    msg << "Error: indexList1 has length " << indexList1.size() << ", indexList2 has length " << indexList2.size();
    throw KTRException(msg.str(), "KTRProblem::SetComplementarity", __LINE__);

    //string.Format(
    //"Error: indexList1 has length {0}, indexList2 has length {1}", indexList1.Count, indexList2.Count));
  }

  _complementarityIndexList1 = indexList1;
  _complementarityIndexList2 = indexList2;
}

inline int KTRProblem::getObjType() const {
  return _objective.getType();
}

inline int KTRProblem::getObjFnType() const {
  return _objective.getFnType();
}

inline int KTRProblem::getObjGoal() const {
  return _objective.getGoal();
}

inline void KTRProblem::setObjType(int objType) {
  _objective.setType(objType);
}

inline void KTRProblem::setObjFnType(int objFnType) {
  _objective.setFnType(objFnType);
}

inline void KTRProblem::setObjGoal(int objGoal) {
  _objective.setGoal(objGoal);
}

inline const std::vector<int>& KTRProblem::getVarTypes() const {
  return _variables.getTypes();
}

inline void KTRProblem::setVarTypes(int id, int val) {
  _variables.setTypes(id, val);
}

inline void KTRProblem::setVarTypes(int val) {
  _variables.setTypes(val);
}

inline void KTRProblem::setVarTypes(const std::vector<int>& varTypes) {
  _variables.setTypes(varTypes);
}

inline const std::vector<double>& KTRProblem::getVarLoBnds() const {
  return _variables.getLoBnds();
}

inline void KTRProblem::setVarLoBnds(int id, double val) {
  _variables.setLoBnds(id, val);
}

inline void KTRProblem::setVarLoBnds(double val) {
  _variables.setLoBnds(val);
}

inline void KTRProblem::setVarLoBnds(const std::vector<double>& varLoBnds) {
  _variables.setLoBnds(varLoBnds);
}

inline const std::vector<double>& KTRProblem::getVarUpBnds() const {
  return _variables.getUpBnds();
}

inline void KTRProblem::setVarUpBnds(int id, double val) {
  _variables.setUpBnds(id, val);
}

inline void KTRProblem::setVarUpBnds(double val) {
  _variables.setUpBnds(val);
}

inline void KTRProblem::setVarUpBnds(const std::vector<double>& varUpBnds) {
  _variables.setUpBnds(varUpBnds);
}

inline const std::vector<int>& KTRProblem::getConTypes() const {
  return _constraints.getTypes();
}

inline void KTRProblem::setConTypes(int id, int val) {
  _constraints.setTypes(id, val);
}

inline void KTRProblem::setConTypes(int val) {
  _constraints.setTypes(val);
}

inline void KTRProblem::setConTypes(const std::vector<int>& conTypes) {
  _constraints.setTypes(conTypes);
}

inline const std::vector<int>& KTRProblem::getConFnTypes() const {
  return _constraints.getFnTypes();
}

inline void KTRProblem::setConFnTypes(int id, int val) {
  _constraints.setFnTypes(id, val);
}

inline void KTRProblem::setConFnTypes(int val) {
  _constraints.setFnTypes(val);
}

inline void KTRProblem::setConFnTypes(const std::vector<int>& conFnTypes) {
  _constraints.setFnTypes(conFnTypes);
}

inline const std::vector<double>& KTRProblem::getConLoBnds() const {
  return _constraints.getLoBnds();
}

inline void KTRProblem::setConLoBnds(int id, double val) {
  _constraints.setLoBnds(id, val);
}

inline void KTRProblem::setConLoBnds(double val) {
  _constraints.setLoBnds(val);
}

inline void KTRProblem::setConLoBnds(const std::vector<double>& conLoBnds) {
  _constraints.setLoBnds(conLoBnds);
}


inline const std::vector<double>& KTRProblem::getConUpBnds() const {
  return _constraints.getUpBnds();
}

inline void KTRProblem::setConUpBnds(int id, double val) {
  _constraints.setUpBnds(id, val);
}

inline void KTRProblem::setConUpBnds(double val) {
  _constraints.setUpBnds(val);
}

inline void KTRProblem::setConUpBnds(const std::vector<double>& conUpBnds) {
  _constraints.setUpBnds(conUpBnds);
}

inline const std::vector<int>& KTRProblem::getJacIndexCons() const {
  return _jacIndexCons;
}

inline void KTRProblem::setJacIndexCons(int id, int val) {
#ifndef NDEBUG
  _jacIndexCons.at(id) = val;
#else
  _jacIndexCons[id] = val;
#endif
}

inline void KTRProblem::setJacIndexCons(const std::vector<int>& jacIndexCons) {
  // Check length in debug mode
  VERIFY_LENGTH(getNNZJ(), static_cast<int>(jacIndexCons.size()), "KTRProblem::setJacIndexCons", "jacobian constraint indices", __FILE__, __LINE__);

  _jacIndexCons = jacIndexCons;
}

inline const std::vector<int>& KTRProblem::getJacIndexVars() const {
  return _jacIndexVars;
}

inline void KTRProblem::setJacIndexVars(int id, int val) {
#ifndef NDEBUG
  _jacIndexVars.at(id) = val;
#else
  _jacIndexVars[id] = val;
#endif
}

inline void KTRProblem::setJacIndexVars(const std::vector<int>& jacIndexVars) {
  // Check length in debug mode
  VERIFY_LENGTH(getNNZJ(), static_cast<int>(jacIndexVars.size()), "KTRProblem::setJacIndexVars", "jacobian variable indices", __FILE__, __LINE__);

  _jacIndexVars = jacIndexVars;
}

inline const std::vector<int>& KTRProblem::getHessIndexCols() const {
  return _hessIndexCols;
}

inline void KTRProblem::setHessIndexCols(int id, int val) {
#ifndef NDEBUG
  _hessIndexCols.at(id) = val;
#else
  _hessIndexCols[id] = val;
#endif
}

inline void KTRProblem::setHessIndexCols(const std::vector<int>& hessIndexCols) {
  // Check length in debug mode
  VERIFY_LENGTH(getNNZH(), static_cast<int>(hessIndexCols.size()), "KTRProblem::setHessIndexCols", "hessian column indices", __FILE__, __LINE__);

  _hessIndexCols = hessIndexCols;
}

inline const std::vector<int>& KTRProblem::getHessIndexRows() const {
  return _hessIndexRows;
}

inline void KTRProblem::setHessIndexRows(int id, int val) {
#ifndef NDEBUG
  _hessIndexRows.at(id) = val;
#else
  _hessIndexRows[id] = val;
#endif
}

inline void KTRProblem::setHessIndexRows(const std::vector<int>& hessIndexRows) {
  // Check length in debug mode
  VERIFY_LENGTH(getNNZH(), static_cast<int>(hessIndexRows.size()),  "KTRProblem::setHessIndexRows", "hessian row indices", __FILE__, __LINE__);

  _hessIndexRows = hessIndexRows;
}

inline int KTRProblem::getNumVars() const {
  return _variables.size();
}

inline int KTRProblem::getNumCons() const {
  return _constraints.size();
}

inline int KTRProblem::getNumCompCons() const {
  return static_cast<int>(_complementarityIndexList1.size());
}

inline int KTRProblem::getNNZJ() const {
  return static_cast<int>(_jacIndexVars.size());
}

inline int KTRProblem::getNNZH() const {
  return static_cast<int>(_hessIndexCols.size());
}

inline bool KTRProblem::isMipProblem() const {

  std::vector<int> vartypes = getVarTypes();

  // var types may not be set (they're not needed if all variables are continuous).
  if (vartypes.size() == 0) {
    return false;
  }

  for (int i = 0; i < getNumVars(); i++) {
    if (vartypes[i] == KTREnums::VariableType::Binary || vartypes[i] == KTREnums::VariableType::Integer) {
      return true;
    }
  }
  return false;
}

inline const std::vector<double>& KTRProblem::getXInitial() const {
  return _xInitial;
}

inline void KTRProblem::setXInitial(int id, double val) {
  if (_xInitial.empty()) {
    _xInitial.resize(getNumVars());
  }

#ifndef NDEBUG
  _xInitial.at(id) = val;
#else
  _xInitial[id] = val;
#endif
}

inline void KTRProblem::setXInitial(const std::vector<double>& xInitial) {
  // Check length in debug mode
  VERIFY_LENGTH(getNumVars(), static_cast<int>(xInitial.size()), "KTRProblem::setXInitial", "variable value", __FILE__, __LINE__);

  _xInitial = xInitial;
}

inline const std::vector<double>& KTRProblem::getLambdaInitial() const {
  return _lambdaInitial;
}

inline void KTRProblem::setLambdaInitial(int id, double val) {
  if (_lambdaInitial.empty()) {
    _lambdaInitial.resize(getNumVars());
  }

#ifndef NDEBUG
  _lambdaInitial.at(id) = val;
#else
  _lambdaInitial[id] = val;
#endif
}

inline void KTRProblem::setLambdaInitial(const std::vector<double>& lambdaInitial) {
  // Check length in debug mode
  VERIFY_LENGTH(getNumVars() + getNumCons(), static_cast<int>(lambdaInitial.size()),
      "KTRProblem::setLambdaInitial", "lambda value", __FILE__, __LINE__);

  _lambdaInitial = lambdaInitial;
}

inline double KTRProblem::evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
									std::vector<double>& jac) {			
	return KTR_RC_CALLBACK_ERR;
}

inline int KTRProblem::evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad,
                                  std::vector<double>& jac) {
  return KTR_RC_CALLBACK_ERR;
}

inline int KTRProblem::evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                                    std::vector<double>& hess) {

  return KTR_RC_CALLBACK_ERR;
}

inline int KTRProblem::evaluateHessianVector(const std::vector<double>& x, double objScaler,
                                             const std::vector<double>& lambda, std::vector<double>& vector) {

  return KTR_RC_CALLBACK_ERR;
}

inline int KTRProblem::putStringFunctionWrapper(const std::string & str, KTRISolver* solver) {
  if (_putStringFunction == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _putStringFunction->CallbackFunction(str.c_str(), (KTRSolver*)solver);
}

inline int KTRProblem::mipNodeCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda,
                                              double obj, std::vector<double>& c, std::vector<double>& objGrad,
                                              std::vector<double>& jac, std::vector<double>& hessian,
                                              std::vector<double>& hessVector, KTRISolver* solver) {
  if (_mipNodeCallback == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _mipNodeCallback->CallbackFunction(x, lambda, obj, c, (KTRSolver*)solver);
}

inline int KTRProblem::msProcessCallbackWrapper(int evalRequestCode, std::vector<double>& x,
                                                std::vector<double>& lambda, double obj, std::vector<double>& c,
                                                std::vector<double>& objGrad, std::vector<double>& jac,
                                                std::vector<double>& hessian, std::vector<double>& hessVector,
                                                KTRISolver* solver) {
  if (_msProcessCallback == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _msProcessCallback->CallbackFunction(x, lambda, obj, c, (KTRSolver*)solver);
}

inline int KTRProblem::msInitPtCallbackWrapper(int nSolveNumber, const std::vector<double>& xLoBnds,
                                               const std::vector<double>& xUpBnds, std::vector<double>& x,
                                               std::vector<double>& lambda, KTRISolver* solver) {
  if (_msInitPtCallback == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _msInitPtCallback->CallbackFunction(nSolveNumber, 
                                             xLoBnds, xUpBnds, x, lambda, 
                                             solver);
}

inline int KTRProblem::newPointCallbackWrapper(KTR_context_ptr kc, const std::vector<double>& x,
                                               const std::vector<double>& lambda, double obj,
                                               const std::vector<double>& c, const std::vector<double>& objGrad,
                                               const std::vector<double>& jac, KTRISolver* solver) {
  if (_newPointCallback == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _newPointCallback->CallbackFunction(x, lambda, 
                                             obj, c, objGrad, jac, (KTRSolver*)solver);
}

inline void KTRProblem::setNewPointCallback(KTRNewptCallback* newPointCallback) {
  _newPointCallback = newPointCallback;
}

inline void KTRProblem::setMSProcessCallback(KTRMSProcessCallback* MSProcessCallback) {
  _msProcessCallback = MSProcessCallback;
}

inline void KTRProblem::setPutStringFunction(KTRPutString* putStringFunction) {
  _putStringFunction = putStringFunction;
}

inline void KTRProblem::setMSInitPtCallback(KTRMSInitptCallback* MSInitPtCallback) {
  _msInitPtCallback = MSInitPtCallback;
}

inline void KTRProblem::setMipNodeCallback(KTRMipNodeCallback* mipNodeCallback) {
  _mipNodeCallback = mipNodeCallback;
}

inline void KTRProblem::setDenseJacobian() {
  for (int i = 0; i < getNumVars(); i++) {
    for (int j = 0; j < getNumCons(); j++) {
#ifndef NDEBUG
      _jacIndexCons.at(j*getNumVars() + i) = j;
      _jacIndexVars.at(j*getNumVars() + i) = i;
#else
      _jacIndexCons[j * getNumVars() + i] = j;
      _jacIndexVars[j * getNumVars() + i] = i;
#endif
    }
  }
}

inline KTRProblem::~KTRProblem() {
}

}


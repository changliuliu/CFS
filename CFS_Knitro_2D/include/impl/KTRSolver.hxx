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

#include "KTRSolver.h"
#include "KTRException.h"
#include "KTRProblem.h"

namespace knitro {

class KTRIProblem;

inline KTRSolver::KTRSolver(KTRIProblem * problem, int gradopt, int hessopt)
    : KTRISolver(problem, gradopt, hessopt),
      _integralityRelaxed(false),
      _useMipNodeCallback(false) {
  initProblem();
}

inline KTRSolver::KTRSolver(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt)
    : KTRISolver(zlm, problem, gradopt, hessopt),
      _integralityRelaxed(false),
      _useMipNodeCallback(false) {
  initProblem();
}

inline KTRSolver::KTRSolver(KTRIProblem * problem)
    : KTRISolver(problem),
      _integralityRelaxed(false),
      _useMipNodeCallback(false) {
  initProblem();
}

inline KTRSolver::KTRSolver(const ZLM * zlm, KTRIProblem * problem)
    : KTRISolver(zlm, problem),
      _integralityRelaxed(false),
      _useMipNodeCallback(false) {
  initProblem();
}

inline KTRSolver::~KTRSolver() {

}

inline void KTRSolver::useMipNodeCallback() {
  _useMipNodeCallback = true;
}

inline void KTRSolver::setConScaling(const std::vector<double>& cScaleFactors, const std::vector<double>& ccScaleFactors){
  int retCode = KTR_set_con_scalings(_kc, cScaleFactors.data(), ccScaleFactors.data());
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_con_scalings; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::setIntVarStrategy(const int xIndex, const int xStrategy){
  int retCode = KTR_mip_set_intvar_strategy(_kc, xIndex, xStrategy);
  if (retCode != 0){
    std::ostringstream message;
    message << "Error returned from KTR_set_obj_scalings; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::setFeasTols(const std::vector<double>& cFeasTols, const std::vector<double>& xFeasTols,
                                   const std::vector<double>& ccFeasTols) {
  if (cFeasTols.size() != 0 && cFeasTols.size() != _problem->getNumCons()) {
    std::ostringstream message;

    message << "cFeastols has length " << cFeasTols.size() << ". Expected length " << _problem->getNumCons();

    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (xFeasTols.size() != 0 && xFeasTols.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xFeasTols has length " << xFeasTols.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (ccFeasTols.size() != 0 && ccFeasTols.size() != _problem->getNumCompCons()) {
    std::ostringstream message;
    message << "ccFeasTols has length " << ccFeasTols.size() << ". Expected length " << _problem->getNumCompCons();

    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_set_feastols(_kc, cFeasTols.size() == 0 ? NULL : &cFeasTols[0],
                                 xFeasTols.size() == 0 ? NULL : &xFeasTols[0],
                                 ccFeasTols.size() == 0 ? NULL : &ccFeasTols[0]);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_feastols; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::setNames(const std::string& objName, const std::vector<std::string>& varNames,
                                const std::vector<std::string>& conNames) {
  if (varNames.size() != 0 && varNames.size() != _problem->getNumVars()) {
    std::ostringstream message;

    message << "varNames has length " << varNames.size() << ". Expected length " << _problem->getNumVars();

    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (conNames.size() != 0 && conNames.size() != _problem->getNumCons()) {
    std::ostringstream message;

    message << "conNames has length " << conNames.size() << ". Expected length " << _problem->getNumCons();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  std::vector<char *> tmpVarNames(varNames.size());
  for (size_t i(0); i < varNames.size(); ++i)
    tmpVarNames[i] = convert(varNames[i]);
//  transform(varNames.begin(), varNames.end(), back_inserter(tmpVarNames), convert);

  std::vector<char *> tmpConNames(conNames.size());
  for (size_t i(0); i < conNames.size(); ++i)
    tmpConNames[i] = convert(conNames[i]);

//  transform(conNames.begin(), conNames.end(), back_inserter(tmpVarNames), convert);

  int retCode = KTR_set_names(_kc, objName.size() == 0 ? NULL : objName.c_str(),
                              tmpVarNames.size() == 0 ? NULL : &tmpVarNames.front(),
                              tmpConNames.size() == 0 ? NULL : &tmpConNames.front());

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_names";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::restart(const std::vector<double>& xInitial, const std::vector<double>& lambdaInitial) {
  if (xInitial.size() != 0 && xInitial.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xInitial has length " << xInitial.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (lambdaInitial.size() != 0 && lambdaInitial.size() != _problem->getNumVars() + _problem->getNumCons()) {
    std::ostringstream message;
    message << "lambdaInitial has length " << lambdaInitial.size() << ". Expected length "
            << _problem->getNumVars() + _problem->getNumCons();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_restart(_kc, xInitial.size() == 0 ? NULL : &xInitial[0],
                            lambdaInitial.size() == 0 ? NULL : &lambdaInitial[0]);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_restart";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRSolver::getNumberFCEvals() const {
  int numEvals = KTR_get_number_FC_evals(_kc);
  if (numEvals < 0) {
    std::ostringstream message;
    message << "Negative value returned from get_number_FC_evals";
    message << "; value: " << numEvals;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return numEvals;
}

inline int KTRSolver::getNumberGAEvals() const {
  int numEvals = KTR_get_number_FC_evals(_kc);
  if (numEvals < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_number_GA_evals";
    message << "; value: " << numEvals;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numEvals;
}

inline int KTRSolver::getNumberHEvals() const {
  int numEvals = KTR_get_number_H_evals(_kc);
  if (numEvals < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_number_H_evals";
    message << "; value: " << numEvals;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numEvals;
}

inline int KTRSolver::getNumberHVEvals() const {
  int numEvals = KTR_get_number_HV_evals(_kc);
  if (numEvals < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_number_HV_evals";
    message << "; value: " << numEvals;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numEvals;
}

inline double KTRSolver::getAbsFeasError() const {
  double error = KTR_get_abs_feas_error(_kc);
  if (error < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_abs_feas_error";
    message << "; value: " << error;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return error;
}

inline double KTRSolver::getRelFeasError() const {
  double error = KTR_get_rel_feas_error(_kc);
  if (error < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_rel_feas_error";
    message << "; value: " << error;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return error;
}

inline std::vector<double> KTRSolver::getConstraintValues() const {
  std::vector<double> c(_problem->getNumCons());

  int retCode = KTR_get_constraint_values(_kc, &c[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from get_constraint_values";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return c;
}

inline std::vector<double> KTRSolver::getJacobianValues() const {
  std::vector<double> jac(_problem->getNNZJ());
  int retCode = KTR_get_jacobian_values(_kc, &jac[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_jacobian_values";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return jac;
}

inline std::vector<double> KTRSolver::getHessianValues() const {
  int nnzh;

  int hessopt = getIntParam(KTR_PARAM_HESSOPT);
  if (hessopt == KTR_HESSOPT_BFGS || hessopt == KTR_HESSOPT_SR1) {
    // dense Hessian pattern
    int n = _problem->getNumVars();
    nnzh = (n*n + n) / 2;

    if (getIntParam(KTR_PARAM_PRESOLVE) != KTR_PRESOLVE_NONE) {
      std::ostringstream message;
      message << "Error returned from KTR_get_hessian_values:";
      message << " cannot get hessian values when using BFGS or SR1 and presolve is activated";
      throw KTRException(message.str(), __FILE__, __LINE__);
    }
  } else if (hessopt == KTR_HESSOPT_EXACT) {
      // User sparsity pattern
      nnzh = _problem->getNNZH();
  } else {
      std::ostringstream message;
      message << "Error returned from KTR_get_hessian_values:";
      message << " cannot get hessian values when using KTR_PARAM_HESSOPT = " << hessopt;
      throw KTRException(message.str(), __FILE__, __LINE__);
  }

  std::vector<double> hess(nnzh);
  int retCode = KTR_get_hessian_values(_kc, &hess[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_hessian_values";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return hess;
}

inline void KTRSolver::mipSetBranchingPriorities(const std::vector<int>& xPriorities) {
  if (xPriorities.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xPriorities has length " << xPriorities.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_mip_set_branching_priorities(_kc, &xPriorities[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_mip_set_branching_priorities";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRSolver::getMipNumNodes() const {
  int numNodes = KTR_get_mip_num_nodes(_kc);
  if (numNodes < 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_mip_num_nodes";
    message << "; error code: " + numNodes;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numNodes;
}

inline int KTRSolver::getMipNumSolves() const {
  int numSolves = KTR_get_mip_num_solves(_kc);
  if (numSolves < 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_mip_num_solves";
    message << "; error code: " + numSolves;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return numSolves;
}

inline double KTRSolver::getMipAbsGap() const {
  double gap = KTR_get_mip_abs_gap(_kc);
  if (gap < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_mip_abs_gap";
    message << "; value: " << gap;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return gap;
}

inline double KTRSolver::getMipRelGap() const {
  double gap = KTR_get_mip_rel_gap(_kc);
  if (gap < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_mip_rel_gap";
    message << "; value: " << gap;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return gap;
}

inline double KTRSolver::getMipIncumbentObj() const {
  return KTR_get_mip_incumbent_obj(_kc);
}

inline double KTRSolver::getMipRelaxationBnd() const {
  double bound = KTR_get_mip_relaxation_bnd(_kc);
  if (bound < 0) {
    std::ostringstream message;
    message << "Negative value returned from KTR_get_mip_relaxation_bnd";
    message << "; value: " << bound;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return bound;
}

inline double KTRSolver::getMipLastNodeObj() const {
  return KTR_get_mip_lastnode_obj(_kc);
}

inline std::vector<double> KTRSolver::getMipIncumbentX() const {
  std::vector<double> incumbent(_problem->getNumVars());

  int retCode = KTR_get_mip_incumbent_x(_kc, &incumbent[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_mip_incumbent_x; Error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return incumbent;
}

inline void KTRSolver::setIntegralityRelaxed(bool integralityRelaxed) {
  _integralityRelaxed = integralityRelaxed;
}

inline void KTRSolver::setMipNodeCallback() {
  int retCode = KTR_set_mip_node_callback(_kc, callbackMipNode);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_mip_node_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::setMSInitptCallback() {
  int retCode = KTR_set_ms_initpt_callback(_kc, callbackMSInitpt);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_ms_initpt_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRSolver::resolve(bool resetVariableBounds) {
  int returnCode;
  int evalStatus = 0;

  if (resetVariableBounds) {
    chgVarBnds(_problem->getVarLoBnds(), _problem->getVarUpBnds());
  }

  if (isMipProblem()) {
    returnCode = KTR_mip_solve(_kc, var(), dual(), evalStatus, obj(), NULL, NULL, NULL, NULL, NULL,
                               static_cast<void*>(this));
  } else {
    returnCode = KTR_solve(_kc, var(), dual(), evalStatus, obj(), NULL, NULL, NULL, NULL, NULL,
                           static_cast<void*>(this));
  }

  return returnCode;
}

inline void KTRSolver::addCompCons() {

  KTRProblem* prob = dynamic_cast<KTRProblem*>(_problem);
  if (prob == NULL) {
    return;
  }

  std::vector<int> indexList1 = prob->complementarityIndexList1();
  std::vector<int> indexList2 = prob->complementarityIndexList2();

  if (indexList1.size() != indexList2.size()) {
    std::ostringstream message;
    message << "indexList1 has length " << indexList1.size() << ". indexList2 has length " << indexList2.size();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  int retCode = KTR_addcompcons(_kc, static_cast<int>(indexList1.size()), &indexList1[0], &indexList2[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_addcompcons";
    message << "; numCompConstraints: " + indexList1.size();
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolver::initCallbacks() {
  if (this->_useMipNodeCallback) {
    this->setMipNodeCallback();
  }

  if (this->_useMSInitptCallback) {
    this->setMSInitptCallback();
  }

  if (this->_useMSProcessCallback) {
    this->setMSProcessCallback();
  }

  if (this->_useNewptCallback) {
    this->setNewptCallback();
  }

  int retCode = KTR_set_func_callback(_kc, callbackEvalFC);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_func_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  retCode = KTR_set_grad_callback(_kc, callbackEvalGA);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_func_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  retCode = KTR_set_hess_callback(_kc, callbackEvalHess);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_func_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline bool KTRSolver::isMipProblem() const {
  if (_integralityRelaxed)
    return false;

  return _problem->isMipProblem();
}

inline int KTRSolver::callbackEvalFC(int evalRequestCode, int n, int m, int nnzJ, int nnzH, const double* const x,
                                     const double* const lambda, double* const obj, double* const c,
                                     double* const objGrad, double* const jac, double* const hessian,
                                     double* const hessVector, void* userParams) {
  if (evalRequestCode != KTR_RC_EVALFC) {
    std::cout << "*** callbackEvalFC incorrectly called with eval code " << evalRequestCode << std::endl;
    return KTR_RC_CALLBACK_ERR;
  }

  KTRSolver* solver = static_cast<KTRSolver*>(userParams);
  int gradopt = solver->getIntParam(KTR_PARAM_GRADOPT);
  size_t gradSize = (size_t)n, jacSize = (size_t)nnzJ;

  // Altering jacobian and gradient sizes so that they are copied only if necessary
  if (gradopt != KTR_GRADOPT_EXACT) {
      gradSize = 0;
      jacSize = 0;
  }

  // Copying x
  std::vector<double> xVec(x, x + n);

  // Reserving size for user inputs
  std::vector<double> cVec(m);
  std::vector<double> objGradVec(gradSize);
  std::vector<double> jacVec(jacSize);

  int status = solver->getProblem()->evaluateFC(xVec, *obj, cVec, objGradVec, jacVec);

  for(size_t i(0); i < (size_t)m; ++i)
    c[i] = cVec[i];
  
  for (size_t i(0); i < gradSize; ++i)
    objGrad[i] = objGradVec[i];
  
  for (size_t i(0); i < jacSize; ++i)
    jac[i] = jacVec[i];

  return status;
}

inline int KTRSolver::callbackMSInitpt(int nSolveNumber, int n, int m, double const* const xLoBnds,
                                       double const* const xUpBnds, double* const x, double* const lambda,
                                       void* const userParams) {

  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  std::vector<double> xLoBndsVec(xLoBnds, xLoBnds + n);
  std::vector<double> xUpBndsVec(xUpBnds, xUpBnds + n);
  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n + m);

  int returnVal = solver->getProblem()->msInitPtCallbackWrapper(nSolveNumber, 
                                                                xLoBndsVec, xUpBndsVec, 
                                                                xVec, lambdaVec,
                                                                solver);

  for(size_t i(0); i<xVec.size();++i)
    x[i] = xVec[i];
  for(size_t i(0); i<lambdaVec.size();++i)
    lambda[i] = lambdaVec[i];
  return returnVal;
}

inline void KTRSolver::setMSProcessCallback() {
  int retCode = KTR_set_ms_process_callback(_kc, callbackMSProcess);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_ms_process_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRSolver::callbackMSProcess(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                        double const* const lambda, double* const obj, double* const c,
                                        double* const objGrad, double* const jac, double* const hessian,
                                        double* const hessVector, void* userParams) {
  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n + m);
  std::vector<double> cVec(c, c + m);

  std::vector<double> objGradVec;

  if (objGrad != NULL) {
    objGradVec = std::vector<double>(objGrad, objGrad + n);
  }

  std::vector<double> jacVec;

  if (jac != NULL) {
    jacVec = std::vector<double>(jac, jac + nnzJ);
  }

  std::vector<double> hessianVec;

  if (hessian != NULL) {
    hessianVec = std::vector<double>(hessian, hessian + nnzH);
  }

  std::vector<double> hessVectorVec;

  if (hessVector != NULL) {
    hessVectorVec = std::vector<double>(hessVector, hessVector + n);
  }

  return solver->getProblem()->msProcessCallbackWrapper(evalRequestCode, xVec, lambdaVec, *obj, cVec, objGradVec,
                                                        jacVec, hessianVec, hessVectorVec, solver);
}

inline void KTRSolver::setNewptCallback() {
  int retCode = KTR_set_newpt_callback(_kc, callbackNewPt);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_newpt_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}


inline int KTRSolver::callbackMipNode(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                      double const* const lambda, double* const obj, double* const c,
                                      double* const objGrad, double* const jac, double* const hessian,
                                      double* const hessVector, void* userParams) {
  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n + m);
  std::vector<double> cVec(c, c + m);

  std::vector<double> objGradVec;
  if (objGrad != NULL) {
    objGradVec = std::vector<double>(objGrad, objGrad + n);
  }

  std::vector<double> jacVec;
  if (jac != NULL) {
    jacVec = std::vector<double>(jac, jac + nnzJ);
  }

  std::vector<double> hessianVec;
  if (hessian != NULL) {
    hessianVec = std::vector<double>(hessian, hessian + nnzH);
  }

  std::vector<double> hessVectorVec;
  if (hessVector != NULL) {
    hessVectorVec = std::vector<double>(hessVector, hessVector + n);
  }

  return solver->getProblem()->mipNodeCallbackWrapper(evalRequestCode, xVec, lambdaVec, *obj, cVec, objGradVec, jacVec,
                                                      hessianVec, hessVectorVec, solver);
}

inline int KTRSolver::callbackNewPt(KTR_context_ptr kc, int n, int m, int nnzJ, double const* const x,
                                    double const* const lambda, double obj, double const* const c,
                                    double const* const objGrad, double const* const jac, void* userParams) {
  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n + m);
  std::vector<double> cVec(c, c + m);
  std::vector<double> objGradVec(objGrad, objGrad + n);
  std::vector<double> jacVec(jac, jac + nnzJ);

  return solver->getProblem()->newPointCallbackWrapper(kc, xVec, lambdaVec, obj, cVec, objGradVec, jacVec, solver);
}

inline int KTRSolver::callbackRedirectOutput(const char* const str, void* userParams) {
  KTRSolver* solver = static_cast<KTRSolver*>(userParams);
  return solver->getProblem()->putStringFunctionWrapper(str, solver);
}

inline int KTRSolver::callbackEvalGA(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                     double const* const lambda, double* const obj, double* const c,
                                     double* const objGrad, double* const jac, double* const hessian,
                                     double* const hessVector, void* userParams) {
  if (evalRequestCode != KTR_RC_EVALGA) {
    std::cout << "*** callbackEvalGA incorrectly called with eval code " << evalRequestCode << std::endl;
    return KTR_RC_CALLBACK_ERR;
  }

  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> objGradVec(n);
  std::vector<double> jacVec(nnzJ);

  int returnVal = solver->getProblem()->evaluateGA(xVec, objGradVec, jacVec);

  if (returnVal == KTR_RC_EVALFCGA) {
    // No copy to perform, this has been done in evalfc already
    // KTR_RC_EVALFCGA is internal to C++ interface and callback worked
    // as expected so we return 0.
    return 0;
  }

  for(size_t i(0); i<objGradVec.size();++i)
    objGrad[i] = objGradVec[i];
  for(size_t i(0); i<jacVec.size();++i)
    jac[i] = jacVec[i];

  return returnVal;
}

inline int KTRSolver::callbackEvalHess(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                       double const* const lambda, double* const obj, double* const c,
                                       double* const objGrad, double* const jac, double* const hessian,
                                       double* const hessVector, void* userParams) {

  KTRSolver* solver = static_cast<KTRSolver*>(userParams);

  int returnCode = KTR_RC_CALLBACK_ERR;

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n + m);
  std::vector<double> hessianVec(hessian, hessian + nnzH);
  std::vector<double> hessVectorVec(hessVector, hessVector + n);

  switch (evalRequestCode) {
    case KTR_RC_EVALH:
      returnCode = solver->getProblem()->evaluateHess(xVec, 1.0, lambdaVec, hessianVec);
      break;
    case KTR_RC_EVALH_NO_F:
      returnCode = solver->getProblem()->evaluateHess(xVec, 0.0, lambdaVec, hessianVec);
      break;
    case KTR_RC_EVALHV:
      returnCode = solver->getProblem()->evaluateHessianVector(xVec, 1.0, lambdaVec, hessVectorVec);
      break;
    case KTR_RC_EVALHV_NO_F:
      returnCode = solver->getProblem()->evaluateHessianVector(xVec, 0.0, lambdaVec, hessVectorVec);
      break;
    default:
      std::cout << "*** callbackEvalHess incorrectly called with eval code " << evalRequestCode << std::endl;
  }

  for(size_t i(0); i<hessianVec.size();++i)
    hessian[i] = hessianVec[i];
  for(size_t i(0); i<hessVectorVec.size();++i)
    hessVector[i] = hessVectorVec[i];
  return returnCode;
}

inline void KTRSolver::initProblem() {
  int returnCode;

  KTRProblem* prob = dynamic_cast<KTRProblem*>(_problem);
  if ( prob == NULL ) {
    return; 
  }

  if (isMipProblem()) {
    returnCode = KTR_mip_init_problem(
        _kc, prob->getNumVars(), prob->getObjGoal(), prob->getObjType(), prob->getObjFnType(),
        &prob->getVarTypes()[0], prob->getVarLoBnds().size() == 0 ? NULL : &prob->getVarLoBnds()[0],
        prob->getVarUpBnds().size() == 0 ? NULL : &prob->getVarUpBnds()[0], prob->getNumCons(),
        prob->getNumCons() == 0 ? NULL : &prob->getConTypes()[0],
        prob->getNumCons() == 0 ? NULL : &prob->getConFnTypes()[0],
        prob->getNumCons() == 0 ? NULL : &prob->getConLoBnds()[0],
        prob->getNumCons() == 0 ? NULL : &prob->getConUpBnds()[0], prob->getNNZJ(),
        prob->getNNZJ() == 0 ? NULL : &prob->getJacIndexVars()[0],
        prob->getNNZJ() == 0 ? NULL : &prob->getJacIndexCons()[0], _problem->getNNZH(),
        prob->getNNZH() == 0 ? NULL : &prob->getHessIndexRows()[0],
        prob->getNNZH() == 0 ? NULL : &prob->getHessIndexCols()[0],
        prob->getXInitial().size() == 0 ? NULL : &prob->getXInitial()[0],
        prob->getLambdaInitial().size() == 0 ? NULL : &prob->getLambdaInitial()[0]);
  } else {
    returnCode = KTR_init_problem(_kc, prob->getNumVars(), prob->getObjGoal(), prob->getObjType(),
                                  prob->getVarLoBnds().size() == 0 ? NULL : &prob->getVarLoBnds()[0],
                                  prob->getVarUpBnds().size() == 0 ? NULL : &prob->getVarUpBnds()[0],
                                  prob->getNumCons(),
                                  prob->getNumCons() == 0 ? NULL : &prob->getConTypes()[0],
                                  prob->getNumCons() == 0 ? NULL : &prob->getConLoBnds()[0],
                                  prob->getNumCons() == 0 ? NULL : &prob->getConUpBnds()[0],
                                  prob->getNNZJ(),
                                  prob->getNNZJ() == 0 ? NULL : &prob->getJacIndexVars()[0],
                                  prob->getNNZJ() == 0 ? NULL : &prob->getJacIndexCons()[0],
                                  prob->getNNZH(),
                                  prob->getNNZH() == 0 ? NULL : &prob->getHessIndexRows()[0],
                                  prob->getNNZH() == 0 ? NULL : &prob->getHessIndexCols()[0],
                                  prob->getXInitial().size() == 0 ? NULL : &prob->getXInitial()[0],
                                  prob->getLambdaInitial().size() == 0 ? NULL : &prob->getLambdaInitial()[0]);
  }

  if (returnCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_init_problem";
    message << "; error code: " + returnCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (prob->getNumCompCons() > 0) {
    addCompCons();
  }
}

}


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

#include "KTRSolverLSQ.h"
#include "KTRException.h"
#include "KTRProblemLSQ.h"

namespace knitro {

inline KTRSolverLSQ::KTRSolverLSQ(KTRIProblem * problem, int gradopt, int hessopt)
    : KTRISolver(problem) {
  construct(gradopt, hessopt);
  initProblem();
}

inline KTRSolverLSQ::KTRSolverLSQ(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt)
    : KTRISolver(problem) {
  construct(gradopt, hessopt);
  initProblem();
}

inline KTRSolverLSQ::KTRSolverLSQ(KTRIProblem * problem)
    : KTRISolver(problem) {
  construct();
  initProblem();
}

inline KTRSolverLSQ::KTRSolverLSQ(const ZLM * zlm, KTRIProblem * problem)
    : KTRISolver(problem) {
  if (_problem->putStringFunctionWrapper("", this) != KTR_RC_CALLBACK_ERR) {
    _kc = KTR_new_zlm(callbackRedirectOutput, static_cast<void*>(this), zlm->getLicenseManager());
  } else {
    _kc = KTR_new_zlm(NULL, static_cast<void*>(this), zlm->getLicenseManager());
  }
  construct();
  initProblem();
}

inline KTRSolverLSQ::~KTRSolverLSQ() {
}

inline void KTRSolverLSQ::setNames(const std::string& objName, 
                                   const std::vector<std::string>& varNames) {
  if (varNames.size() != 0 && varNames.size() != _problem->getNumVars()) {
    std::ostringstream message;

    message << "varNames has length " << varNames.size() << ". Expected length " << _problem->getNumVars();

    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  std::vector<char *> tmpVarNames(varNames.size());
  for (size_t i(0); i < varNames.size(); ++i)
    tmpVarNames[i] = convert(varNames[i]);
//  transform(varNames.begin(), varNames.end(), back_inserter(tmpVarNames), convert);

  int retCode = KTR_set_names(_kc, objName.size() == 0 ? NULL : objName.c_str(),
                              tmpVarNames.size() == 0 ? NULL : &tmpVarNames.front(),
                              NULL);

  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_names";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolverLSQ::restart(const std::vector<double>& xInitial, const std::vector<double>& lambdaInitial) {
  if (xInitial.size() != 0 && xInitial.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "xInitial has length " << xInitial.size() << ". Expected length " << _problem->getNumVars();
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  if (lambdaInitial.size() != 0 && lambdaInitial.size() != _problem->getNumVars()) {
    std::ostringstream message;
    message << "lambdaInitial has length " << lambdaInitial.size() << ". Expected length "
            << _problem->getNumVars();
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

inline std::vector<double> KTRSolverLSQ::getJacobianValues() const {
  std::vector<double> jac(_problem->getNNZJ());
  int retCode = KTR_lsq_get_jacobian_values(_kc, &jac[0]);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_get_jacobian_values";
    message << "; error code: " << retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  return jac;
}

inline std::vector<double> KTRSolverLSQ::getHessianValues() const {
  int nnzh;

  int hessopt = getIntParam(KTR_PARAM_HESSOPT);
  if (hessopt == KTR_HESSOPT_BFGS || hessopt == KTR_HESSOPT_SR1
                                  || hessopt == KTR_HESSOPT_GAUSS_NEWTON) {
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


inline void KTRSolverLSQ::setMSInitptCallback() {
  int retCode = KTR_set_ms_initpt_callback(_kc, callbackMSInitpt);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_ms_initpt_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolverLSQ::setMSProcessCallback() {
  int retCode = KTR_set_ms_process_callback(_kc, callbackMSProcess);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_ms_process_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline void KTRSolverLSQ::setNewptCallback() {
  int retCode = KTR_set_newpt_callback(_kc, callbackNewPt);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_set_newpt_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline int KTRSolverLSQ::resolve(bool resetVariableBounds) {
  int returnCode;
  int evalStatus = 0;
  if (resetVariableBounds) {
    chgVarBnds(_problem->getVarLoBnds(), _problem->getVarUpBnds());
  }

  returnCode = KTR_solve(_kc, var(), dual(), evalStatus, obj(), NULL, NULL, NULL, NULL, NULL,
                          static_cast<void*>(this));
  return returnCode;
}

inline void KTRSolverLSQ::initCallbacks() {

  if (this->_useMSInitptCallback) {
    this->setMSInitptCallback();
  }

  if (this->_useMSProcessCallback) {
    this->setMSProcessCallback();
  }

  if (this->_useNewptCallback) {
    this->setNewptCallback();
  }

  int retCode = KTR_lsq_set_res_callback(_kc, callbackEvalRes);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_lsq_set_res_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
  retCode = KTR_lsq_set_jac_callback(_kc, callbackEvalJac);
  if (retCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_lsq_set_jac_callback";
    message << "; error code: " + retCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

inline bool KTRSolverLSQ::isMipProblem() const {
  return false;
}

inline int KTRSolverLSQ::callbackEvalRes(   	int            n,
												                  int            m,
												                int            nnzJ,
                                         const double * const  x,
                                               double * const  res,
                                               double * const  jac,
                                               void *          userParams) {

  KTRSolverLSQ* solver = static_cast<KTRSolverLSQ*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> rVec(res, res+m);

  KTRProblemLSQ* lsqProb = dynamic_cast<KTRProblemLSQ*>(solver->getProblem());
  if (lsqProb == NULL) {
    std::cout << "*** Error casting KTRIProblem* to KTRProblem*." << std::endl;
    return KTR_RC_CALLBACK_ERR;
  }

  int out = lsqProb->evaluateResidual(xVec, rVec);

  for(size_t i(0); i<rVec.size();++i)
    res[i] = rVec[i];
  																
	return( 0 );
}

inline int KTRSolverLSQ::callbackEvalJac(       int            n,
                                                int            m,
                                                int            nnzJ,
                                          const double * const x,
                                                double * const res,
                                                double * const jac,
                                                void *         userParams) {
													
  KTRSolverLSQ* solver = static_cast<KTRSolverLSQ*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> jVec(jac, jac + nnzJ);

  KTRProblemLSQ* lsqProb = dynamic_cast<KTRProblemLSQ*>(solver->getProblem());
  if (lsqProb == NULL) {
    std::cout << "*** Error casting KTRIProblem* to KTRProblem*." << std::endl;
    return KTR_RC_CALLBACK_ERR;
  }

  int out = lsqProb->evaluateJacobian(xVec, jVec);

  for (size_t i(0) ; i<jVec.size() ; ++i) {
    jac[i] = jVec[i];
  }

  return( 0 );
}

inline int KTRSolverLSQ::callbackNewPt(KTR_context_ptr kc, int n, int m, int nnzJ, double const* const x,
                                        double const* const lambda, double obj, double const* const c, double const* const objGrad,
                                        double const* const jac, void* userParams) {
  KTRSolverLSQ* solver = static_cast<KTRSolverLSQ*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n);
  std::vector<double> cVec;
  std::vector<double> objGradVec(objGrad, objGrad + n);
  std::vector<double> jacVec(jac, jac + nnzJ);

  return solver->getProblem()->newPointCallbackWrapper(kc, xVec, lambdaVec, obj, cVec, objGradVec, jacVec, solver);
}

inline int KTRSolverLSQ::callbackMSInitpt(int nSolveNumber, int n, int m, double const* const xLoBnds,
                                          double const* const xUpBnds, double* const x, double* const lambda,
                                          void* const userParams) {

  KTRSolverLSQ* solver = static_cast<KTRSolverLSQ*>(userParams);

  std::vector<double> xLoBndsVec(xLoBnds, xLoBnds + n);
  std::vector<double> xUpBndsVec(xUpBnds, xUpBnds + n);
  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n);

  KTRProblemLSQ* lsqPb = dynamic_cast<KTRProblemLSQ*>(solver->getProblem());
  if (lsqPb == NULL) {
    std::ostringstream message;
    message << "Invalid conversion from KTRIProblem* to KTRProblemLSQ*";
    throw KTRException(message.str(), __FILE__, __LINE__);
  } 

  int returnVal = lsqPb->msInitPtCallbackWrapper(nSolveNumber, xLoBndsVec, xUpBndsVec, xVec, lambdaVec, solver);

  for(size_t i(0); i<xVec.size();++i)
    x[i] = xVec[i];
  for(size_t i(0); i<lambdaVec.size();++i)
    lambda[i] = lambdaVec[i];
  return returnVal;
}

inline int KTRSolverLSQ::callbackMSProcess(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                        double const* const lambda, double* const obj, double* const c,
                                        double* const objGrad, double* const jac, double* const hessian,
                                        double* const hessVector, void* userParams) {
  KTRSolverLSQ* solver = static_cast<KTRSolverLSQ*>(userParams);

  std::vector<double> xVec(x, x + n);
  std::vector<double> lambdaVec(lambda, lambda + n);
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

  KTRProblemLSQ* lsqPb = dynamic_cast<KTRProblemLSQ*>(solver->getProblem());
  if (lsqPb == NULL) {
    std::ostringstream message;
    message << "Invalid conversion from KTRIProblem* to KTRProblemLSQ*";
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  return lsqPb->msProcessCallbackWrapper(evalRequestCode, xVec, lambdaVec, *obj, cVec, objGradVec,
                                         jacVec, hessianVec, hessVectorVec, solver);
}


inline void KTRSolverLSQ::initProblem() {
  int returnCode;

  KTRProblemLSQ* lsqProb = dynamic_cast<KTRProblemLSQ*>(_problem);
  if (lsqProb == NULL) {
    std::ostringstream message;
    message << "Invalid conversion from KTRIProblem* to KTRProblemLSQ*";
    throw KTRException(message.str(), __FILE__, __LINE__);
  }

  returnCode = KTR_lsq_init_problem(_kc, 
                                    lsqProb->getNumVars(), 
                                    lsqProb->getVarLoBnds().size() == 0 ? NULL : &lsqProb->getVarLoBnds()[0],
                                    lsqProb->getVarUpBnds().size() == 0 ? NULL : &lsqProb->getVarUpBnds()[0],
                                    lsqProb->getNumRes(),
                                    NULL,
                                    lsqProb->getNNZJ(),
                                    lsqProb->getNNZJ() == 0 ? NULL : &lsqProb->getJacIndexVars()[0],
                                    lsqProb->getNNZJ() == 0 ? NULL : &lsqProb->getJacIndexRes()[0],
                                    lsqProb->getXInitial().size() == 0 ? NULL : &lsqProb->getXInitial()[0],
									lsqProb->getLambdaInitial().size() == 0 ? NULL : &lsqProb->getLambdaInitial()[0]);

  if (returnCode != 0) {
    std::ostringstream message;
    message << "Error returned from KTR_init_problem";
    message << "; error code: " + returnCode;
    throw KTRException(message.str(), __FILE__, __LINE__);
  }
}

}


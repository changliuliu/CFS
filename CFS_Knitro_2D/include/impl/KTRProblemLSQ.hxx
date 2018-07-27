#pragma once

#include "knitro.h"
#include "KTRMsInitptCallback.h"
#include "KTRMSProcessCallback.h"
#include "KTRNewptCallback.h"
#include "KTRProblemLSQ.h"
#include "KTRSolverLSQ.h"

namespace knitro {
	

inline KTRProblemLSQ::KTRProblemLSQ(int n, int m) 
	: _lambdaInitial(0),
	  _jacIndexRes(n*m),
	  _jacIndexVars(n*m),
	  _xInitial(0),
	  _variables(n),
	  _residuals(m),
	  _objective(),
	  _putStringFunction(NULL),
	  _msInitPtCallback(NULL),
	  _msProcessCallback(NULL),
	  _newPointCallback(NULL) {
	setDenseJacobian();
}
	
inline KTRProblemLSQ::KTRProblemLSQ(int n, int m, int nnzJ) 
	: _lambdaInitial(0),
	  _jacIndexRes(nnzJ),
	  _jacIndexVars(nnzJ),
	  _xInitial(0),
	  _variables(n),
	  _residuals(m),
	  _objective(),
	  _putStringFunction(NULL),
	  _msInitPtCallback(NULL),
	  _msProcessCallback(NULL),
	  _newPointCallback(NULL) {

}

inline KTRProblemLSQ::~KTRProblemLSQ() {

}

inline int KTRProblemLSQ::getNumVars() const {
	return( static_cast<int>( _variables.size() ) );
}

inline int KTRProblemLSQ::getNumRes() const {
	return( static_cast<int>( _residuals.size() ) );
}

inline int KTRProblemLSQ::getNumCons() const {
	return( 0 );
}

inline int KTRProblemLSQ::getNNZJ() const {
	return( static_cast<int>( _jacIndexRes.size() ) );
}

inline int KTRProblemLSQ::getNNZH() const {
	return( static_cast<int>( _hessIndexCols.size() ) );
}

inline const std::vector<int>& KTRProblemLSQ::getVarTypes() const {
	return _variables.getTypes();
}

inline void KTRProblemLSQ::setDenseJacobian() {
  for (int i = 0; i < getNumVars(); i++) {
    for (int j = 0; j < getNumRes(); j++) {
#ifndef NDEBUG
      _jacIndexRes.at(j*getNumVars() + i) = j;
      _jacIndexVars.at(j*getNumVars() + i) = i;
#else
      _jacIndexRes[j * getNumVars() + i] = j;
      _jacIndexVars[j * getNumVars() + i] = i;
#endif
    }
  }
}

const std::vector<double>& KTRProblemLSQ::getXInitial() const {
	return( _xInitial );
}

const std::vector<double>& KTRProblemLSQ::getLambdaInitial() const {
	return( _lambdaInitial );
}

int KTRProblemLSQ::getObjGoal() const {
	return( _objective.getGoal() );
}

const std::vector<double>& KTRProblemLSQ::getVarLoBnds() const {
	return( _variables.getLoBnds() );
}

const std::vector<double>& KTRProblemLSQ::getVarUpBnds() const {
	return( _variables.getUpBnds() );
}

const std::vector<int>& KTRProblemLSQ::getJacIndexVars() const {
	return( _jacIndexVars );
}

const std::vector<int>& KTRProblemLSQ::getJacIndexRes() const {
	return( _jacIndexRes );
}

const std::vector<int>& KTRProblemLSQ::getHessIndexCols() const {
	return( _hessIndexCols );
}

const std::vector<int>& KTRProblemLSQ::getHessIndexRows() const {
	return( _hessIndexRows );
}

void KTRProblemLSQ::setObjType(int objType) {
	_objective.setType(objType);
}

void KTRProblemLSQ::setObjGoal(int objGoal) {
	_objective.setGoal(objGoal);
}

void KTRProblemLSQ::setVarLoBnds(int id, double val) {
	_variables.setLoBnds(id, val);
}

void KTRProblemLSQ::setVarUpBnds(int id, double val) {
	_variables.setUpBnds(id, val);
}

void KTRProblemLSQ::setXInitial(int id, double val) {
	if (_xInitial.empty()) {
    	_xInitial.resize(getNumVars());
  	}

#ifndef NDEBUG
  	_xInitial.at(id) = val;
#else
  	_xInitial[id] = val;
#endif
}

void KTRProblemLSQ::setPutStringFunction(KTRPutString *putStringFunction) {
	_putStringFunction = putStringFunction;
}

inline int KTRProblemLSQ::putStringFunctionWrapper(const std::string & str, KTRISolver* solver) {
  if (_putStringFunction == NULL) {
    return KTR_RC_CALLBACK_ERR;
  }

  return _putStringFunction->CallbackFunction(str.c_str(), (KTRSolverLSQ*)solver);
}

int KTRProblemLSQ::msInitPtCallbackWrapper(int nSolveNumber, const std::vector<double>& xLoBnds,
                                      const std::vector<double>& xUpBnds, std::vector<double>& x,
                                      std::vector<double>& lambda, KTRISolver* solver) {
  	if (_msInitPtCallback == NULL) {
    	return KTR_RC_CALLBACK_ERR;
  	}

  	return( _msInitPtCallback->CallbackFunction(nSolveNumber, 
                                             	xLoBnds, xUpBnds, x, lambda, 
                                            	(KTRSolverLSQ*)solver) );
}

int KTRProblemLSQ::msProcessCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda, double obj,
                               				std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac,
                               				std::vector<double>& hessian, std::vector<double>& hessVector, KTRISolver* solver) {
	if (_msProcessCallback == NULL) {
		return KTR_RC_CALLBACK_ERR;
	}

	return( _msProcessCallback->CallbackFunctionLSQ(x, obj, (KTRSolverLSQ*)solver) );
}

int KTRProblemLSQ::newPointCallbackWrapper(KTR_context_ptr kc, const std::vector<double>& x,
                                           const std::vector<double>& lambda, double obj, const std::vector<double>& c,
                                      		const std::vector<double>& objGrad, const std::vector<double>& jac,
                                      		KTRISolver* solver) {

	if (_newPointCallback == NULL) {
		return KTR_RC_CALLBACK_ERR;
	}

	return( _newPointCallback->CallbackFunctionLSQ(x, obj, objGrad, jac, (KTRSolverLSQ*)solver) );
}

int KTRProblemLSQ::evaluateResidual(const std::vector<double>& x, std::vector<double>& residual) {
	return KTR_RC_CALLBACK_ERR;
}

int KTRProblemLSQ::evaluateJacobian(const std::vector<double>& x, std::vector<double>& jacobian) {
	return KTR_RC_CALLBACK_ERR;
}

void KTRProblemLSQ::setJacIndexRes(int id, int val) {
#ifndef NDEBUG
  _jacIndexRes.at(id) = val;
#else
  _jacIndexRes[id] = val;
#endif
}

void KTRProblemLSQ::setJacIndexVars(int id, int val) {
#ifndef NDEBUG
  _jacIndexVars.at(id) = val;
#else
  _jacIndexVars[id] = val;
#endif
}

void KTRProblemLSQ::setJacIndexRes(const std::vector<int>& jacIndexRes) {
	_jacIndexRes = jacIndexRes;
}

void KTRProblemLSQ::setJacIndexVars(const std::vector<int>& jacIndexVars) {
	_jacIndexVars = jacIndexVars;
}

}


/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include "knitro.h"
#include "KTREnums.h"

namespace knitro {
    class KTRMipNodeCallback;
    class KTRMSInitptCallback;
    class KTRPutString;
    class KTRMSProcessCallback;
    class KTRNewptCallback;
    class KTRISolver;

/**
 * Interface for defining an optimization problem and passing it to a KTRSolver object.
 * Partially implemented in the KTRProblem class. A minimal problem definiton implements the objective function
 * and constraint evaluation function EvaluateFC along with the functions defined in KTRProblem.
 */
class KTRIProblem {
 public:

  /**
   * Virtual destructor required for virtual classes.
   */
  virtual ~KTRIProblem() {}

  /**
   *
   * @return True if the problem contains any integer or binary variables, defined by the variable types.
   */
  virtual bool isMipProblem() const = 0;
  
  /**
   *
   * @return True if the problem has a least-squares objective
   */
   virtual bool isLSQProblem() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The objective type. See KTREnums::ObjectiveType.
   */
  virtual int getObjType() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The objective function type. See KTREnums::FunctionType.
   */
  virtual int getObjFnType() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The objective goal. See KTREnums::ObjectiveGoal.
   */
  virtual int getObjGoal() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The variable types. See KTREnums::VariableType.
   */
  virtual const std::vector<int>& getVarTypes() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The variable lower bounds, in the range [-KTR_INFBOUND KTR_INFBOUND].
   * If the return value is an empty std::vector, all variables are unbounded below.
   */
  virtual const std::vector<double>& getVarLoBnds() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return The variable upper bounds, in the range [-KTR_INFBOUND KTR_INFBOUND].
   * If the return value is an empty std::vector, all variables are unbounded above.
   */
  virtual const std::vector<double>& getVarUpBnds() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Variable indices of non-zero elements of the Jacobian.
   */
  virtual const std::vector<int>& getJacIndexVars() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Column indices of non-zero elements of the Hessian.
   */
  virtual const std::vector<int>& getHessIndexCols() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Row indices of non-zero elements of the Hessian.
   */
  virtual const std::vector<int>& getHessIndexRows() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Number of variables in the problem.
   */
  virtual int getNumVars() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Number of constraints in the problem, not including variable upper and lower bounds or complementarity constraints.
   */
  virtual int getNumCons() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Number of complementarity constraints in the problem.
   */
  virtual int getNumCompCons() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Number of non-zero entries in the Jacobian.
   */
  virtual int getNNZJ() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Number of non-zero entries in the Hessian.
   */
  virtual int getNNZH() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Initial values of primal variables. If this empty, KNITRO determines initial primal variables automatically.
   */
  virtual const std::vector<double>& getXInitial() const = 0;

  /**
   * Called by KTRSolver when passing problem information to KNITRO.
   * @return Initial values of dual variables. If this empty, KNITRO determines initial dual variables automatically.
   */
  virtual const std::vector<double>& getLambdaInitial() const = 0;
  
  /**
   * Function to evaluate the constraint and the objective function,
   * and optionally, the objective function gradient and Jacobian values.
   * If the gradient and jacobian values are evaluated in this function,
   * the evaluateGA function should have an empty implementation that returns zero.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[out] c Values of each constraints to be evaluated at x by this function.
   * @param[out] objGrad Values of the objective function gradient to be evaluated at x by this function or by evaluateGA, but not both.
   * @param[out] jac Values of the Jacobian to be evaluated at x by this function or by evaluateGA, but not both.
   * @return Objective function value at x.
   */
  virtual double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                            std::vector<double>& jac) = 0;

  /**
   * Function to evaluate the constraint and the objective function,
   * and optionally, the objective function gradient and Jacobian values.
   * If the gradient and jacobian values are evaluated in this function,
   * the evaluateGA function should have an empty implementation that returns zero.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[out] obj Objective function value at x.
   * @param[out] c Values of each constraints to be evaluated at x by this function.
   * @param[out] objGrad Values of the objective function gradient to be evaluated at x by this function or by evaluateGA, but not both.
   * @param[out] jac Values of the Jacobian to be evaluated at x by this function or by evaluateGA, but not both.
   * @return A callback return status: 0 if no error is encountered, and KTR_RC_CALLBACK_ERR if an error is encountered.
   */
  inline virtual int evaluateFC(const std::vector<double>& x, double& obj,
      std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac) {
    obj = evaluateFC(x, c, objGrad, jac);
    return 0;
  }

  /**
   * Function to evaluate the objective function gradient and Jacobian values.
   * This function is optional, but may greatly increase performance. If this function is not implemented,
   * first derivatives can be approximated numerically. See the KNITRO manual information on KTR_PARAM_GRADOPT for informational on approximation.
   * If approximate first derivatives are to be used, the KTRSolver constructor should be passed a value of KTR_GRADOPT corresponding to the approxmation type.
   * If the gradient and jacobian values are evaluated in this function,
   * the evaluateFC function should not evaluate them.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[out] objGrad Values of the objective function gradient to be evaluated at x by this function or by evaluateFC, but not both.
   * @param[out] jac Values of non-zero elements of the Jacobian to be evaluated at x by this function or by evaluateFC, but not both.
   * @return 0 if no error is encountered, and KTR_RC_CALLBACK_ERR if an error is encountered. An error will stop the KNITRO solver.
   */
  virtual int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, 
                         std::vector<double>& jac) = 0;

  /**
   * Function to evaluate the Hessian of the Lagrangian.
   * This function is optional, but may greatly increase performance. If this function is not implemented,
   * the Hessian can be approximated numerically. See the KNITRO manual information on KTR_PARAM_HESSOPT for informational on approximation.
   * If an approximate Hesian is to be used, the KTRSolver constructor should be passed a value of KTR_HESSOPT corresponding to the approxmation type.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[in] objScaler Value to scale objective component of the Hessian.
   * @param[in] lambda The values of the dual variables to evaluate.
   * @param[out] hess Values of non-zero elements of the Hessian to be evaluated at x.
   * @return 0 if no error is encountered, and KTR_RC_CALLBACK_ERR if an error is encountered. An error will stop the KNITRO solver.
   */
  virtual int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                           std::vector<double>& hess) = 0;

  /**
   * Function to evaluate the Hessian-vector product.
   * This function is optional, but may increase performance over numerical approximations. If this function is not implemented,
   * the Hessian can be approximated numerically. See the KNITRO manual information on KTR_PARAM_HESSOPT for informational on approximation.
   * If this function is to be used, the KTRSolver constructor should be passed a KTR_HESSOPT_PRODUCT for the Hessian evaluation type parameter.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[in] objScaler Value to scale objective component of the Hessian.
   * @param[in] lambda The values of the dual variables to evaluate.
   * @param vector Values of the Hessian-vector product to be evaluated at x.
   * @return 0 if no error is encountered, and KTR_RC_CALLBACK_ERR if an error is encountered. An error will stop the KNITRO solver.
   */
  virtual int evaluateHessianVector(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                                    std::vector<double>& vector) = 0;

  /**
   * Used by KTRSolver to call the output redirection callback defined in an instance of KTRPutString.
   * An implementation of this function should pass its parameters on to a KTRPutString object, rather than
   * redirecting output itself. See KTRProblem for an implementation and the KTRPutString class for more information on output redirection.
   * @param str
   * @param solver
   * @return
   */
  virtual int putStringFunctionWrapper(const std::string & str, 
                                        KTRISolver* solver) = 0;

  /**
  * This function is used to register an output redirection callback with the problem.
  * @param putStringFunction The callback that will be called by the KNITRO solver.
  */
  virtual void setPutStringFunction(KTRPutString *putStringFunction) = 0;

  /**
   * Used by KTRSolver to call the MIP node callback defined in an instance of KTRMipNodeCallback.
   * An implementation of this function should pass its parameters on to a KTRMipNodeCallback object, rather than
   * implementing the MIP node callback itself. See KTRProblem for an implementation and the KTRMipNodeCallback class for more information on MIP node callbacks.
   * @param evalRequestCode
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param hessian
   * @param hessVector
   * @param solver
   * @return
   */
  virtual int mipNodeCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda,
                                     double obj, std::vector<double>& c, std::vector<double>& objGrad,
                                     std::vector<double>& jac, std::vector<double>& hessian,
                                     std::vector<double>& hessVector, KTRISolver* solver) = 0;

  /**
   * Used by KTRSolver to call the multistart process callback defined in an instance of KTRMSProcessCallback.
   * An implementation of this function should pass its parameters on to a KTRMSProcessCallback object, rather than
   * implementing the multistart process callback itself. See KTRProblem for an implementation and the KTRMSProcessCallback class for more information on multistart process callbacks.
   * @param evalRequestCode
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param hessian
   * @param hessVector
   * @param solver
   * @return
   */
  virtual int msProcessCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda,
                                       double obj, std::vector<double>& c, std::vector<double>& objGrad,
                                       std::vector<double>& jac, std::vector<double>& hessian,
                                       std::vector<double>& hessVector, KTRISolver* solver) = 0;

  /**
   * Used by KTRSolver to call the multistart initial point callback defined in an instance of KTRMSInitptCallback.
   * An implementation of this function should pass its parameters on to a KTRMSInitptCallback object, rather than
   * implementing the multistart initialization callback itself. See KTRProblem for an implementation and the KTRMSInitptCallback class for more information on multistart initialization callbacks.
   * @param nSolveNumber
   * @param xLoBnds
   * @param xUpBnds
   * @param x
   * @param lambda
   * @param solver
   * @return
   */
  virtual int msInitPtCallbackWrapper(int nSolveNumber, const std::vector<double>& xLoBnds,
                                      const std::vector<double>& xUpBnds, std::vector<double>& x,
                                      std::vector<double>& lambda, KTRISolver* solver) = 0;

  /**
   * Used by KTRSolver to call the new point callback defined in an instance of KTRNewptCallback.
   * An implementation of this function should pass its parameters on to a KTRNewptCallback object, rather than
   * implementing the new point callback itself. See KTRProblem for an implementation and the KTRNewptCallback class for more information on new point callbacks.
   * @param kc
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param solver
   * @return
   */
  virtual int newPointCallbackWrapper(KTR_context_ptr kc, const std::vector<double>& x,
                                      const std::vector<double>& lambda, double obj, const std::vector<double>& c,
                                      const std::vector<double>& objGrad, const std::vector<double>& jac,
                                      KTRISolver* solver) = 0;

};
}

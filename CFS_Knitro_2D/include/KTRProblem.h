/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "KTRIProblem.h"
#include "KTRVariables.h"
#include "KTRConstraints.h"
#include "KTRObjective.h"

namespace knitro {

class KTRSolver;
class KTRISolver;

/**
 * Partial implementation of the KTRIProblem interface. This class is extended in the included problem definition examples, which implement the function evaluations.
 * Holds problem information as private members and includes functions to set problem characteristics.
 */
class KTRProblem : public KTRIProblem {

 public:
  /**
   * Initialize a problem with n variables and m constraints (not including complementarity constraints or variable upper and lower bounds as
   * constraints). Initializes the problem without any Jacobian or Hessian
   * sparsity information. Sets the Jacobian sparsity pattern to a dense matrix.
   * Even if approximate first derivatives are used, Jacobian sparsity structure should be specified for increased performance, and this constructor should not be used.
   * @param n Number of variables in the problem.
   * @param m Number of constraints in the problem (not including variable bounds or complementarity constraints).
   */
  KTRProblem(int n, int m);

  /**
   * Initialize a problem with n variables, m constraints (not including complementarity constraints or variable upper and lower bounds as
   * constraints) and the number of non-zero elements of the Jacobian. This constructor should be used if Jacobian sparsity information is set in the problem,
   * but Hessian sparsity information is not.
   * @param n Number of variables in the problem.
   * @param m Number of constraints in the problem (not including variable bounds or complementarity constraints).
   * @param nnzJ Number of non-zero elements in the Jacobian.
   */
  KTRProblem(int n, int m, int nnzJ);

  /**
   * Initialize a problem with n variables, m constraints (not including complementarity constraints or variable upper and lower bounds as
   * constraints) and specify the number of non-zero Jacobian and Hessian elements.
   * @param n Number of variables in the problem.
   * @param m Number of constraints in the problem (not including variable bounds or complementarity constraints).
   * @param nnzJ Number of non-zero elements in the Jacobian.
   * @param nnzH Number of non-zero elements in the Hessian.
   */
  KTRProblem(int n, int m, int nnzJ, int nnzH);

  /**
   * Virtual destructor required for virtual classes.
   */
  virtual ~KTRProblem();

  /**
   * @return List of indices of "left hand side" of variables with complementarity constraints.
   * Variable with index _complementarityIndexList1[i] is complementary to variable with index _complementarityIndexList2[i].
   * Called by KTRSolver to pass complementarity constraints to the KNITRO solver.
   */
  const std::vector<int>& complementarityIndexList1() const;

  /**
   * @return List of indices of "left hand side" of variables with complementarity constraints.
   * Variable with index _complementarityIndexList1[i] is complementary to variable with index _complementarityIndexList2[i].
   * Called by KTRSolver to pass complementarity constraints to the KNITRO solver.
   */
  const std::vector<int>& complementarityIndexList2() const;

  /**
   * Sets the lists of variable indices which are complementary to each other.
   * The variable with index _complementarityIndexList1[i] is complementary to variable with index _complementarityIndexList2[i].
   @throws KTRException if indexList1 and indexList2 are not the same size.
   */
  void setComplementarity(const std::vector<int>& indexList1, const std::vector<int>& indexList2);

  /**
   *
   * @return Objective type value set by setObjType() function.
   */
  int getObjType() const;

  /**
   *
   * @return Objective function type value set by setObjFnType() function.
   */
  int getObjFnType() const;

  /**
   *
   * @return Objective goal value set by setObjGoal() function.
   */
  int getObjGoal() const;

  /**
   * Stores the objective type to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in any optimization problem.
   * @param objType The objective type value.
   */
  void setObjType(int objType);

  /**
   * Stores the objective function type to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param objFnType The objective function type value.
   */
  void setObjFnType(int objFnType);

  /**
   * Stores the objective goal to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in any optimization problem.
   * @param objGoal The objective goal value.
   0   */
  void setObjGoal(int objGoal);

  /**
   *
   * @return Variable types set by setVarTypes() functions.
   */
  const std::vector<int>& getVarTypes() const;

  /**
   * Stores the type of one variable to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param id Index of variable.
   * @param val Variable type.
   */
  void setVarTypes(int id, int val);

  /**
   * Sets all variable types to the same type.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param val Variable type for all variables.
   */
  void setVarTypes(int val);

  /**
   * Set all variable types to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param varTypes The types of all variables to set.
   */
  void setVarTypes(const std::vector<int>& varTypes);

  /**
   *
   * @return Variable lower bounds set by setVarLoBnds() functions.
   */
  const std::vector<double>& getVarLoBnds() const;

  /**
   * Stores the lower bound of one variable to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of variable.
   * @param val Lower bound value.
   */
  void setVarLoBnds(int id, double val);

  /**
   * Sets all variable lower bounds to the same value.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param val The lower bound of all variables to set.
   */
  void setVarLoBnds(double val);

  /**
   * Set all variable lower bounds to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param varLoBnds The lower bound values to set.
   */
  void setVarLoBnds(const std::vector<double>& varLoBnds);

  /**
   *
   * @return Variable upper bounds set by setVarUpBnds() functions.
   */
  const std::vector<double>& getVarUpBnds() const;

  /**
   * Stores the upper bound of one variable to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of variable.
   * @param val Upper bound value.
   */
  void setVarUpBnds(int id, double val);

  /**
   * Sets all variable upper bounds to the same value.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param val The upper bound of all variables to set.
   */
  void setVarUpBnds(double val);

  /**
   * Set all variable lower bounds to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param varUpBnds The upper bound values to set.
   */
  void setVarUpBnds(const std::vector<double>& varUpBnds);

  /**
   *
   * @return Constraint types set by setConTypes() functions.
   */
  const std::vector<int>& getConTypes() const;

  /**
   * Stores the type of one constraint to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in any optimization problem.
   * @param id Index of variable.
   * @param val The constraint type to set.
   */
  void setConTypes(int id, int val);

  /**
   * Sets all constraint types to the same type.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in any optimization problem.
   * @param val Constraint type for all constraints.
   */
  void setConTypes(int val);

  /**
   * Set all constraint types to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in any optimization problem.
   * @param conTypes The types of all constraints to set.
   */
  void setConTypes(const std::vector<int>& conTypes);

  /**
   *
   * @return Constraint function types set by setConFnTypes() functions.
   */
  const std::vector<int>& getConFnTypes() const;

  /**
   * Stores the function type of one constraint to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param id Index of variable.
   * @param val The constraint function type to set.
   */
  void setConFnTypes(int id, int val);

  /**
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param val Constraint function type for all constraints.
   */
  void setConFnTypes(int val);

  /**
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * This value is used in an optimization problem with integer and/or binary variables.
   * @param conFnTypes The function types of all constraints to set.
   */
  void setConFnTypes(const std::vector<int>& conFnTypes);

  /**
   *
   * @return Constraint lower bounds set by setConLoBnds() functions.
   */
  const std::vector<double>& getConLoBnds() const;

  /**
   * Stores the lower bound of one constraint to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of constraint.
   * @param val Lower bound value.
   */
  void setConLoBnds(int id, double val);

  /**
   * Sets all constraint lower bounds to the same value.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param val The lower bound of all constraints to set.
   */
  void setConLoBnds(double val);

  /**
   * Set all constraint lower bounds to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param conLoBnds The lower bound values to set.
   */
  void setConLoBnds(const std::vector<double>& conLoBnds);

  /**
   *
   * @return Constraint upper bounds set by setConUpBnds() functions.
   */
  const std::vector<double>& getConUpBnds() const;

  /**
   * Stores the upper bound of one constraint to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of constraint.
   * @param val Upper bound value.
   */
  void setConUpBnds(int id, double val);

  /**
   * Sets all constraint upper bounds to the same value.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param val The upper bound of all constraints to set.
   */
  void setConUpBnds(double val);

  /**
   * Set all constraint upper bounds to the values in the parameter.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param conUpBnds The upperlower bound values to set.
   */
  void setConUpBnds(const std::vector<double>& conUpBnds);

  /**
   *
   * @return Jacobian sparsity constraint index values set by setJacIndexCons() functions.
   */
  const std::vector<int>& getJacIndexCons() const;

  /**
   * Stores the constraint index of one non-zero element of the Jacobian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param i Indicates that val is the ith non-zero element of the Jacobian.
   * @param val Constraint index corresponding to ith nonzero Jacobian element.
   */
  void setJacIndexCons(int i, int val);

  /**
   * Stores the constraint indices of non-zero elements of the Jacobian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param jacIndexCons Constraint indices of nonzero Jacobian elements.
   */
  void setJacIndexCons(const std::vector<int>& jacIndexCons);

  /**
   *
   * @return Jacobian sparsity constraint index values set by setJacIndexVars() functions.
   */
  const std::vector<int>& getJacIndexVars() const;

  /**
   * Stores the variable indices of non-zero elements of the Jacobian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Indicates that val is the ith non-zero element of the Jacobian.
   * @param val Variable index corresponding to ith nonzero Jacobian element.
   */
  void setJacIndexVars(int id, int val);

  /**
   * Stores the variable indices of non-zero elements of the Jacobian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param jacIndexVars Variable indices of nonzero Jacobian elements.
   */
  void setJacIndexVars(const std::vector<int>& jacIndexVars);

  /**
   *
   * @return Hessian sparsity column index values set by setHessIndexCols() functions.
   */
  const std::vector<int>& getHessIndexCols() const;

  /**
   * Stores the column index of one non-zero element of the Hessian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Hessian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param i Indicates that val is the ith non-zero element of the Hessian.
   * @param val Column index corresponding to ith nonzero Hessian element.
   */
  void setHessIndexCols(int i, int val);

  /**
   * Stores the column indices of non-zero elements of the Hessian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Hessian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param hessIndexCols Column indices of nonzero Hessian elements.
   */
  void setHessIndexCols(const std::vector<int>& hessIndexCols);

  /**
   *
   * @return Hessian sparsity row index values set by setHessIndexRows() functions.
   */
  const std::vector<int>& getHessIndexRows() const;

  /**
   * Stores the row index of one non-zero element of the Hessian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Hessian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param i Indicates that val is the ith non-zero element of the Hessian.
   * @param val Row index corresponding to ith nonzero Hessian element.
   */
  void setHessIndexRows(int i, int val);

  /**
   * Stores the row indices of non-zero elements of the Hessian.
   * See the Derivatives section of the KNITRO Documentation for details on how to specify Hessian sparsity.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param hessIndexRows Column indices of nonzero Hessian elements.
   */
  void setHessIndexRows(const std::vector<int>& hessIndexRows);

  /**
   *
   * @return The number of variables in the problem set by the KTRProblem constructor.
   */
  int getNumVars() const;

  /**
   *
   * @return The number of variables in the problem set by the KTRProblem constructor, not counting variable
   * upper and lower bounds or complementarity constraints.
   */
  int getNumCons() const;

  /**
   *
   * @return The number of complementarity constraints in the problem set in the SetComplementarity function.
   */
  int getNumCompCons() const;

  /**
   *
   * @return The number of non-zero entries of the Jacobian set by the constructor of the problem.
   * If NNZJ is not specified in the constructor, returns n * m (implying a full sparsity pattern).
   */
  int getNNZJ() const;

  /**
   *
   * @return The number of non-zero entries of the Hessian. If NNZH is not specified in the constructor, returns 0.
   */
  int getNNZH() const;

  /**
   *
   * @return True if the problem has any integer or binary variables and false otherwise, based on
   * the variable types set by the setVarTypes functions.
   */
  virtual bool isMipProblem() const;
  
  /**
   *
   * @return True if the problem has a least-squares objective.
   */
  virtual bool isLSQProblem() const { return false; }

  /**
   *
   * @return Initial primal variable values set by setXInitial() functions.
   */
  const std::vector<double>& getXInitial() const;

  /**
   * Stores the initial of one primal variable to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of the primal variable.
   * @param val Initial value of the primal variable.
   */
  void setXInitial(int id, double val);

  /**
   * Stores the initial values of primal variables to be passed to KTRSolver when registering the problem with KNITRO.
   * @param xInitial Initial values of primal variables.
   */
  void setXInitial(const std::vector<double>& xInitial);

  /**
   *
   * @return Initial dual variable values set by setXInitial() functions.
   */
  const std::vector<double>& getLambdaInitial() const;

  /**
   * Stores the initial of one dual variable to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param id Index of the dual variable.
   * @param val Initial value of the dual variable.
   */
  void setLambdaInitial(int id, double val);

  /**
   * Stores the initial values of dual variables to be passed to KTRSolver when registering the problem with KNITRO.
   * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   * @param lambdaInitial
   */
  void setLambdaInitial(const std::vector<double>& lambdaInitial);

   /**
   * Empty implementation of evaluateFC.
   * To implement objective and constraints in an inheriting class of KTRProblem, this function needs to be overriden.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[in] c The values of the constraints at x.
   * @param[out] objGrad Gradient values to evaluate at x.
   * @param [out] jac Values of non-zero elements of the Jacobian to be evaluated at x.
   * @return KTR_RC_CALLBACK_ERR, a KNITRO error code indicating an error in evaluating the first derivatives.
   */
   virtual double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac);
  
  /**
   * Empty implementation of evaluateGA.
   * To implement exact first derivatives in an inheriting class of KTRProblem, this function needs to be overriden.
   * If this function is not overriden and KTR_GRADOPT is not specified in KTRSolver as one of the approximate gradient evaluation types,
   * this function returns an error and the KNITRO solver will halt.
   * If gradient and jacobian are returned in evaluateFC, this function should do nothing but returning KTR_RC_EVALFCGA.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[out] objGrad Gradient values to evaluate at x.
   * @param [out] jac Values of non-zero elements of the Jacobian to be evaluated at x.
   * @return KTR_RC_CALLBACK_ERR, a KNITRO error code indicating an error in evaluating the first derivatives or
   *         KTR_RC_EVALFCGA if gradient and jacobian were returned by evaluateFC.
   */
  virtual int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac);

  /**
   * Empty implementation of evaluateHess.
   * To implement exact second derivatives in an inheriting class of KTRProblem, this function needs to be overriden.
   * If this function is not overriden and KTR_HESSOPT is not specified in KTRSolver as one of the approximate Hessian evaluation types,
   * this function returns an error and the KNITRO solver will halt.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[in] objScaler Value to scale objective component of the Hessian.
   * @param[in] lambda The values of the dual variables to evaluate.
   * @param[out] hess Values of non-zero elements of the Hessian to be evaluated at x.
   * @return KTR_RC_CALLBACK_ERR, a KNITRO error code indicating an error in evaluating the second derivatives.
   */
  virtual int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda, std::vector<double>& hess);

  /**
   * Empty implementation of evaluateHessianVector.
   * To implement Hessian-vector product evaluation in an inheriting class of KTRProblem, this function needs to be overriden.
   * If this function is not overriden and KTR_HESSOPT is specified in KTRSolver as KTR_HESSOPT_PRODUCT,
   * this function returns an error and the KNITRO solver will halt.
   * @param[in] x The values of the primal variables to evaluate.
   * @param[in] objScaler Value to scale objective component of the Hessian.
   * @param[in] lambda The values of the dual variables to evaluate.
   * @param vector Values of the Hessian-vector product to be evaluated at x.
   * @return KTR_RC_CALLBACK_ERR, a KNITRO error code indicating an error in evaluating the second derivatives.
   */
  virtual int evaluateHessianVector(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                                     std::vector<double>& vector);

  /**
   * Used by KTRSolver to call the output redirection callback defined in an instance of KTRPutString.
   * This function should passes its input parameters to a KTRPutString object, rather than
   * redirecting output itself. See the KTRPutString class for more information on output redirection.
   * @param str
   * @param solver
   * @return
   */
  virtual int putStringFunctionWrapper(const std::string & str, KTRISolver* solver);

  /**
   * Used by KTRSolver to call the MIP node callback defined in an instance of KTRMipNodeCallback.
   * This function passes its parameters on to a KTRMipNodeCallback object, rather than
   * implementing the MIP node callback itself. See the KTRMipNodeCallback
   * class for more information on MIP node callbacks.
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
  virtual int mipNodeCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda, double obj,
                             std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac,
                             std::vector<double>& hessian, std::vector<double>& hessVector, KTRISolver* solver);

  /**
   * Used by KTRSolver to call the multistart process callback defined in an instance of KTRMSProcessCallback.
   * This function passes its parameters on to a KTRMSProcessCallback object, rather than
   * implementing the multistart process callback itself. See the KTRMSProcessCallback class for more information on multistart process callbacks.
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
  int msProcessCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda, double obj,
                               std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac,
                               std::vector<double>& hessian, std::vector<double>& hessVector, KTRISolver* solver);

  /**
   * Used by KTRSolver to call the multistart initial point callback defined in an instance of KTRMSInitptCallback.
   * This function passes its parameters on to a KTRMSInitptCallback object, rather than
   * implementing the multistart initialization callback itself. See the KTRMSInitptCallback class for more information on multistart initialization callbacks.
   * @param nSolveNumber
   * @param xLoBnds
   * @param xUpBnds
   * @param x
   * @param lambda
   * @param solver
   * @return
   */
  int msInitPtCallbackWrapper(int nSolveNumber, const std::vector<double>& xLoBnds, const std::vector<double>& xUpBnds,
                              std::vector<double>& x, std::vector<double>& lambda, KTRISolver* solver);

  /**
   * Used by KTRSolver to call the new point callback defined in an instance of KTRNewptCallback.
   * This function should pass its parameters on to a KTRNewptCallback object, rather than
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
  int newPointCallbackWrapper(KTR_context_ptr kc, const std::vector<double>& x, const std::vector<double>& lambda,
                              double obj, const std::vector<double>& c, const std::vector<double>& objGrad,
                              const std::vector<double>& jac, KTRISolver* solver);

  /**
   * This function is used to register a new point callback with the problem.
   * @param newPointCallback The callback that will be called by the KNITRO solver.
   */
  void setNewPointCallback(KTRNewptCallback* newPointCallback);

  /**
   * This function is used to register a multistart process callback with the problem.
   * @param MSProcessCallback The callback that will be called by the KNITRO solver.
   */
  void setMSProcessCallback(KTRMSProcessCallback* MSProcessCallback);

  /**
   * This function is used to register an output redirection callback with the problem.
   * @param putStringFunction The callback that will be called by the KNITRO solver.
   */
  void setPutStringFunction(KTRPutString* putStringFunction);

  /**
   * This function is used to register a multistart initial point callback with the problem.
   * @param MSInitPtCallback The callback that will be called by the KNITRO solver.
   */
  void setMSInitPtCallback(KTRMSInitptCallback* MSInitPtCallback);

  /**
   * This function is used to register a MIP node callback with the problem.
   * @param mipNodeCallback The callback that will be called by the KNITRO solver.
   */
  void setMipNodeCallback(KTRMipNodeCallback* mipNodeCallback);

  
 protected:
  /**
   * Sets the Jacobian sparsity to a dense matrix in column-major order.
   * This function is called if the user does not pass the constructor a value
   * for the number of non-zero elements in the Jacobian.
   * Not specifying Jacobian information is not recommended.
   * Even if the gradient evaluation is approximate, a sparsity pattern will speed up derivative evaluation.
   */
  void setDenseJacobian();

 private:
  /**
   * Initial dual variable values. If this is empty, KNITRO chooses initial dual variable values automatically.
   */
  std::vector<double> _lambdaInitial;

  /**
   * Column indices of non-zero elements of the Hessian.
   */
  std::vector<int> _hessIndexCols;

  /**
   * Row indices of non-zero elements of the Hessian.
   */
  std::vector<int> _hessIndexRows;

  /**
   * Constraint indices of non-zero elements of the Jacobian.
   */
  std::vector<int> _jacIndexCons;

  /**
   * Variable indices of non-zero elements of the Jacobian.
   */
  std::vector<int> _jacIndexVars;

  /**
   * Initial values of primal variables.
   */
  std::vector<double> _xInitial;

  /**
   * List of indices of "left hand side" of variables with complementarity constraints.
   * Variable with index _complementarityIndexList1[i] is complementary to variable with index _complementarityIndexList2[i].
   */
  std::vector<int> _complementarityIndexList1;

  /**
   * List of indices of "right hand side" of variables with complementarity constraints.
   * Variable with index _complementarityIndexList2[i] is complementary to variable with index _complementarityIndexList1[i].
   */
  std::vector<int> _complementarityIndexList2;

  /**
   * Used when the KNITRO solver calls a multistart process callback.
   */
  KTRMSProcessCallback* _msProcessCallback;

  /**
   * Used when the KNITRO solver calls a MIP node callback.
   */
  KTRMipNodeCallback* _mipNodeCallback;

  /**
   * Used when the KNITRO solver calls a MIP node callback.
   */
  KTRNewptCallback* _newPointCallback;

  /**
   * Used when the KNITRO solver calls a multistart initialization callback.
   */
  KTRMSInitptCallback* _msInitPtCallback;

  /**
   * Used when the KNITRO solver calls an output redirection callback.
   */
  KTRPutString* _putStringFunction;

  /**
   * Holds problem objective information.
   */
  KTRObjective _objective;

  /**
   * Holds problem variable information.
   */
  KTRVariables _variables;

  /**
   * Holds problem constraint information.
   */
  KTRConstraints _constraints;
};
}

#include "impl/KTRProblem.hxx"



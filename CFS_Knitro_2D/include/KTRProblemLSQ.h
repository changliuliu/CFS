#pragma once

#include "KTRVariables.h"
#include "KTRResiduals.h"
#include "KTRObjective.h"
#include "KTRIProblem.h"
#include "KTRPutString.h"

namespace knitro {

class KTRISolver;
	
class KTRProblemLSQ : public KTRIProblem {
	
public: 
	/**
	* Initialize a nonlinear least-squares problem with n variables and m residuals. Initializes the problem without any Jacobian or Hessian
	* sparsity information. Sets the Jacobian sparsity pattern to a dense matrix.
	* Even if approximate first derivatives are used, Jacobian sparsity structure should be specified for increased performance, and this constructor should not be used.
	* @param n Number of variables in the problem.
	* @param m Number of residuals in the problem.
	*/
	explicit KTRProblemLSQ(int n, int m); 

	/**
	* Initialize a nonlinear least-squares problem with n variables, m residuals and the number of non-zero elements of the residuals Jacobian. 
	* This constructor should be used if Jacobian sparsity information is set in the problem,
	* but Hessian sparsity information is not.
	* @param n Number of variables in the problem.
	* @param m Number of residuals in the problem.
	* @param nnzJ Number of non-zero elements in the residuals Jacobian.
	*/
	explicit KTRProblemLSQ(int n, int m, int nnzJ);

	/**
	* Virtual destructor required for virtual classes.
	*/
	virtual ~KTRProblemLSQ();

	/**
   	*
   	* @return Objective goal value set by setObjGoal() function.
   	*/
  	int getObjGoal() const;

	/**
	*
	* @return Indicator whether the problem is a MIP. False by default for nonlinear least-squares.
	*/
    virtual bool isMipProblem() const { return false; }
	
	/**
	*
	* @return Indicator whether the problem is a least-squares problem.
	*/
	virtual bool isLSQProblem() const { return true; }

	/**
	*
	* @return Objective type
	*/
    virtual int getObjType() const { return KTR_OBJTYPE_GENERAL; }

	/**
	*
	* @return Objective function type.
	*/
    virtual int getObjFnType() const { return KTR_FNTYPE_NONCONVEX; }
	
	/**
	* 
	* @return Vector containing variables types
	*/
    virtual const std::vector<int>& getVarTypes() const;
    
	/**
	*
	* @return Number of constraints. By default, 0 for nonlinear least-squares problems. 
	*/
  virtual int getNumCons() const;

	/**
	*
	* @return Number of complementarity constraints. By default, 0 for nonlinear least-squares.
	*/
  virtual int getNumCompCons() const { return 0; }

  	/**
    * Stores the objective goal to be passed to KTRSolver when registering the problem with KNITRO.
    * The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
    * This value is used in any optimization problem.
    * @param objGoal The objective goal value.
    */
  	void setObjGoal(int objGoal);

	/**
	* Stores the objective type to be passed to KTRSolver when registering the problem with KNITRO.
	* The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
    * This value is used in any optimization problem.
    * @param objType The objective goal value.
    */
    void setObjType(int objType);

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
   	* @return Jacobian sparsity residual index values set by setJacIndexCons() functions.
   	*/
  	const std::vector<int>& getJacIndexRes() const;

  	/**
   	* Stores the residual index of one non-zero element of the Jacobian.
   	* See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   	* The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   	* @param i Indicates that val is the ith non-zero element of the Jacobian.
   	* @param val Residual index corresponding to ith nonzero Jacobian element.
   	*/
 	  void setJacIndexRes(int i, int val);

    /**
   	* Stores the residual indices of non-zero elements of the Jacobian.
   	* See the Derivatives section of the KNITRO Documentation for details on how to specify Jacobian sparsity.
   	* The value should be set before the KTRProblem object is passed to a KTRSolver constructor.
   	* @param jacIndexCons Residual indices of nonzero Jacobian elements.
   	*/
  	void setJacIndexRes(const std::vector<int>& jacIndexRes);

  	/**
   	*
   	* @return Jacobian sparsity residual index values set by setJacIndexVars() functions.
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
     *
     * @return Hessian sparsity row index values set by setHessIndexRows() functions.
     */
    const std::vector<int>& getHessIndexRows() const;

  	/**
   	*
   	* @return The number of variables in the problem set by the KTRProblemLSQ constructor.
   	*/
  	int getNumVars() const;

  	/**
  	*
  	* @return The number of residuals in the problem set by the KTRProblemLSQ constructor. 
	  */
  	int getNumRes() const;

  	/**
    *
    * @return The number of non-zero entries of the Jacobian set by the constructor of the problem.
    * If NNZJ is not specified in the constructor, returns n * m (implying a full sparsity pattern).
    */
  	int getNNZJ() const;

    /**
    *
    * @return The number of non-zero entries of the Jacobian set by the constructor of the problem.
    * If NNZJ is not specified in the constructor, returns n * m (implying a full sparsity pattern).
    */
    int getNNZH() const;

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


  	virtual int evaluateResidual(const std::vector<double>& x, std::vector<double>& residual);


  	virtual int evaluateJacobian(const std::vector<double>& x, std::vector<double>& jacobian);


  	void setDenseJacobian();


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
                              std::vector<double>& jac) { return KTR_RC_CALLBACK_ERR; }

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
                           std::vector<double>& jac) { return KTR_RC_CALLBACK_ERR; }

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
                             std::vector<double>& hess) { return KTR_RC_CALLBACK_ERR; }

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
                                    std::vector<double>& vector) { return KTR_RC_CALLBACK_ERR; }

    /**
    * This function is used to register an output redirection callback with the problem.
    * @param putStringFunction The callback that will be called by the KNITRO solver.
    */
    void setPutStringFunction(KTRPutString *putStringFunction);

    /**
    * Used by KTRSolver to call the output redirection callback defined in an instance of KTRPutString.
    * An implementation of this function should pass its parameters on to a KTRPutString object, rather than
    * redirecting output itself. See KTRProblem for an implementation and the KTRPutString class for more information on output redirection.
    * @param str
    * @param solver
    * @return
    */
    virtual int putStringFunctionWrapper(const std::string & str, KTRISolver* solver);

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
                                     std::vector<double>& hessVector, KTRISolver* solver) { return KTR_RC_CALLBACK_ERR; }

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
    virtual int msProcessCallbackWrapper(int evalRequestCode, std::vector<double>& x, std::vector<double>& lambda, double obj,
                               std::vector<double>& c, std::vector<double>& objGrad, std::vector<double>& jac,
                               std::vector<double>& hessian, std::vector<double>& hessVector, KTRISolver* solver);

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
                                      std::vector<double>& lambda, KTRISolver* solver);

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
                                      KTRISolver* solver);

private:
	/**
   	* Initial dual variable values. If this is empty, KNITRO chooses initial dual variable values automatically.
   	*/
  	std::vector<double> _lambdaInitial;

  	/**
   	* Constraint indices of non-zero elements of the Residuals Jacobian.
   	*/
  	std::vector<int> _jacIndexRes;

  	/**
   	* Variable indices of non-zero elements of the Residuals Jacobian.
   	*/
  	std::vector<int> _jacIndexVars;

    /**
    * Column indices of non-zero elements of the Hessian.
    */
    std::vector<int> _hessIndexCols;

    /**
    * Row indices of non-zero elements of the Hessian.
    */
    std::vector<int> _hessIndexRows;

  	/**
   	* Initial values of primal variables.
   	*/
  	std::vector<double> _xInitial;

	/**
	* LSQ problem variables 
	*/
	KTRVariables _variables;

	/** 
	* LSQ problem residuals 
	*/
	KTRResiduals _residuals;	

	/** 
	* Objective information 
	*/
	KTRObjective _objective;
    
    /**
    * Used when the KNITRO solver calls an output redirection callback.
    */
    KTRPutString* _putStringFunction;

    /**
    * Used when the KNITRO solver calls a multistart initialization callback.
    */
    KTRMSInitptCallback* _msInitPtCallback;

    /**
    * Used when the KNITRO solver calls a multistart process callback.
    */
    KTRMSProcessCallback* _msProcessCallback;

    /**
    * Used when the KNITRO solver calls a MIP node callback.
    */
    KTRNewptCallback* _newPointCallback;

};	

}

#include "impl/KTRProblemLSQ.hxx"


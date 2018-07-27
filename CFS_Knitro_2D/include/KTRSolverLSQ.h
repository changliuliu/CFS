#pragma once

#include <vector>
#include <string>

#include "knitro.h"
#include "zlm.h"
#include "KTRISolver.h"

namespace knitro {

class KTRProblemLSQ;

class KTRSolverLSQ : public KTRISolver {

public: 
  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  explicit KTRSolverLSQ(KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  explicit KTRSolverLSQ(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  explicit KTRSolverLSQ(KTRIProblem * problem);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  explicit KTRSolverLSQ(const ZLM * zlm, KTRIProblem * problem);

  /**
   * Destructor for solver. Clears the KNITRO problem information stored by the KNITRO solver. This function shoulld not be called manually.
   */
  virtual ~KTRSolverLSQ();

  /**
   * Set names for problem components. KNITRO will use these names when printing problem and solution information.
   * @param objName Objective name.
   * @param varNames Variable names.
   * @param conNames Constraint names.
   * @throws KTRException if any of the vectors are the wrong length. One or both vectors may be empty if default names for a component are to be used, but should otherwise have size equal to number of variables and number of constraints, respectively.
   */
  void setNames(const std::string& objName, const std::vector<std::string>& varNames);

  /**
   * This function should be called between successive calls to KTRSolver::solve(). See KTR_restart in the KNITRO user manual for information on what problem changes can be made between calling solve.
   * @param xInitial New primal variable initial values. If this vector is empty, KNITRO will choose initial primal variables automatically.
   * @param lambdaInitial New dual variable initial values. If this vector is empty, KNITRO will choose initial dual variables automatically.
   * @throws KTRException if the parameters are the wrong length. One or both arguments may be empty if initial primal and/or dual values are to be set automatically by KNITRO, but should otherwise specify initial values for all variables.
   */
  virtual void restart(const std::vector<double>& xInitial, const std::vector<double>& lambdaInitial);

  /**
  * This function returns the values of the nonzero elements in the residuals jacobian.
  */
  virtual std::vector<double> getJacobianValues() const;
	
  /*
  * This function returns the values of the nonzero elements in the problem hessian.
  */
  virtual std::vector<double> getHessianValues() const;

   /**
   * Registers a KTRMSInitptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setMSInitptCallback();

  /**
   * Registers a KTRMSProcessCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setMSProcessCallback();

  /**
   * Registers a KTRNewptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setNewptCallback();

private:
  /**
   * Called internally when solve() is called. This function should not be called manually.
   * @param resetVariableBounds If this parameter is true, variable bounds are updated to the current bounds in the KTRIProblem pointed to by this solver object. If this parameter is false, the bounds originally set in the KTRSolver constructor are used.
   * @return KNITRO solver return code.
   */
  virtual int resolve(bool resetVariableBounds);

  /**
   * Registers callbacks with KNITRO solver. This function is called automatically when solve() is called and does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual void initCallbacks();

  /**
   *
   * @return True if the problem passed to this object's constructor has any integer or binary variables, and integrality is not relaxed.
   */
  virtual bool isMipProblem() const;

	/**
	* This function is used to call the user-defined residual function and pass the results back to the KNITRO solver.
	* @param evalRequestCode
    * @param n
    * @param m
    * @param nnzJ
    * @param x
    * @param lambda
    * @param res
    * @param jac
    * @param userParams
    * @return
	*/
	static int callbackEvalRes(int n,
                              int m,
                              int nnzJ,
                              const double * const x,
                              double * const res,
                              double * const jac,
                              void * userParams);

  /**
	* This function is used to call the user-defined residual function and pass the results back to the KNITRO solver.
	* @param evalRequestCode
    * @param n
    * @param m
    * @param nnzJ
    * @param x
    * @param lambda
    * @param res
    * @param jac
    * @param userParams
    * @return
	*/
	static int callbackEvalJac(int n,
							  int m,
							  int nnzJ,
							  const double * const x,
							  double * const res,
							  double * const jac,
							  void * userParams);

  /**
   * This function is used to call the user-defined multistart initial point callback, and pass the results back to the KNITRO solver.
   This function does not need to be called manually.
   * @param nSolveNumber
   * @param n
   * @param m
   * @param xLoBnds
   * @param xUpBnds
   * @param x
   * @param lambda
   * @param userParams
   * @return
   */
  static int callbackMSInitpt(int nSolveNumber, int n, int m, double const* const xLoBnds, double const* const xUpBnds,
                              double* const x, double* const lambda, void* const userParams);

  /**
   * This function is used to call the user-defined multistart process  callback, and pass the results back to the KNITRO solver.
   This function does not need to be called manually.
   * @param evalRequestCode
   * @param n
   * @param m
   * @param nnzJ
   * @param nnzH
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param hessian
   * @param hessVector
   * @param userParams
   * @return
   */						  
  static int callbackMSProcess(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                                double const* const lambda, double* const obj, double* const c,
                                double* const objGrad, double* const jac, double* const hessian,
                                double* const hessVector, void* userParams);
    /**
   * This function is used to call the user-defined new point callback, and pass the results back to the KNITRO solver. This function does not need to be called manually.
   * @param kc
   * @param n
   * @param m
   * @param nnzJ
   * @param x
   * @param lambda
   * @param obj
   * @param c
   * @param objGrad
   * @param jac
   * @param userParams
   * @return
   */
  static int callbackNewPt(KTR_context_ptr kc, int n, int m, int nnzJ, double const* const x,
                           double const* const lambda, double obj, double const* const c, double const* const objGrad,
                           double const* const jac, void* userParams);


  /**
   * The KTRSolver copy constructor is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   */
  KTRSolverLSQ(const KTRSolverLSQ&);

  /**
   * The KTRSolver copy operator is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   * @return
   */
  KTRSolverLSQ& operator=(const KTRSolverLSQ&);

  /**
   * Called by the KTRSolver constructor to register the problem properties with the KNITRO solver. Problem properties are defined by the KTRIProblem object passed to the constructor of this object.
   * @throws KTRException if KNITRO encounters a problem. This usually indicates some invalid problem properties.
   */
  virtual void initProblem();
};

}

#include "impl/KTRSolverLSQ.hxx"







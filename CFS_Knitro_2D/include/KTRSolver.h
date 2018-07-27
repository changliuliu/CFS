/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include <string>

#include "knitro.h"
#include "zlm.h"
#include "KTRISolver.h"
#include "KTRConstants.h"

namespace knitro {

class KTRIProblem;

class KTRSolver : public KTRISolver {
  
 public:
  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  explicit KTRSolver(KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  explicit KTRSolver(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  explicit KTRSolver(KTRIProblem * problem);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  explicit KTRSolver(const ZLM * zlm, KTRIProblem * problem);

  /**
   * Destructor for solver. Clears the KNITRO problem information stored by the KNITRO solver. This function shoulld not be called manually.
   */
  virtual ~KTRSolver();

  /**
   * This function should be called before calling solve() if a MIP node callback is to be called by the KNITRO solver. This requires a KTRMIPNodeCallback object to be passed to the KTRIProblem object that was passed to this object's constructor.
   */
  void useMipNodeCallback();

  /**
   * Set scaling parameters (constraints and complementarity constraints)
   * @param cScaleFactors Vector of scaling factors on constraints
   * @param ccScaleFactors Vector of scaling factors on complementariy constraints 
   */
  void setConScaling(const std::vector<double>& cScaleFactors, const std::vector<double>& ccScaleFactors);

  /**
   * Sets absolute feasibility tolerances to use for solver termination tests. See KNITRO callable library option KTR_set_feastols for more information.
   * @param cFeasTols Absolute feasbility tolerances for constraints.
   * @param xFeasTols Absolute feasbility tolerances for variables.
   * @param ccFeasTols Absolute feasbility tolerances for complementarity constraints.
   * @throws KTRException if any of the vectors are the wrong length. A vector may be empty if default feasbility tolerances for a component are to be used, but should otherwise have size equal to number of constraints, number of variables, and number of complementarity constraints, respectively.
   */
  void setFeasTols(const std::vector<double>& cFeasTols, const std::vector<double>& xFeasTols,
                   const std::vector<double>& ccFeasTols);

  /**
   * Set names for problem components. KNITRO will use these names when printing problem and solution information.
   * @param objName Objective name.
   * @param varNames Variable names.
   * @param conNames Constraint names.
   * @throws KTRException if any of the vectors are the wrong length. One or both vectors may be empty if default names for a component are to be used, but should otherwise have size equal to number of variables and number of constraints, respectively.
   */
  void setNames(const std::string& objName, const std::vector<std::string>& varNames,
                const std::vector<std::string>& conNames);

  /**
   * This function should be called between successive calls to KTRSolver::solve(). See KTR_restart in the KNITRO user manual for information on what problem changes can be made between calling solve.
   * @param xInitial New primal variable initial values. If this vector is empty, KNITRO will choose initial primal variables automatically.
   * @param lambdaInitial New dual variable initial values. If this vector is empty, KNITRO will choose initial dual variables automatically.
   * @throws KTRException if the parameters are the wrong length. One or both arguments may be empty if initial primal and/or dual values are to be set automatically by KNITRO, but should otherwise specify initial values for all variables.
   */
  virtual void restart(const std::vector<double>& xInitial, const std::vector<double>& lambdaInitial);

  /**
   *
   * @return The number of function evaluations from solve().
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberFCEvals() const;

  /**
   *
   * @return The number of gradient evaluations from solve().
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberGAEvals() const;

  /**
   *
   * @return The number of Hessian evaluations from solve().
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberHEvals() const;

  /**
   *
   * @return The number of Hessian-vector product evaluations from solve().
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberHVEvals() const;

  /**
   *
   * @return The absolute feasibility error in the solution.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual double getAbsFeasError() const;

  /**
   *
   * @return The relative feasibility error in the solution.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getRelFeasError() const;

  /**
   *
   * @return The values of constraints found from solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  std::vector<double> getConstraintValues() const;


  std::vector<double> getJacobianValues() const;
  /**
   *
   * @return The value of the Hessian found from solve for each non-zero Hessian entry defined in the KTRIProblem object.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  std::vector<double> getHessianValues() const;

  /**
   * Sets branching priorities for integer variables. Values for continuous variables are ignored.
   * @param xPriorities Vector of branching priorities for variables. Has one entry for each of primal variables, although values for continuous variables are ignored.
   * @throws KTRException if a valid branching priorities are not set for all variables.
   */
  void mipSetBranchingPriorities(const std::vector<int>& xPriorities);

  /**
   *
   * @return The number of MIP nodes processed in a MIP solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getMipNumNodes() const;

  /**
   *
   * @return The number of continuous subproblem solves in a MIP solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getMipNumSolves() const;

  /**
   *
   * @return The absolute MIP integrality gap from the MIP solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getMipAbsGap() const;

  /**
   *
   * @return The relative MIP integrality gap from the MIP solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getMipRelGap() const;

  /**
   *
   * @return The objective function value of the MIP incumbent. Returns KTR_INFBOUND (infinity) if no incumbent has been found.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getMipIncumbentObj() const;

  /**
   *
   * @return The incumbent value of the current MIP relaxation bound.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getMipRelaxationBnd() const;

  /**
   *
   * @return The objective function value of the most recently solved MIP node subproblem.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getMipLastNodeObj() const;

  /**
   *
   * @return Primal variable values for the most recently solved MIP node subproblem.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  std::vector<double> getMipIncumbentX() const;

  /**
   *
   * @param name String name of a KNITRO parameter.
   * @return Integer identifier of a KNITRO parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  int getParamID(const std::string& name) const;

  /**
   * Sets integrality of a MIP problem to relaxed or not.
   * @param integralityRelaxed True if integrality of the MIP problem is relaxed (all variable are treated as continuous).
   */
  void setIntegralityRelaxed(bool integralityRelaxed);

  /**
  * Set strategies for dealing with individual integer variables. Possible
  *  strategy values include:
  *    KTR_MIP_INTVAR_STRATEGY_NONE    0 (default)
  *    KTR_MIP_INTVAR_STRATEGY_RELAX   1
  *    KTR_MIP_INTVAR_STRATEGY_MPEC    2 (binary variables only)    
  * @param xIndex index of integer variable, xStrategy strategy 
  */
  void setIntVarStrategy(const int xIndex, const int xStrategy);

 private:
  /**
   * Registers a KTRMIPNodeCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  void setMipNodeCallback();

  /**
   * Registers a KTRMSInitptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  void setMSInitptCallback();

  /**
   * Called internally when solve() is called. This function should not be called manually.
   * @param resetVariableBounds If this parameter is true, variable bounds are updated to the current bounds in the KTRIProblem pointed to by this solver object. If this parameter is false, the bounds originally set in the KTRSolver constructor are used.
   * @return KNITRO solver return code.
   */
  virtual int resolve(bool resetVariableBounds);

  /**
   * Called before solve to register complementarity constraints with KNITRO solver, using complementarity information specified in the KTRIProblem object pointed to by this object. This function should not be called manually.
   @throws KTRException if the list of complementarity indices are not valid.
   */
  void addCompCons();

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
   * Registers a KTRMSProcessCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setMSProcessCallback();

   /**
   * Registers a KTRNewptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setNewptCallback();

  /**
   * This function is used to call the user-defined objective function and constraint evaluation, and pass the results back to the KNITRO solver.
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
  static int callbackEvalFC(int evalRequestCode, int n, int m, int nnzJ, int nnzH, const double* const x,
                            const double* const lambda, double* const obj, double* const c, double* const objGrad,
                            double* const jac, double* const hessian, double* const hessVector, void* userParams);
							
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
                                double const* const lambda, double* const obj, double* const c, double* const objGrad,
                                double* const jac, double* const hessian, double* const hessVector, void* userParams);

  /**
   * This function is used to call the user-defined MIP node callback, and pass the results back to the KNITRO solver. This function does not need to be called manually.
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
  static int callbackMipNode(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                             double const* const lambda, double* const obj, double* const c, double* const objGrad,
                             double* const jac, double* const hessian, double* const hessVector, void* userParams);

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
   * This function is used to call the user-defined output redirection function, and pass the results back to the KNITRO solver. This function does not need to be called manually.
   * @param str
   * @param userParams
   * @return
   */
  static int callbackRedirectOutput(const char * const str, void* userParams);

  /**
   * This function is used to call the user-defined gradient and Jacobian evaluation function, and pass the results back to the KNITRO solver. This function does not need to be called manually.
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
  static int callbackEvalGA(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                            double const* const lambda, double* const obj, double* const c, double* const objGrad,
                            double* const jac, double* const hessian, double* const hessVector, void* userParams);

  /**
   * This function is used to call the user-defined Hessian or Hessian-vector product evaluation function,
   * and pass the results back to the KNITRO solver. This function does not need to be called manually.
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
  static int callbackEvalHess(int evalRequestCode, int n, int m, int nnzJ, int nnzH, double const* const x,
                              double const* const lambda, double* const obj, double* const c, double* const objGrad,
                              double* const jac, double* const hessian, double* const hessVector, void* userParams);

  /**
   * The KTRSolver copy constructor is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   */
  KTRSolver(const KTRSolver&);

  /**
   * The KTRSolver copy operator is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   * @return
   */
  KTRSolver& operator= (const KTRSolver&);

  /**
   * Called by the KTRSolver constructor to register the problem properties with the KNITRO solver. Problem properties are defined by the KTRIProblem object passed to the constructor of this object.
   * @throws KTRException if KNITRO encounters a problem. This usually indicates some invalid problem properties.
   */
  virtual void initProblem();

  /**
   * True if variable integrality is to be ignored when calling the KNITRO solver.
   */
  bool _integralityRelaxed;

  /**
   * True if a KTRMIPNodeCallback is to be used by the KNITRO solver.
   */
  bool _useMipNodeCallback;

};
}
#include "impl/KTRSolver.hxx"


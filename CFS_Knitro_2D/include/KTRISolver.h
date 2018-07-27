/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <vector>
#include <string>

#include "knitro.h"
#include "zlm.h"

namespace knitro {

class KTRIProblem;

class KTRISolver {

public:
  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  KTRISolver(KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when specifying a gradient and Hessian evaluation other than the default exact gradient and hessian evaluations, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   * @param gradopt The type of gradient evaluation to use when solving the problem. See KTR_PARAM_GRADOPT in the KNITRO manual or in knitro.h for more information.
   * @param hessopt  The type of Hessian  evaluatoin to use when solving the problem. See KTR_PARAM_HESSOPT in the KNITRO manual or in knitro.h for more information.
   */
  KTRISolver(const ZLM * zlm, KTRIProblem * problem, int gradopt, int hessopt);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a local KNITRO license. Checks that the KNITRO license is valid and registers the problem structure with the KNITRO solver.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  KTRISolver(KTRIProblem * problem);

  /**
   * This constructor should be used when using the default exact gradient and Hessian evaluation, and when using a floating KNITRO license with a Ziena License Manager server. Registers the problem structure with the KNITRO solver.
   * @param zlm Pointer to a Ziena License Manager object containing network license information.
   * @param problem Pointer to a problem defining variable, constraint, objective, and function definition properties.
   */
  KTRISolver(const ZLM * zlm, KTRIProblem * problem);

  /**
   * Destructor for solver. Clears the KNITRO problem information stored by the KNITRO solver. This function shoulld not be called manually.
   */
  virtual ~KTRISolver();

  /**
   *
   * @return A pointer to the problem that is registered to the KNITRO solver, to be solved (or having already been solved) by this solver object.
   */
  KTRIProblem * getProblem();

  /**
   * This function should be called before calling solve() if a multistart initial point callback is to be called by the KNITRO solver. This requires a KTRMSInitptCallback object to be passed to the KTRIProblem object that was passed to this object's constructor.
   */
  void useMSInitptCallback();

  /**
   * This function should be called before calling solve() if a multistart process callback is to be called by the KNITRO solver. This requires a KTRMSProcessCallback object to be passed to the KTRIProblem object that was passed to this object's constructor.
   */
  void useMSProcessCallback();

  /**
   * This function should be called before calling solve() if a new point callback is to be called by the KNITRO solver. This requires a KTRNewptCallback object to be passed to the KTRIProblem object that was passed to this object's constructor.
   */
  void useNewptCallback();

  /**
   * Calls KNITRO to solve the problem, outputs problem information (unless all output is suppressed), and returns a KNITRO return code indicating solve status. See the KNITRO user manual for more information about return codes.
   * @param resetVariableBounds If this parameter is true or is not set, variable bounds are updated to the current bounds in the KTRIProblem pointed to by this solver object. If this parameter is false, the bounds originally set in the KTRSolver constructor are used.
   * @return KNITRO solver return code.
   */
  int solve(bool resetVariableBounds = true);

  /**
   * Resets all KNITRO parameters to default values.
   * @throws KTRException if KNITRO encounters an error. This is usually the result of a KNITRO license error.
   */
  void resetParamsToDefault();

  /**
   * Sets all KNITRO parameters specified by the input file. See "The KNITRO options file" in the KNITRO user manual for more information.
   * @param filename Name of a valid KNITRO parameters file.
   * @throws KTRException if KNITRO encounters an error, such as an invalid parameters file.
   */
  void loadParamFile(const std::string& filename);

  /**
   * Saves all currently set KNITRO parameters (and any defaults) to a KNITRO options file.
   * See "The KNITRO options file" in the KNITRO user manual for more information.
   * @param filename Name of the file in which to save KNITRO parameters.
   * @throws KTRException if KNITRO encounters an error. This is usually the result of a KNITRO license error, or insufficient write permissions.
   */
  void saveParamFile(const std::string& filename) const;

  /**
   * Set scaling parameters (factors and centers) on variables 
   * @param scaleFactors Vector of scaling factors on variables
   * @param scaleCenters Vector of centers on variables
   */
  void setVarScaling(const std::vector<double>& xScaleFactors, const std::vector<double>& xScaleCenters);

  /**
   * Set scaling paramter (objective)
   * @param objScaleFactor Scaling factor on objective
   */
  void setObjScaling(const double objScaleFactor);

  /**
   * Set honor bounds
   * @param honorBnds Indices of bounds for which bounds should be honored
   */
  void setHonorBounds(const std::vector<int>& honorBnds);

  /**
   * Set an integer-valued parameter using its integer identifier (see KNITRO user options in the KNITRO manual).
   * @param paramId Integer identifier of the parameter.
   * @param value Integer value of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(int paramId, int value);

  /**
   * Set an integer-valued parameter using its string name (see KNITRO user options in the KNITRO manual).
   * @param name String name of the parameter.
   * @param value Integer value of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(const std::string& name, int value);

  /**
   * Set a string-valued parameter using its integer identifier (see KNITRO user options in the KNITRO manual).
   * @param paramId Integer identifier of the parameter.
   * @param value String value of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(int paramId, const std::string& value);

  /**
   * Set a string-valued parameter using its string name (see KNITRO user options in the KNITRO manual).
   * @param name String name of the parameter.
   * @param value String of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(const std::string& name, const std::string& value);

  /**
   * Set a double-valued parameter using its integer identifier (see KNITRO user options in the KNITRO manual).
   * @param paramId Integer identifier of the parameter.
   * @param value Double value of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(int paramId, double value);

  /**
   * Set a double-valued parameter using its string name (see KNITRO user options in the KNITRO manual).
   * @param name String name of the parameter.
   * @param value Double of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  void setParam(const std::string& name, double value);

  /**
   *
   * @param paramId The integer identifier of an integer-valued parameter (see KNITRO user options in the KNITRO manual).
   * @return An integer-valued parameter value.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier is passed).
   */
  int getIntParam(int paramId) const;

  /**
   *
   * @param name The string name of an integer-valued parameter (see KNITRO user options in the KNITRO manual).
   * @return An integer-valued parameter value.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier is passed).
   */
  int getIntParam(const std::string& name) const;

  /**
   *
   * @param paramId The integer identifier of a double-valued parameter (see KNITRO user options in the KNITRO manual).
   * @return A double-valued parameter value.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier is passed).
   */
  double getDoubleParam(int paramId) const;

  /**
   *
   * @param name The string name of a double-valued parameter (see KNITRO user options in the KNITRO manual).
   * @return A double-valued parameter value.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier is passed).
   */
  double getDoubleParam(const std::string& name) const;

  /**
   *
   * @return A string containing the KNITRO release (version) information.
   */
  std::string getRelease() const;

  /**
   * Load a file of KNITRO tuner options. See "The KNITRO Tuner" in the KNITRO user manual for more information.
   * @param filename Name of the KNITRO tuner options files.
   * @throws KTRException if KNITRO encounters an error, such as an invalid parameters file.
   */
  void loadTunerFile(const std::string& filename);

  /**
   * This function should be called between successive calls to KTRSolver::solve(). See KTR_restart in the KNITRO user manual for information on what problem changes can be made between calling solve.
   * @param xInitial New primal variable initial values. If this vector is empty, KNITRO will choose initial primal variables automatically.
   * @param lambdaInitial New dual variable initial values. If this vector is empty, KNITRO will choose initial dual variables automatically.
   * @throws KTRException if the parameters are the wrong length. One or both arguments may be empty if initial primal and/or dual values are to be set automatically by KNITRO, but should otherwise specify initial values for all variables.
   */
  virtual void restart(const std::vector<double>& xInitial, const std::vector<double>& lambdaInitial) = 0;

  /**
   * Set relative stepsizes to use for the finite-difference gradient and Jacobian evaluations. See KTR_set_findiff_relstepsizes in the KNITRO user manual for more information.
   * @param relStepSizes Relative step sizes. This vector should have one entry for each problem variable, or be empty to set step sizes to default values.
   * @throws KTRException If relative steps are not set to valid values for either all variables or no variables (the latter by passing an empty vector argument).
   */
  void setFindiffRelstepsizes(const std::vector<double>& relStepSizes);

  /**
   *
   * @return The primal variable values found by the KNITRO solver.
   */
  const std::vector<double>& getXValues() const;

  /**
   *
   * @return The dual variable values found by the KNITRO solver.
   */
  const std::vector<double>& getLambdaValues() const;

  /**
   *
   * @param id The index of the primal variable.
   * @return Primal variable value for one index found by the KNITRO solver.
   */
  double getXValues(int id) const;

  /**
   *
   * @param id The index of the dual variable.
   * @return Dual variable value for one index found by the KNITRO solver.
   */
  double getLambdaValues(int id) const;

  /**
   *
   * @return The objective function value found by the KNITRO solver.
   */
  double getObjValue() const;

  /**
   *
   * @return The number of KNITRO evaluations from solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberIters() const;

  /**
   *
   * @return The number of conjugate gradient evaluations from solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  int getNumberCGIters() const;

  /**
   *
   * @return The absolute optimality error in the solution.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getAbsOptError() const;

  /**
   *
   * @return The absolute feasibility error in the solution.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual double getAbsFeasError() const { return KTR_INFBOUND; }

  /**
   *
   * @return The absolute MIP integrality gap from the MIP solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual double getMipAbsGap() const { return KTR_INFBOUND; }

  /**
   *
   * @return The relative optimality error in the solution.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  double getRelOptError() const;

  /**
   *
   * @return The value of the gradient found from solve.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  std::vector<double> getObjGradValues() const;

  /**
   *
   * @return The value of the Jacobian found from solve for each non-zero Jacobian entry defined in the KTRIProblem object.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual std::vector<double> getJacobianValues() const = 0;

  /**
   *
   * @param paramId Integer identifier of a KNITRO parameter.
   * @return String name of a KNITRO parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  std::string getParamName(int paramId) const;

  /**
   *
   * @param paramId Integer identifier of a KNITRO parameter.
   * @return Description of a KNITRO parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  std::string getParamDoc(int paramId) const;

  /**
   *
   * @param paramId Integer identifier of a KNITRO parameter.
   * @return Type of a KNITRO parameter (integer, double, or string) indiciated by a KNITRO parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  int getParamType(int paramId) const;

  /**
   *
   * @param paramId Integer identifier of a KNITRO parameter.
   * @return Number of possible values of a KNITRO parameter. Returns zero if there are not a finite of possible values of the parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  int getNumParamValues(int paramId) const;

  /**
   *
   * @param paramId Integer identifier of a KNITRO parameter.
   * @param answerId Index of a possible KNITRO parameter value, if the parameter has a finite number of possible values.
   * @return Description of the parameter value.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  std::string getParamValueDoc(int paramId, int answerId) const;

  /**
   *
   * @param name String name of a KNITRO parameter.
   * @return Integer identifier of a KNITRO parameter.
   * @throws KTRException if KNITRO encounters an error (e.g., if an invalid parameter indentifier or value is passed).
   */
  int getParamID(const std::string& name) const;

  /**
   * Registers a KTRMSProcessCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setMSProcessCallback() = 0;

  /**
   * Registers a KTRMSInitptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setMSInitptCallback() = 0;

  /**
   * Registers a KTRNewptCallback with the KNITRO solver. Called by solve(). This function does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem when registering the callback.
   */
  virtual void setNewptCallback() = 0;

  /**
   * Called internally when solve() is called. This function should not be called manually.
   * @param resetVariableBounds If this parameter is true, variable bounds are updated to the current bounds in the KTRIProblem pointed to by this solver object. If this parameter is false, the bounds originally set in the KTRSolver constructor are used.
   * @return KNITRO solver return code.
   */
  virtual int resolve(bool resetVariableBounds) = 0;

  /**
   * Called before solve to change variable bounds, using whatever variable bounds are specified in the KTRIProblem object pointed to by this object. To change variable bounds, variable bounds stored by the KTRIProblem should be changed. This function should not be called manually.
   * @param xLoBnds Variable lower bounds.
   * @param xUpBnds Variable upper bounds.
   * @throws KTRException if parameters are the wrong size. Bound vectors may be empty if all variables are unbounded below and/or above, but should otherwise specify bounds for all variables in the range [-KTR_INFBOUND, KTR_INFBOUND].
   */
  void chgVarBnds(const std::vector<double>& xLoBnds, const std::vector<double>& xUpBnds);

  /**
   * Checks whether the KNITRO license is valid. This functions calledby the constructor and does not need to be called manually.
   * @throws KTRException if a valid KNITRO license cannot be found.
   */
  void checkValidKNITROLicense() const;

  /**
   * Registers callbacks with KNITRO solver. This function is called automatically when solve() is called and does not need to be called manually.
   * @throws KTRException if KNITRO encounters a problem (this usually indicates a license error).
   */
  virtual void initCallbacks() = 0;

  /**
   * Function called by the constructor to register the problem with KNITRO. Does not need to be called manually.
   * @param gradopt Parameter specifying gradient evaluation type, set by the KNITRO constructor.
   * @param hessopt Parameter specifying Hessian evaluation type, set by the KNITRO constructor.
   */
  void construct(int gradopt = KTR_GRADOPT_EXACT, int hessopt = KTR_HESSOPT_EXACT);

  /**
   *
   * @return True if the problem passed to this object's constructor has any integer or binary variables, and integrality is not relaxed.
   */
  virtual bool isMipProblem() const = 0;

  /**
   * This function is used to call the user-defined output redirection function, and pass the results back to the KNITRO solver. This function does not need to be called manually.
   * @param str
   * @param userParams
   * @return
   */
  static int callbackRedirectOutput(const char * const str, void* userParams);
							
  /**
   * The KTRSolver copy constructor is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   */
  KTRISolver(const KTRISolver&);

  /**
   * The KTRSolver copy operator is unimplemented so that copies of a KTRSolver object cannot be made, as each KTRSolver object holds a unique KTR_context_ptr.
   * @param
   * @return
   */
  KTRISolver& operator=(const KTRISolver&);

  /**
   * A function used internally to print a KNITRO error code when the KNITRO solver returns one.
   * @param errorCode An integer-valued KNITRO error code. The section on return codes in the KNITRO user manual for more information on KNITRO return codes.
   * @return
   */
  std::string CreateErrorMessage(int errorCode);

  /**
   * Called by the KTRSolver constructor to register the problem properties with the KNITRO solver. Problem properties are defined by the KTRIProblem object passed to the constructor of this object.
   * @throws KTRException if KNITRO encounters a problem. This usually indicates some invalid problem properties.
   */
  virtual void initProblem() = 0;

  /**
   * Called by the KTRSolver::solve() method when calling the KNITRO solver. This returns primal variable values as a double pointer, but the public KTRSolver::getXValues() method should be used to access variable values.
   * @return A pointer to location where the KNITRO solver should write primal variable values.
   * @throws KTRException if the vector holding variable values is null.
   */
  double * var();

  /**
   * Called by the KTRSolver::solve() method when calling the KNITRO solver. This returns dual variable values as a double pointer, but the public KTRSolver::getLambdaValues() method should be used to access variable values.
   * @return A pointer to location where the KNITRO solver should write dual variable values.
   * @throws KTRException if the vector holding variable values is null.
   */
  double * dual();

  /**
   * Called by the KTRSolver::solve() method when calling the KNITRO solver. This returns the objective function value as a double pointer, but the public KTRSolver::getObjValue() method should be used to access the objective function value.
   * @return A pointer to location where the KNITRO solver should write the objective function value.
   */
  double * obj();

protected:
  /**
   * A pointer to the problem to be solved.
   */
  KTRIProblem * _problem;

  /**
   * The KTR_context_ptr stores problem information, parameters, and license information.
   */
  KTR_context_ptr _kc;

  /**
   * The objective function value.
   */
  double _obj;

  /**
   * Primal variable values.
   */
  std::vector<double> _xValues;

  /**
   * Dual variable values.
   */
  std::vector<double> _lambdaValues;

  /**
   * True if a KTRMSInitptCallback is to be used by the KNITRO solver.
   */
  bool _useMSInitptCallback;

  /**
   * True if a KTRMSProcessCallback is to be used by the KNITRO solver.
   */
  bool _useMSProcessCallback;

  /**
   * True if a KTRNewptCallback is to be used by the KNITRO solver.
   */
  bool _useNewptCallback;

  /**
   * Used internally to track whether the KNITRO solver is being called for the first time for this KTRSolver object.
   */
  bool _initialSolveCall;

};
}

#include "impl/KTRISolver.hxx"


#include "KTRSolver.h"
#include "customizedPrint.h"
#include "ProblemNLNC.h"
#include "PlanningProblem.h"
#include <cstdio>
#include <ctime>
#include <chrono>

/**
 * An example of solving a scfs problem.
 */

int main() {
  PlanningProblem pp;
  ProblemNLNC planning(&pp);

  // Create a solver
  knitro::KTRSolver solver(&planning,1,4);
  //solver.setParam(KTR_PARAM_PRESOLVE, 0);
  solver.setParam(KTR_PARAM_OPTTOL, 1.0e-1);
  solver.setParam(KTR_PARAM_ALG, loadAlgorithm());
  solver.setParam(KTR_PARAM_FEASTOL, 1.0e-4);
  //solver.setParam(KTR_PARAM_FTOL, 5.0e-6);
  solver.setParam(KTR_PARAM_XTOL, 1.0e-6);
  solver.setParam(KTR_PARAM_MAXIT, loadMaxIt());

  printMessage("solving");
  double tic = std::clock();
  solver.solve();
  double toc = std::clock();

  printTimeEllapse(tic,toc,"Computation time");
  
  std::vector<double> x_ = solver.getXValues();
  pp.printTrajectory(x_);

  std::vector<double> objGrad, jac, c;
  printMessage("Optimal cost: ");
  std::cout << planning.evaluateFC(x_, c, objGrad, jac) << std::endl; 

  //printMessage("Grad: ");
  //planning.evaluateGA(x_, objGrad, jac);
  //printVector(objGrad.data(), objGrad.size());
  //printVector(jac.data(), jac.size());

  return 0;
}
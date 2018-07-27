#include "KTRSolver.h"
#include "customizedPrint.h"
#include "ProblemNLNC.h"
#include "PlanningProblem.h"
#include <cstdio>
#include <ctime>
#include <chrono>


int main() {
  PlanningProblem pp;
  ProblemNLNC planning(&pp);

  std::vector<double> time(8);

  for (int i = 0; i < 8; i++){
    pp.nstep_ = 30 + i*10;
    // Create a solver
    knitro::KTRSolver solver(&planning,1,3);
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

    time[i] = (toc-tic)/CLOCKS_PER_SEC*1000;
  }
  printVector(time.data(),time.size());

  return 0;
}
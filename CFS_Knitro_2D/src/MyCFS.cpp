#include "KTRSolver.h"
#include "PlanningProblem.h"
#include "ProblemCFS.h"
#include "loadParameter.h"
#include <cstdio>
#include <ctime>
#include <chrono>

/**
 * An example of solving a problem with complementarity constraints.
 */
int main() {
  PlanningProblem pp;
  ProblemCFS cfs(&pp);

  //cfs.autotest();
  
  std::cout << "solving" << std::endl;

  double tic = std::clock();
  cfs.iteration(loadMaxIt(), 0.001);
  double toc = std::clock();

  cfs.printResult();

  std::cout << "Computation time: " << (toc-tic)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;
  return 0;
}
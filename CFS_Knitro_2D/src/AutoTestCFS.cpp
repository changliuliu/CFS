#include "KTRSolver.h"
#include "PlanningProblem.h"
#include "ProblemCFS.h"
#include "loadParameter.h"
#include "customizedPrint.h"
#include <cstdio>
#include <ctime>
#include <chrono>
#include <numeric>

/**
 * An example of solving a problem with complementarity constraints.
 */
int main() {
  PlanningProblem pp;
  ProblemCFS cfs(&pp);

  std::vector<double> process_time(8);
  std::vector<double> qp_time(8);

  for (int i = 0; i < 8; i++){
    pp.nstep_ = 30 + i*10;
    double tic = std::clock();
    cfs.iteration(loadMaxIt(), 0.0001);
    double toc = std::clock();
    process_time[i] = std::accumulate(cfs.process_time_.begin(), cfs.process_time_.end(), 0.0)/cfs.process_time_.size();
    qp_time[i] = std::accumulate(cfs.qp_time_.begin(), cfs.qp_time_.end(), 0.0)/cfs.qp_time_.size();
  }
  printVector(process_time.data(),process_time.size());
  printVector(qp_time.data(),qp_time.size());
  
  return 0;
}
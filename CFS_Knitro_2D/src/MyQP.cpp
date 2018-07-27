#include "KTRSolver.h"

#include "ProblemQP.h"

/**
 * An example of solving a problem with complementarity constraints.
 */
int main() {
  // Create a problem instance.

  int n_=2;
  int m_=2;
  std::vector< std::vector<double> > Lfull_(m_,std::vector<double>(n_));
  std::vector<double> S_(m_);
  std::vector< std::vector<double> > Hfull_(n_,std::vector<double>(n_));
  std::vector<double> f_(n_);

  std::vector<double> H_(2);
  std::vector<double> L_(2);
  std::vector<int> hessrows_(2);
  std::vector<int> hesscols_(2);
  std::vector<int> jacrows_(2);
  std::vector<int> jaccols_(2);

  Lfull_[0][0] = 1;
  Lfull_[1][1] = 1;
  S_[0] = -1;
  S_[1] = 1;
  Hfull_[0][0] = 1;
  Hfull_[1][1] = 1;

  H_[0] = 1; hessrows_[0] = 0; hesscols_[0] = 0;
  H_[1] = 1; hessrows_[1] = 1; hesscols_[1] = 1;
  L_[0] = 1; jacrows_[0] = 0; jaccols_[0] = 0; 
  L_[1] = 1; jacrows_[1] = 1; jaccols_[1] = 1; 

  ProblemQP instance(n_,m_,hessrows_,hesscols_,jacrows_,jaccols_);
  instance.setObjective(Hfull_,f_);
  instance.setConstraints(Lfull_,S_);

  // Create a solver
  knitro::KTRSolver solver(&instance);

  int solveStatus = solver.solve();
  std::vector<double> x1 = solver.getXValues();
  std::cout << "(" << x1[0] << "," << x1[1] << ")" << std::endl;

  // printSolutionResults(solver, solveStatus);

  ProblemQP instance_sparce(n_,m_,hessrows_,hesscols_,jacrows_,jaccols_);
  instance_sparce.setObjective(H_,f_);
  instance_sparce.setConstraints(L_,S_);

  // Create a solver
  std::cout << "solve sparce problem" << std::endl;
  knitro::KTRSolver solver_sparce(&instance_sparce);

  solver_sparce.solve();
  std::vector<double> x2 = solver_sparce.getXValues();
  std::cout << "(" << x2[0] << "," << x2[1] << ")" << std::endl;

  return 0;
}
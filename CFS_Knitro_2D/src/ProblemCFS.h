//#ifndef _PROBLEMCFS_H_
//#define _PROBLEMCFS_H_
#pragma once
#include "KTRSolver.h"
#include "KTRProblem.h"
#include "ProblemQP.h"
#include "d2problem.h"
#include "customizedPrint.h"
#include "PlanningProblem.h"
#include <math.h>
#include <cmath>
#include <vector>
#include <iomanip>

class ProblemCFS {

 public:
  //ProblemCFS(int nstep, int dim, int nobs)
  //   : nstep_(nstep), dim_(dim), nobs_(nobs) { }
  ProblemCFS(PlanningProblem* pp) {
    pp_ = pp;
  }

  PlanningProblem* pp_;  
  double iteration_time_;
  int iteration_;
  std::vector<double> qp_time_;
  std::vector<double> process_time_;
  std::vector<double> cost_;
  std::vector<double> soln_;

  bool checkConvergence(const std::vector<double>& x, std::vector<double>& xold, const double& tolX){
    double diff=0;
    int N = pp_->nstep_ * pp_->dim_;
    for (int i=0; i<N; i++){
      diff += pow(x[i] - xold[i],2);
        xold[i] = x[i];
    }
    std::cout << "diff = " << sqrt(diff) << "; tolX = " << tolX << std::endl;
    if (sqrt(diff)/N<tolX){
      return true;
    }
    else{
      return false;
    }
  }
/*
  bool autotest(){
    
    // Construct Quadratic Objective x'*Hfull_*x + f_*x;
    std::vector<double> H_;
    std::vector< std::vector<double> > Hfull_(nstep_*dim_,std::vector<double>(nstep_*dim_));
    std::vector<int> hessrows, hesscols;
    std::vector<double> f_(nstep_*dim_);
    H_.clear(); hessrows.clear(); hesscols.clear();

    double hij;
    for (int i=0; i<nstep_*dim_; i++){
      f_[i] = 0;
      for (int j=0; j<nstep_*dim_; j++)
      { 
        f_[i] += -2*Qref_[i][j]*xref_[j];
        if (i<=j){
          hij = Qref_[i][j] + Qself_[i][j];
          Hfull_[i][j] = hij;
          Hfull_[j][i] = hij;
          if (hij) {
            H_.push_back(hij);
            hessrows.push_back(i);
            hesscols.push_back(j);
          }
        }
      }
    }
    //printVector(hessrows.data(),hessrows.size());
    //printVector(hesscols.data(),hesscols.size());

    std::vector<int> jacrows, jaccols;
    setConstraintSparcity(jacrows,jaccols);

    // Instanciate a QP subproblem
    ProblemQP subproblem(nstep_*dim_,nstep_*nobs_,hessrows,hesscols,jacrows,jaccols);
    
    std::vector<double> x_(xref_);
    std::vector<double> obspoly_;
    std::vector< std::vector<double> > Lfull_(nstep_*nobs_, std::vector<double>(nstep_*dim_));
    std::vector<double> S_(nstep_*nobs_);
    std::vector<double> L_;

    // Get L_ and S_;
    L_.clear();
    for (int i=0; i<nstep_; i++){
      for (int j=0; j<nobs_; j++){
        std::vector<double> l(dim_);
        double s;
        //std::cout << "Step " << i << ", Obstacle " << j << std::endl;
        obspoly_ = Obs_[i][j];

        d2poly(obspoly_, x_.data()+i*dim_, l, s);

        //std::cout << "l = (" << l[0] << "," << l[1] << "); s = " << s << std::endl;

        Lfull_[i*nobs_+j][i*dim_] = l[0];
        Lfull_[i*nobs_+j][i*dim_ + 1] = l[1];
        S_[i*nobs_+j] = s;

        L_.push_back(l[0]);
        L_.push_back(l[1]);
      }
    }
    //test sparce
    subproblem.setObjective(H_,f_);
    subproblem.setConstraints(L_,S_);

    std::vector<double> c1, objGrad1, jac1;
    double obj1 = subproblem.evaluateFC(xref_, c1, objGrad1, jac1);
    subproblem.evaluateGA(xref_, objGrad1, jac1);

    subproblem.setObjective(Hfull_,f_);
    subproblem.setConstraints(Lfull_,S_);

    std::vector<double> c2, objGrad2, jac2;
    double obj2 = subproblem.evaluateFC(xref_, c2, objGrad2, jac2);
    subproblem.evaluateGA(xref_, objGrad2, jac2);

    printMessage("The difference between obj is:");
    std::cout << obj1-obj2 << std::endl;
    printVector(c1.data(),c1.size());
    printVector(c2.data(),c2.size());
    printVector(objGrad1.data(),objGrad1.size());
    printVector(objGrad2.data(),objGrad2.size());
    printVector(jac1.data(),jac1.size());
    printVector(jac2.data(),jac2.size());
  }*/

  int iteration(int iterMax, double tolX)
  {
    std::vector<double> xref(pp_->nstep_ * (pp_->dim_ + pp_->neq_));
    pp_->setReferenceTrajectory(xref);
    //pp_->printTrajectory(xref);

    bool sparce_obj = true;
    bool sparce_con = true;

    int two_side = 2;
    
    std::vector<double> H_;
    std::vector< std::vector<double> > Hfull_(pp_->nstep_ * pp_->dim_, std::vector<double>(pp_->nstep_ * pp_->dim_));
    std::vector<double> f_(pp_->nstep_*(pp_->dim_ + pp_->neq_));
    std::vector<int> hessrows, hesscols;

    pp_->setSparceObjective(xref, H_, f_, hessrows, hesscols);
    printMessage("Sparce objective set.");

    std::vector<int> jacrows, jaccols;
    pp_->setConstraintSparcity(jacrows,jaccols,two_side);
    printMessage("Sparce constraint set.");

    // Instanciate a QP subproblem
    int n_variable = pp_->nstep_ * (pp_->dim_ + pp_->neq_);
    int n_constraint = pp_->nstep_ * pp_->nobs_ + (pp_->nstep_ - 2) * pp_->neq_ * two_side;
    ProblemQP subproblem(n_variable, n_constraint , hessrows,hesscols,jacrows,jaccols);
    if (sparce_obj){
      subproblem.setObjective(H_,f_);
    }
    else{
      subproblem.setObjective(Hfull_,f_);
    }
    subproblem.setInitial(xref,pp_->dim_,pp_->nstep_,2);

    // Create a solver
    knitro::KTRSolver solver(&subproblem);
    solver.setParam(KTR_PARAM_OPTTOL, 1.0e-3);
    solver.setParam(KTR_PARAM_ALG, 1);
    solver.setParam(KTR_PARAM_PRESOLVE, 1);
    //solver.setParam(KTR_PARAM_XTOL, 1.0e-3);
    int solveStatus;

    printMessage("Solver Created in CFS");
    
    std::vector<double> x_(xref);
    std::vector<double> xold_(xref);

    std::vector<double> S_(pp_->nstep_ * (pp_->nobs_ + pp_->neq_));
    double iter_start_time = 0;
    double iter_end_time = 0;
    double qp_start_time = 0;
    double qp_end_time = 0;
    double process_start_time = 0;
    double process_end_time = 0;

    // The iteration
    qp_time_.clear();
    cost_.clear();
    cost_.push_back(subproblem.evaluateObj(xref));
    std::vector<double> lambda(pp_->nstep_ * (pp_->nobs_ + pp_->neq_));
    iter_start_time = std::clock();
    for (int k=0; k<iterMax; k++){
      //std::cout << "----------------------" << std::endl;
      //std::cout << "Iteration " << k << std::endl;

      // Processing
      process_start_time = std::clock();
      if (sparce_con){
        std::vector<double> L_(jaccols.size());
        if (k<0) {pp_->linConstraint(x_, L_, S_, 0);}
        else{
          pp_->linConstraint(x_, L_, S_, two_side);
        }
        subproblem.setConstraints(L_,S_);
      }
      else{
        std::vector< std::vector<double> > Lfull_(pp_->nstep_*pp_->nobs_, std::vector<double>(pp_->nstep_ * pp_->dim_));
        pp_->linConstraint(x_, Lfull_, S_);
        subproblem.setConstraints(Lfull_,S_);
      }
      process_end_time = std::clock();
      //printTimeEllapse(process_start_time, process_end_time, "Process Time");
      process_time_.push_back((process_end_time - process_start_time)/CLOCKS_PER_SEC*1000);

      printMessage("Specify initial");
      subproblem.setInitial(x_,pp_->dim_,pp_->nstep_);
      
      // Solve the subproblem
      qp_start_time = std::clock();
      solveStatus = solver.solve();
      qp_end_time = std::clock();
      //printTimeEllapse(qp_start_time, qp_end_time, "QP Time");
      qp_time_.push_back((qp_end_time - qp_start_time)/CLOCKS_PER_SEC*1000);

      x_ = solver.getXValues();
      //pp_->printTrajectory(x_);
      lambda = solver.getLambdaValues();
      if (pp_->neq_) {pp_->normTrajectory(x_);}
      //pp_->printTrajectory(x_);
      cost_.push_back(subproblem.evaluateObj(x_));

      // Convergence check
      if (checkConvergence(x_,xold_,tolX)){
        std::cout << "Converged at step " << k << std::endl;
        iteration_ = k;
        break;
      }
      solver.restart(x_, lambda);
    }
    soln_ = x_;
    iter_end_time = std::clock();
    //printTimeEllapse(iter_start_time, iter_end_time, "Iteration Time");
    iteration_time_ = (iter_end_time - iter_start_time)/CLOCKS_PER_SEC*1000;
    return iteration_;
  }
  
  int printResult(){
    printVector(process_time_.data(),process_time_.size());
    printVector(qp_time_.data(),qp_time_.size());
    printVector(cost_.data(),cost_.size());
    //pp_->printTrajectory(soln_);
    std::cout << "total iteration time = " << iteration_time_ << "ms" << std::endl;
  }
};

//#endif

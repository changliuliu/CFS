#include "KTRSolver.h"
#include "customizedPrint.h"
#include "ProblemNLNC.h"
#include "ProblemCFS.h"
#include <cstdio>
#include <ctime>
#include <chrono>

/**
 * An example of solving a scfs problem.
 */
int nstep_ = 20;
int dim_ = 2;
int nobs_ = 1;
int neq_ = 1;

int printTrajectory(const std::vector<double>& xref){
  std::cout << "Reference Trajectory:" << std::endl;
  std::cout << "["
  for (int i=0; i < nstep_; i++)
  {
    std::cout << xref[i*dim_];
    if (i < nstep_ - 1){
      std::cout << ",";
    } 
  }
  std::cout << ";" << std::endl;
  for (int i=0; i < nstep_; i++)
  {
    std::cout << xref[i*dim_+1];
    if (i < nstep_ - 1){
      std::cout << ",";
    } 
  }
  std::cout << "];" << std::endl;
  return 0;
}

int setReferenceTrajectory(std::vector<double>& xref){
  std::vector<double> start(2);
  std::vector<double> end(2);
  end[0] = 9;
  for (int i=0; i < nstep_; i++)
  {
    xref[i*dim_] = ((nstep_-1-i)*start[0] + i*end[0])/(nstep_-1);
    xref[i*dim_+1] = ((nstep_-1-i)*start[1] + i*end[1])/(nstep_-1);
  }
  //std::cout << "Start Point: (" << start[0] << "," << start[1] << ")" << std::endl;
  //std::cout << "End Point: (" << end[0] << "," << end[1] << ")" << std::endl;
  return 0;
}

int setConstraintSparcity(std::vector<int>& jacrows, std::vector<int>& jaccols){
    jacrows.clear(); jaccols.clear();
    // Get L_ and S_ for obstacles;
    for (int i=0; i<nstep_; i++){
        for (int j=0; j<nobs_; j++){
          jacrows.push_back(i*nobs_+j);
          jaccols.push_back(i*dim_);
          //std::cout << i*nobs_+j << "," << i*dim_<< std::endl;
          jacrows.push_back(i*nobs_+j);
          jaccols.push_back(i*dim_ + 1);
          //std::cout << i*nobs_+j << "," << i*dim_+1 << std::endl;
        }
    }
    // Get L_ and S_ for equality constraints;
    if (!neq_) {return 0;} 
    for (int i=2; i<nstep_; i++){
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-2)*dim_);
          //std::cout << nstep_*nobs_+i << "," << (i-2)*dim_ << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-2)*dim_+1);
          //std::cout << nstep_*nobs_+i << "," << (i-2)*dim_+1 << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-1)*dim_);
          //std::cout << nstep_*nobs_+i << "," << (i-1)*dim_ << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-1)*dim_+1);
          //std::cout << nstep_*nobs_+i << "," << (i-1)*dim_+1 << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-0)*dim_);
          //std::cout << nstep_*nobs_+i << "," << (i-0)*dim_ << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back((i-0)*dim_+1);
          //std::cout << nstep_*nobs_+i << "," << (i-0)*dim_+1 << std::endl;
          jacrows.push_back(nstep_*nobs_+i-2);
          jaccols.push_back(nstep_*nobs_*dim_+i);
          //std::cout << nstep_*nobs_+i << "," << nstep_*nobs_*dim_+i << std::endl;
    }
    return 0;
  }

int setCostMatrix(std::vector< std::vector<double> >& Qref, std::vector< std::vector<double> >& Qself, std::vector< std::vector<double> >& Qeq){
  double weight_ref_[] = {1,10,4};
  double weight_self_[] = {0,0,2};
  std::vector< std::vector<double> > Pos_(Qref);
  std::vector< std::vector<double> > Vel_(Qref);
  std::vector< std::vector<double> > Adiff_((nstep_-2)*dim_, std::vector<double>(nstep_*dim_));
  std::vector< std::vector<double> > Acc_(Qref);
  for (int i=0; i < nstep_*dim_; i++)
  {
    Pos_[i][i] = 1;
    if ((i >= dim_) && (i<(nstep_-1)*dim_)){
      Vel_[i][i] = 2;
      Vel_[i][i-dim_] = -1;
      Vel_[i-dim_][i] = -1;
      Vel_[i][i+dim_] = -1;
      Vel_[i+dim_][i] = -1;
    }else{
      Vel_[i][i] = 1;
    }
  }

  for (int i=0; i < (nstep_-2)*dim_; i++){
    Adiff_[i][i] = 1;
    if (i + 2 < nstep_*dim_){
      Adiff_[i][i+2] = -2;
    }
    if (i + 4 < nstep_*dim_){
      Adiff_[i][i+4] = 1;
    }
  }

  for (int i=0; i < nstep_*dim_; i++)
  {
    for (int j=0; j < nstep_*dim_; j++)
    {
      Acc_[i][j] = 0;
      for (int k=0; k < (nstep_-2)*dim_; k++)
      {
        Acc_[i][j] += Adiff_[k][i]*Adiff_[k][j];
      }
    }
  }

  for (int i=0; i < nstep_*dim_; i++){
    for (int j=0; j < nstep_*dim_; j++){
      Qref[i][j] = weight_ref_[0] * Pos_[i][j] + weight_ref_[1] * Vel_[i][j] + weight_ref_[2] * Acc_[i][j];
      Qself[i][j] = weight_self_[0] * Pos_[i][j] + weight_self_[1] * Vel_[i][j] + weight_self_[2] * Acc_[i][j];
    }
  }

  for (int i=0; i < nstep_*neq_; i++){
    Qeq[i][i] = 5;
  }
  return 0;
}

int setObstacle(std::vector< std::vector< std::vector<double> > >& Obs){
  double obspoly[]={3, -1, 5, -1.1, 6, 3, 2, 3};
  for (int i=0; i<nstep_; i++){
    for (int j=0; j<nobs_; j++){
      for (int k=0; k<8; k++){
        Obs[i][j][k] = obspoly[k];
      }
    }
  }
  return 0;
}

int setObjective(const std::vector<double>& xref_, std::vector<std::vector<double> >& H_, std::vector<double>& f_){
  std::vector< std::vector<double> > Qref(nstep_*dim_,std::vector<double>(nstep_*dim_));
  std::vector< std::vector<double> > Qself(nstep_*dim_,std::vector<double>(nstep_*dim_));
  std::vector< std::vector<double> > Qeq(nstep_*neq_,std::vector<double>(nstep_*neq_));
  setCostMatrix(Qref, Qself, Qeq);
  int sparcity_H = 0;
  for (int i=0; i<nstep_*dim_; i++){
    f_[i] = 0;
    for (int j=0; j<nstep_*dim_; j++)
    {
      H_[i][j] = Qref[i][j] + Qself[i][j];
      if ((H_[i][j]) && (i<=j)) {sparcity_H++;}
      f_[i] += -2*Qref[i][j]*xref_[j];
    }
  }
  printMessage("Cost for path");
  for (int i=nstep_*dim_; i<nstep_*(dim_+neq_); i++){
    for (int j=nstep_*dim_; j<nstep_*(dim_+neq_); j++)
    {
      H_[i][j] = Qeq[i-nstep_*dim_][j-nstep_*dim_];
      if ((H_[i][j]) && (i<=j)) {sparcity_H++;}
    }
  }
  //printMatrix(H_.data(),H_.size(),H_[0].size());
  return sparcity_H;
}


int main() {
  // Create a problem instance.
  std::vector< std::vector<double> > H_(nstep_*(dim_+neq_),std::vector<double>(nstep_*(dim_+neq_)));
  std::vector<double> f_(nstep_*(dim_+neq_));
  std::vector< std::vector<double> > Qeq_(H_);
  std::vector<double> xref_(nstep_*(dim_+neq_));
  std::vector< std::vector< std::vector<double> > > Obs_(nstep_,std::vector< std::vector<double> >(nobs_, std::vector<double>(dim_*4)));

  // Specify Reference Trajectory
  setReferenceTrajectory(xref_);

  // Specify Cost
  int sparcity_H = setObjective(xref_, H_, f_);

  // Specify Obstacle
  setObstacle(Obs_);

  std::vector<int> jacrows, jaccols;
  setConstraintSparcity(jacrows,jaccols);
    
  int sparcity_J = jacrows.size();

  ProblemNLNC planning(nstep_, dim_, nobs_, neq_, sparcity_J, sparcity_H);
  
  planning.setObjective(H_, f_);
  planning.setInitial(xref_);
  planning.setConstraints(Obs_);
  planning.setDerivativeProperties(jacrows, jaccols);

  //std::cout << "Initialization: " << scfs.xref_.size() << std::endl;

  // Create a solver
  knitro::KTRSolver solver(&planning,2,1);
  solver.setParam(KTR_PARAM_OPTTOL, 1.0e-2);
  solver.setParam(KTR_PARAM_ALG, 5);
  solver.setParam(KTR_PARAM_FEASTOL, 1.0e-4);
  solver.setParam(KTR_PARAM_XTOL, 1.0e-3);

  printMessage("solving");
  double tic = std::clock();
  solver.solve();
  double toc = std::clock();

  std::cout << "Computation time: " << (toc-tic)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;
  
  std::vector<double> x_ = solver.getXValues();
  printTrajectory(x_);

  return 0;
}
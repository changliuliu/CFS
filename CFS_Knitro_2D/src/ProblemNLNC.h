#pragma once
#include "KTRSolver.h"
#include "KTRProblem.h"
#include "d2problem.h"
#include "PlanningProblem.h"
#include <math.h>
#include <cmath>
#include <vector>

class ProblemNLNC: public knitro::KTRProblem {

 public:
  ProblemNLNC(PlanningProblem* pp): KTRProblem(pp->nstep_ * (pp->dim_ + pp->neq_), pp->nstep_ * pp->nobs_ + (pp->nstep_-2)*pp->neq_, pp->getJacSparcity(), pp->getHessSparcity()) 
  {
    pp_ = pp;
    xref_.reserve(pp_->nstep_ * (pp_->dim_ + pp_->neq_));
    pp_->setReferenceTrajectory(xref_);
    //printMessage("Reference Set");
    //pp_->printTrajectory(xref_);
    setInitial(xref_,2);

    pp_->setSparceObjective(xref_, H_, f_, hessrows_, hesscols_);
    //printVector(f_.data(),f_.size());
    //printMessage("Sparce objective set.");

    pp_->setConstraintSparcity(jacrows_, jaccols_);
    //printMessage("Sparce constraint set.");

    pp_->setObstacle(Obs_);
    //printMessage("Obstacle set.");

    int sparce_H = pp->getHessSparcity();
    int sparce_J = pp->getJacSparcity();

    //std::cout << "Problem size: " << pp->nstep_ * (pp->dim_ + pp->neq_)<< "," << pp->nstep_ * pp->nobs_ + (pp->nstep_-0)*pp->neq_ << ",";
    //std::cout << sparce_H << "," << sparce_J << std::endl;

    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    setDerivativeProperties(jacrows_, jaccols_);
    setHessianProperties(hessrows_, hesscols_);

    //std::cout << xref_.size() << std::endl;
    
  }

  // ProblemNLNC(int nstep, int dim, int nobs, int neq, int sparcity_J, int sparcity_H)
  //     : KTRProblem(nstep*(dim+neq), nstep*(nobs)+(nstep-2)*neq, sparcity_J, sparcity_H), nstep_(nstep), dim_(dim), nobs_(nobs), neq_(neq) {
  //   n_ = nstep*(dim+neq);
  //   m_ = nstep*(nobs)+(nstep-2)*neq;
  //   setObjectiveProperties();
  //   setVariableProperties();
  //   setConstraintProperties();
  //   sparce_H = false;
  //   sparce_J = false;
  //   printMessage("A NLNC without exact gradient is initialized.");
  // }

  // ProblemNLNC(int nstep, int dim, int nobs, int neq)
  //     : KTRProblem(nstep*(dim+neq), nstep*(nobs)+(nstep-2)*neq), nstep_(nstep), dim_(dim), nobs_(nobs), neq_(neq) {
  //   n_ = nstep*(dim+neq);
  //   m_ = nstep*(nobs)+(nstep-2)*neq;
  //   setObjectiveProperties();
  //   setVariableProperties();
  //   setConstraintProperties();
  //   sparce_H = false;
  //   sparce_J = false;
  //   printMessage("A NLNC without exact gradient is initialized.");
  // }



  /*int setConstraints(std::vector< std::vector< std::vector<double> > >& Obs, double margin = 0){
    Obs_ = Obs;
    margin_ = margin;
    return 0;
  }

  void setObjective(const std::vector<double>& H, const std::vector<double>& f){
    H_ = H;
    f_ = f;
    //if (!sparce_H) {setHessianProperties();}
    sparce_H = true;
    printMessage("Set Sparce Objective");
  }

  void setObjective(const std::vector< std::vector<double> >& Hfull, const std::vector<double>& f){
    Hfull_ = Hfull;
    f_ = f;
    if (!sparce_H) {setHessianProperties();}
    sparce_H = false;
    printMessage("Set Matrix Objective");
  }*/

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) 
  {
      std::vector<double> L; 
      double S;
      c.clear();
      for (int i=0; i<pp_->nstep_; i++){
        for (int j=0; j<pp_->nobs_; j++){
          c.push_back(d2poly(Obs_[i][j], x.data()+i*pp_->dim_, L, S)-pp_->margin_);
        }
      }
      for (int i=2; i<pp_->nstep_; i++){
        for (int j=0; j<pp_->neq_; j++){
          c.push_back(getYawRateError(x.data()+i*pp_->dim_, x.data()+(i-1)*pp_->dim_, x.data()+(i-2)*pp_->dim_,x.data()+pp_->nstep_*pp_->dim_+i*pp_->neq_+j));
          //c.push_back(getYawRate(x.data()+i*pp_->dim_, x.data()+(i-1)*pp_->dim_, x.data()+(i-2)*pp_->dim_)-x[pp_->nstep_*pp_->dim_+i*pp_->neq_+j]);
        }
      }
      /*
      for (int i=2; i<pp_->nstep_-1; i++){
          c.push_back(pow(x[i*dim_]-x[(i-1)*dim_],2) + pow(x[i*dim_+1]-x[(i-1)*dim_+1],2));
      }*/

    // Compute Objective function
    //printMessage("Evaluate obj");
    double obj=0;
      for (int k=0; k<H_.size(); k++){
        if (hessrows_[k] == hesscols_[k]) {
          obj += x[hessrows_[k]]*H_[k]*x[hesscols_[k]];
        }
        else{
          obj += 2 * x[hessrows_[k]]*H_[k]*x[hesscols_[k]];
        }
      }
      for (int j=0; j<f_.size(); j++){
        obj += x[j]*f_[j];
      }
    return obj;
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    //printMessage("Evaluate objGrad");
    //printVector(f_.data(),f_.size());
    objGrad = f_;
    for (int k=0; k<H_.size(); k++){
      if (hessrows_[k]!=hesscols_[k]){
        objGrad[hessrows_[k]] += 2 * H_[k] * x[hesscols_[k]];
        objGrad[hesscols_[k]] += 2 * H_[k] * x[hessrows_[k]];
      }
      else{
        objGrad[hessrows_[k]] += 2 * H_[k] * x[hesscols_[k]];
      }
    }
    //printVector(objGrad.data(),objGrad.size());
    
    std::vector<double> L,S; 
    L.reserve(jacrows_.size());
    S.reserve(jaccols_.size());
    jac.clear();
    pp_->linConstraint(x, L, S);
    for (int k=0; k<L.size(); k++){
        jac.push_back(-L[k]);
    }
    //printVector(jac.data(),jac.size());
    return 0;
  }

  int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& hess) 
  {
    //printMessage("Evalute Hessian");
    hess.clear();
    if (sparce_H) {
      for (int k=0; k<H_.size(); k++){
        hess.push_back(2*H_[k]*objScaler);
      }
    }
    else{
      for (int j=0; j<Hfull_.size(); j++){
        for (int i=j; i<Hfull_.size(); i++){
          if (Hfull_[j][i])
          {
            hess.push_back(2 * Hfull_[j][i] * objScaler);
          }
        }
      }
    }
    return 0;
  }

  void setInitial(std::vector<double>& xref, int fix = 1, int dim_ = 2)
  {
    //printMessage("setInitial");
    for (int i=0; i< pp_->nstep_ * (pp_->dim_ + pp_->neq_); i++)
    {
      setXInitial(i, xref[i]);
      //std::cout << i << "," << xref[i] << std::endl;
    }
    for (int i=0; i<pp_->dim_*fix; i++){
      setVarLoBnds(i, xref[i]);
      setVarUpBnds(i, xref[i]);
      setVarLoBnds((pp_->nstep_-fix)*pp_->dim_+i, xref[(pp_->nstep_-fix)*pp_->dim_+i]);
      setVarUpBnds((pp_->nstep_-fix)*pp_->dim_+i, xref[(pp_->nstep_-fix)*pp_->dim_+i]);
    }
  }

  void setDerivativeProperties(std::vector<int>& jacrows, std::vector<int>& jaccols) {
    //printMessage("Derivative Property");
    setJacIndexCons(jacrows);
    setJacIndexVars(jaccols);
    //printVector(jacrows.data(),jacrows.size());
    //printVector(jaccols.data(),jaccols.size());
  }

  void setHessianProperties(std::vector<int>& hessrows, std::vector<int>& hesscols) {
    //printMessage("Hessian Property");
    setHessIndexRows(hessrows);
    setHessIndexCols(hesscols);
    std::vector<int> getrows = getHessIndexCols();
    //printVector(getrows.data(),getrows.size());
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjQuadratic);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    for (int i=0; i<xref_.size(); i++)
    {
      setVarLoBnds(i, -KTR_INFBOUND);
      setVarUpBnds(i, KTR_INFBOUND);
      setXInitial(i, 0.0);
    }
  }

  void setConstraintProperties() {
    for (int i=0; i<pp_->nstep_*pp_->nobs_; i++)
    {
      setConTypes(i, knitro::KTREnums::ConstraintType::ConGeneral);
      setConLoBnds(i, 0.0);
      setConUpBnds(i, KTR_INFBOUND);
    }
    //printMessage("Equality Constrint");
    for (int i=pp_->nstep_*pp_->nobs_; i<pp_->nstep_*pp_->nobs_ + (pp_->nstep_-2)*pp_->neq_; i++)
    {
      //std::cout << i << std::endl;
      setConTypes(i, knitro::KTREnums::ConstraintType::ConQuadratic);
      setConLoBnds(i, 0.0);
      setConUpBnds(i, 0.0);
    }
  }

/*  void setDerivativeProperties() {
    printMessage("Derivative Property");
    int counter = 0;
    for (int j=0; j<m_; j++){
        for (int i=0; i<n_; i++){
          if (Lfull_[j][i])
          {
            setJacIndexCons(counter,j);
            setJacIndexVars(counter,i);
            counter ++;
          }
        }
    }
  }*/

  void setHessianProperties(){
    //printMessage("Hessian Property");
    int counter = 0;
    for (int j=0; j<xref_.size(); j++){
        for (int i=j; i<xref_.size(); i++){
          if (Hfull_[j][i])
          {
            setHessIndexRows(counter,j);
            setHessIndexCols(counter,i);;
            counter ++;
          }
        }
    }
  }

  PlanningProblem* pp_;
  bool sparce_H;
  bool sparce_J; 
  std::vector<double> xref_;
  std::vector< std::vector< std::vector<double> > > Obs_;
  std::vector< std::vector<double> > Hfull_;
  std::vector<double> H_;
  std::vector<double> f_;
  std::vector<int> hessrows_; 
  std::vector<int> hesscols_; 
  std::vector<int> jacrows_;
  std::vector<int> jaccols_;

};

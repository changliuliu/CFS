//#ifndef _PROBLEMQP_H_
//#define _PROBLEMQP_H_
#pragma once
#include "KTRSolver.h"
#include "KTRProblem.h"
#include "customizedPrint.h"

/**
 *  Changliu Liu
 *  May 2017
 */
class ProblemQP : public knitro::KTRProblem {

 public:

  // This initializes the problem with list form
  ProblemQP(int n, int m, std::vector<int>& hessrows, std::vector<int>& hesscols, std::vector<int>& jacrows, std::vector<int>& jaccols)
      : KTRProblem(n,m,jaccols.size(),hesscols.size()), n_(n), m_(m), hessrows_(hessrows), hesscols_(hesscols), jacrows_(jacrows), jaccols_(jaccols)
  {
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();

    setJacIndexCons(jacrows);
    setJacIndexVars(jaccols);
    setHessIndexRows(hessrows);
    setHessIndexCols(hesscols);
    sparce_H = true;
    sparce_J = true;
    std::cout << "Problem size: " << n << ", " << m << std::endl;
    std::cout << "A sparce QP is initialized. Sparcity:" << jaccols.size() << "," << hesscols.size() << std::endl; 
  }

  // This initializes the problem with matrix form
  ProblemQP(int n, int m): KTRProblem(n,m), n_(n), m_(m){ 
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    sparce_H = false;
    sparce_J = false;
    std::cout << "A dense QP without exact gradient or hessian is initialized." << std::endl;
  }

  ProblemQP(int n, int m, int sparcity_H, int sparcity_J): KTRProblem(n,m,sparcity_J,sparcity_H), n_(n), m_(m){
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    sparce_H = false;
    sparce_J = false;
    std::cout << "A dense QP with exact gradient is initialized." << std::endl;
  }

  int n_; // Number of decision variables
  int m_; // Number of constraints
  bool sparce_H;
  bool sparce_J; 
  std::vector< std::vector<double> > Lfull_;
  std::vector< std::vector<double> > Hfull_;
  std::vector<double> L_;
  std::vector<double> S_;
  std::vector<double> H_;
  std::vector<double> f_;
  std::vector<int> hessrows_; 
  std::vector<int> hesscols_; 
  std::vector<int> jacrows_;
  std::vector<int> jaccols_;

  void setObjective(const std::vector<double>& H, const std::vector<double>& f){
    H_ = H;
    f_ = f;
    if (!sparce_H) {setHessianProperties();}
    sparce_H = true;
    printMessage("Set Sparce Objective");
  }

  void setObjective(const std::vector< std::vector<double> >& Hfull, const std::vector<double>& f){
    Hfull_ = Hfull;
    f_ = f;
    if (!sparce_H) {setHessianProperties();}
    sparce_H = false;
    printMessage("Set Matrix Objective");
  }

  void setConstraints(const std::vector<double>& L, const std::vector<double>& S) {
    L_ = L;
    S_ = S;
    if (!sparce_J) {setDerivativeProperties();}
    sparce_J = true;
    printMessage("Set Sparce Constraint");
  }

  void setConstraints(const std::vector< std::vector<double> >& Lfull, const std::vector<double>& S) {
    Lfull_ = Lfull;
    S_ = S;
    if (!sparce_J) {setDerivativeProperties();}
    sparce_J = false;
    printMessage("Set Matrix Constraint");
  }

  double evaluateObj(const std::vector<double>& x) 
  {
    double obj=0;
    if (sparce_H){
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
    }
    else
    {
      for (int j=0; j<n_; j++){
        for (int i=0; i<n_; i++){
          obj += x[j]*Hfull_[j][i]*x[i];
        }
        obj += x[j]*f_[j];
      }
    }

    return obj;
  }

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) 
  {
    //printVector(x.data(),x.size());
    // Linear inequality constraint.
    c = S_;
    //printMessage("Evaluate constraint");
    if (sparce_J) {
      for (int k=0; k<jacrows_.size(); k++){
        c[jacrows_[k]] -= L_[k]*x[jaccols_[k]];
      }
    }
    else{
      for (int j=0; j<m_; j++){
        for (int i=0; i<n_; i++){
          c[j] -= Lfull_[j][i]*x[i];
        }
      }
    }

    // Compute Objective function
    //printMessage("Evaluate obj");
    double obj=0;
    if (sparce_H){
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
    }
    else
    {
      for (int j=0; j<n_; j++){
        for (int i=0; i<n_; i++){
          obj += x[j]*Hfull_[j][i]*x[i];
        }
        obj += x[j]*f_[j];
      }
    }

    return obj;
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    //printMessage("Evaluate objGrad");
    objGrad = f_;
    if (sparce_H){
      for (int k=0; k<H_.size(); k++){
        if (hessrows_[k]!=hesscols_[k]){
          objGrad[hessrows_[k]] += 2 * H_[k] * x[hesscols_[k]];
          objGrad[hesscols_[k]] += 2 * H_[k] * x[hessrows_[k]];
        }
        else{
          objGrad[hessrows_[k]] += 2 * H_[k] * x[hesscols_[k]];
        }
      }
    }
    else{
      for (int j=0; j<n_; j++){
        for (int i=0; i<n_; i++){
          objGrad[j] += 2 * Hfull_[j][i] * x[i];
        }
      }
    }

    //printMessage("Evaluate Jacobian");
    jac.clear();
    if (sparce_J){
      for (int k=0; k<jacrows_.size(); k++){
        jac.push_back(-L_[k]);
      }
    }
    else{
      for (int j=0; j<m_; j++){
        for (int i=0; i<n_; i++){
          if (Lfull_[j][i]!=0){
            jac.push_back(-Lfull_[j][i]);
          }
        }
      }
    }

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
      for (int j=0; j<n_; j++){
        for (int i=j; i<n_; i++){
          if (Hfull_[j][i])
          {
            hess.push_back(2 * Hfull_[j][i] * objScaler);
          }
        }
      }
    }
    return 0;
  }

  void setInitial(std::vector<double>& xref, int dim = 2, int nstep = 0, int fix = 1)
  {
    for (int i=0; i<n_; i++)
    {
      setXInitial(i, xref[i]);
    }
    for (int i=0; i<dim * fix; i++){
      setVarLoBnds(i, xref[i]);
      setVarUpBnds(i, xref[i]);
      setVarLoBnds((nstep-fix)*dim+i, xref[(nstep-fix)*dim+i]);
      setVarUpBnds((nstep-fix)*dim+i, xref[(nstep-fix)*dim+i]);
    }
  }

  void setDerivativeProperties(std::vector<int>& jacrows, std::vector<int>& jaccols) {
    printMessage("Derivative Property");
    setJacIndexCons(jacrows);
    setJacIndexVars(jaccols);
  }

  void setHessianProperties(std::vector<int>& hessrows, std::vector<int>& hesscols) {
    printMessage("Hessian Property");
    setHessIndexRows(hessrows);
    setHessIndexCols(hesscols);
    std::vector<int> getrows = getHessIndexCols();
    printVector(getrows.data(),getrows.size());
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjQuadratic);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    for (int i=0; i<n_; i++)
    {
      setVarLoBnds(i, -KTR_INFBOUND);
      setVarUpBnds(i, KTR_INFBOUND);
      setXInitial(i, 0.0);
    }
  }

  void setConstraintProperties() {
    for (int i=0; i<m_; i++)
    {
      setConTypes(i, knitro::KTREnums::ConstraintType::ConLinear);
      setConLoBnds(0.0);
      setConUpBnds(KTR_INFBOUND);
    }
  }

  void setDerivativeProperties() {
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
  }

  void setHessianProperties(){
    printMessage("Hessian Property");
    int counter = 0;
    for (int j=0; j<n_; j++){
        for (int i=j; i<n_; i++){
          if (Hfull_[j][i])
          {
            setHessIndexRows(counter,j);
            setHessIndexCols(counter,i);;
            counter ++;
          }
        }
    }
  }

};

//#endif

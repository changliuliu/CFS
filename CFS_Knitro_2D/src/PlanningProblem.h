#ifndef _PLANNINGPROBLEM_H_
#define _PLANNINGPROBLEM_H_
#include "customizedPrint.h"
#include "d2problem.h"
#include "loadParameter.h"

// Motion Planning Problem
class PlanningProblem {
public:
	PlanningProblem(){
		std::vector<int> data;
		loadParameter(data);
		nstep_ = data[0]; //90;
		dim_ = data[1]; //2;
		nobs_ = data[2]; //1;
		neq_ = data[3]; //0;
		margin_ = 0.25;
/*		double weight_ref[] = {1,10,4};
		double weight_self[] = {0,0,2};
		double obspol[] = {3, -1, 5, -1.1, 6, 3, 2, 3};
		double star[] = {0,0};
		double endin[] = {9,0};
		weight_ref_ = weight_ref;
		weight_self_ = weight_self;
		obspoly = obspol;
		start = star;
		ending = endin;
		std::cout << "start point: (" << start[0] << "," << start[1] << ")" << std::endl;
		std::cout << "end point: (" << ending[0] << "," << ending[1] << ")" << std::endl;*/
		obspolys_.clear();
		std::vector<double> obspoly_;
		for (int k = 0; k < nobs_; k++){
			obspoly_.clear();
			for (int i = 0; i < 8; i++){
				obspoly_.push_back(obspoly[k*8 + i]);
			}
			obspolys_.push_back(obspoly_);
			//printVector(obspoly_.data(),obspoly_.size());
		}
		//printMessage("Planning Problem Initialized");
	}

	int nstep_;
	int dim_;
	int nobs_;
	int neq_;
	double margin_;
	const double weight_ref_[3]= {0,0,0};
	const double weight_self_[3]= {0,0,1};
	const double obspoly[40]= {2, 3, 4.3, 3,  4, -1, 3, -1, 
							   6, 0, 7, 1, 7.5, -3, 6, -3,
							   1, -2, 2, -3, 1.5, -0.1, 0, -1,
							   1, 1, 6, 1, 1, 2, 5, 2,
							   3, -3, 7, -3, 7, -2.1, 3, -2.1};
	/*const double obspoly[40]= {0.5, -1.3, 2.5, -1.3, 2.5, -0.3, 0.5, -0.3,
								6.5, -1.3, 8.5, -1.3, 8.5, -0.3, 6.5, -0.3,
								3.5, -0.45, 5.5, -0.45, 5.5, 0.55, 3.5, 0.55};*/

	const double start[2]= {0,0};
	const double ending[2]= {9,0};
	std::vector< std::vector<double> > obspolys_;

	int printTrajectory(const std::vector<double>& xref){
	  printMessage("Reference Trajectory:");
	  std::cout << "[ ";
	  for (int i=0; i < nstep_; i++)
	  {
	    std::cout << std::setw(5) << std::setprecision(-1) << xref[i*dim_];
	    if (i < nstep_ - 1){
      		std::cout << ",";
    	} 
	  }
	  std::cout << std::endl;
	  for (int i=0; i < nstep_; i++)
	  {
	    std::cout << std::setw(5) << std::setprecision(-1) << xref[i*dim_+1];
	    if (i < nstep_ - 1){
      		std::cout << ",";
    	} 
	  }
	  std::cout << "];" << std::endl;
	  if (xref.size() > nstep_*dim_){
	    printVector(xref.data()+nstep_*dim_, nstep_*neq_);
	  }
	  else{
	  	if (neq_) {
		    std::vector<double> v(nstep_);
		    for (int i=2; i<nstep_; i++){
		      v[i] = getYawRate(xref.data()+i*dim_, xref.data()+(i-1)*dim_, xref.data()+(i-2)*dim_);
		    }
		    printVector(v.data(), v.size());
		}
	  }
	  return 0;
	}

	int normTrajectory(std::vector<double>& xref){
		for (int i=2; i<nstep_; i++){
		    xref[nstep_*dim_+i] = getYawRate(xref.data()+i*dim_, xref.data()+(i-1)*dim_, xref.data()+(i-2)*dim_);
		}
		return 0;
	}

	int setReferenceTrajectory(std::vector<double>& xref){
	  for (int i=0; i < nstep_; i++)
	  {
	    xref[i*dim_] = ((nstep_-1-i)*start[0] + i*ending[0])/(nstep_-1);
	    xref[i*dim_+1] = ((nstep_-1-i)*start[1] + i*ending[1])/(nstep_-1);
	    for (int j=0; j<neq_; j++){
	        if (i>1){
	          xref[nstep_*dim_+i*neq_+j] = getYawRate(xref.data()+i*dim_, xref.data()+(i-1)*dim_, xref.data()+(i-2)*dim_);
	        }
	        else{
	          xref[nstep_*dim_+i*neq_+j] = 0;
	        }
	      }
	  }
	  //std::cout << "In PlanningProblem.h: " << xref.size() << std::endl;
	  //printTrajectory(xref);
	  //printMessage("Set reference trajectory.");
	  return 0;
	}

	int setCostMatrix(std::vector< std::vector<double> >& Qref, std::vector< std::vector<double> >& Qself, std::vector< std::vector<double> >& Qeq){
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
	      Qref[i][j] = weight_ref_[0] * Pos_[i][j] / nstep_ + weight_ref_[1] * Vel_[i][j] * (nstep_-1)  + weight_ref_[2] * Acc_[i][j] * pow(nstep_-1,4) / (nstep_ - 2);
	      Qself[i][j] = weight_self_[0] * Pos_[i][j] / nstep_ + weight_self_[1] * Vel_[i][j] * (nstep_-1) + weight_self_[2] * Acc_[i][j] * pow(nstep_-1,4) / (nstep_ - 2);
	    }
	  }

	  for (int i=0; i < nstep_*neq_; i++){
	    Qeq[i][i] = 10000 / (nstep_ - 2);
	  }
	  return 0;
	}

	int setObstacle(std::vector< std::vector< std::vector<double> > >& Obs){
		std::vector< std::vector<double> > obs_static;
		Obs.clear();
	  	for (int i=0; i<nstep_; i++){
		  	obs_static.clear();
		    for (int j=0; j<nobs_; j++){
		    	obs_static.push_back(obspolys_[j]);
			}
	    Obs.push_back(obs_static);
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
	  //printMessage("Cost for path");
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

	int setSparceObjective(const std::vector<double>& xref_, std::vector<double>& H_, std::vector<double>& f_, std::vector<int>& hessrows, std::vector<int>& hesscols){
		std::vector< std::vector<double> > Qref(nstep_*dim_, std::vector<double>(nstep_*dim_));
		std::vector< std::vector<double> > Qself(nstep_*dim_, std::vector<double>(nstep_*dim_));
		std::vector< std::vector<double> > Qeq(nstep_*neq_, std::vector<double>(nstep_*neq_));
		setCostMatrix(Qref, Qself, Qeq);

		f_.clear(); H_.clear(); hessrows.clear(); hesscols.clear();

	    double hij;
	    for (int i=0; i<nstep_*dim_; i++){
	      f_.push_back(0);
	      for (int j=0; j<nstep_*dim_; j++)
	      { 
	        f_[i] += -2*Qref[i][j]*xref_[j];
	        if (i<=j){
	          hij = Qref[i][j] + Qself[i][j];
	          if (hij) {
	            H_.push_back(hij);
	            hessrows.push_back(i);
	            hesscols.push_back(j);
	          }
	        }
	      }
	    }
	    for (int i=nstep_*dim_; i<nstep_*(dim_+neq_); i++){
		    for (int j=i; j<nstep_*(dim_+neq_); j++)
		    {
		      hij = Qeq[i-nstep_*dim_][j-nstep_*dim_];
		      if (hij) {
		      	H_.push_back(hij);
	            hessrows.push_back(i);
	            hesscols.push_back(j);
	        	}
		    }
		}
	  	return 0;
	}

	int setConstraintSparcity(std::vector<int>& jacrows, std::vector<int>& jaccols, int n = 1){
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
	    for (int j=0; j<n; j++){
	    for (int i=2; i<nstep_; i++){
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-2)*dim_);
	          //std::cout << nstep_*nobs_+i << "," << (i-2)*dim_ << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-2)*dim_+1);
	          //std::cout << nstep_*nobs_+i << "," << (i-2)*dim_+1 << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-1)*dim_);
	          //std::cout << nstep_*nobs_+i << "," << (i-1)*dim_ << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-1)*dim_+1);
	          //std::cout << nstep_*nobs_+i << "," << (i-1)*dim_+1 << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-0)*dim_);
	          //std::cout << nstep_*nobs_+i << "," << (i-0)*dim_ << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back((i-0)*dim_+1);
	          //std::cout << nstep_*nobs_+i << "," << (i-0)*dim_+1 << std::endl;
	          jacrows.push_back(nstep_*nobs_+(i-2)*n+j);
	          jaccols.push_back(nstep_*neq_*dim_+i);
	          //std::cout << nstep_*nobs_+i << "," << nstep_*nobs_*dim_+i << std::endl;
	    }
		}
	    //printVector(jacrows.data(),jacrows.size());
	    //printVector(jaccols.data(),jaccols.size());
	    return 0;
	}

	int getJacSparcity(){
		//printMessage("Get Jacobian Sparcity");
		std::vector<int> jacrows, jaccols;
		setConstraintSparcity(jacrows, jaccols);
		return jacrows.size();
	}

	int getHessSparcity(){
		//printMessage("Get Hessian Sparcity");
		std::vector<double> xref(nstep_*(dim_+neq_));
		setReferenceTrajectory(xref);
		std::vector< std::vector<double> > H(nstep_*(dim_+neq_), std::vector<double>(nstep_*(dim_+neq_)));
		std::vector<double> f(nstep_*(dim_+neq_));
		return setObjective(xref, H, f);
	}

	int linConstraint(const std::vector<double>& xref, std::vector<double>& L, std::vector<double>& S, int n = 1){
		// Get L_ and S_;
      L.clear(); S.clear();
      for (int i=0; i<nstep_; i++){
        for (int j=0; j<nobs_; j++){
          std::vector<double> l(dim_);
          double s;
          d2poly(obspolys_[j], xref.data()+i*dim_, l, s);

          L.push_back(l[0]);
          L.push_back(l[1]);
          S.push_back(s - margin_);
          // std::cout << l[0] << "," << l[1] << ".    " << s << "-" << margin_ << std::endl;
        }
      }
      if (!neq_) {return 0;} 
      if (n) {
        // Get L_ and S_ for equality constraints;
        for (int i=2; i<nstep_; i++){
          double xk0[] = {xref[(i-0)*dim_], xref[(i-0)*dim_+1]};
          double xk1[] = {xref[(i-1)*dim_], xref[(i-1)*dim_+1]};
          double xk2[] = {xref[(i-2)*dim_], xref[(i-2)*dim_+1]};
          double refinput = xref[nstep_*dim_+i];
          if (refinput < 0) {refinput = -refinput;}

          // ||x0-x1||^2*u + (x0-x1)^T P (x1-x2) >=0
          double ltheta = (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2));
          double lk0[] = {2*refinput*(xk0[0]-xk1[0])+(xk1[1]-xk2[1]), 2*refinput*(xk0[1]-xk1[1])-(xk1[0]-xk2[0])};
          double lk1[] = {-2*refinput*(xk0[0]-xk1[0])-(xk1[1]-xk2[1])-(xk0[1]-xk1[1]), -2*refinput*(xk0[1]-xk1[1])+(xk1[0]-xk2[0])+(xk0[0]-xk1[0])};
          double lk2[] = {(xk0[1]-xk1[1]), -(xk0[0]-xk1[0])};

          L.push_back(-lk2[0]);
          L.push_back(-lk2[1]);
          L.push_back(-lk1[0]);
          L.push_back(-lk1[1]);
          L.push_back(-lk0[0]);
          L.push_back(-lk0[1]);
          L.push_back(-ltheta);

          S.push_back((pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2))*refinput + (xk0[0]-xk1[0])*(xk1[1]-xk2[1]) - (xk0[1]-xk1[1])*(xk1[0]-xk2[0]) - ltheta*refinput - lk0[0]*xk0[0] - lk0[1]*xk0[1] - lk1[0]*xk1[0] - lk1[1]*xk1[1] - lk2[0]*xk2[0] - lk2[1]*xk2[1]); 
		}
		if (n > 1) {
		for (int i=2; i<nstep_; i++){
          double xk0[] = {xref[(i-0)*dim_], xref[(i-0)*dim_+1]};
          double xk1[] = {xref[(i-1)*dim_], xref[(i-1)*dim_+1]};
          double xk2[] = {xref[(i-2)*dim_], xref[(i-2)*dim_+1]};
          double refinput = xref[nstep_*dim_+i];
          if (refinput < 0) {refinput = -refinput;}

          // ||x0-x1||^2*u - (x0-x1)^T P (x1-x2) >=0
          double ltheta = (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2));
          double lk0[] = {2*refinput*(xk0[0]-xk1[0])-(xk1[1]-xk2[1]), 2*refinput*(xk0[1]-xk1[1])+(xk1[0]-xk2[0])};
          double lk1[] = {-2*refinput*(xk0[0]-xk1[0])+(xk1[1]-xk2[1])+(xk0[1]-xk1[1]), -2*refinput*(xk0[1]-xk1[1])-(xk1[0]-xk2[0])-(xk0[0]-xk1[0])};
          double lk2[] = {-(xk0[1]-xk1[1]), (xk0[0]-xk1[0])};

          L.push_back(-lk2[0]);
          L.push_back(-lk2[1]);
          L.push_back(-lk1[0]);
          L.push_back(-lk1[1]);
          L.push_back(-lk0[0]);
          L.push_back(-lk0[1]);
          L.push_back(-ltheta);

          S.push_back((pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2))*refinput - (xk0[0]-xk1[0])*(xk1[1]-xk2[1]) + (xk0[1]-xk1[1])*(xk1[0]-xk2[0]) - ltheta*refinput - lk0[0]*xk0[0] - lk0[1]*xk0[1] - lk1[0]*xk1[0] - lk1[1]*xk1[1] - lk2[0]*xk2[0] - lk2[1]*xk2[1]); 
          }
      	}
       }
       else {
       	for (int i=0; i<2*(nstep_-2); i++){
       		for (int j=0; j<7; j++){
       			L.push_back(0);
       		}
            S.push_back(0); 
       	}

       }
      return 0;
	}

	int linConstraint(const std::vector<double>& xref, std::vector< std::vector<double> >& Lfull, std::vector<double>& S){
      for (int i=0; i<nstep_; i++){
        for (int j=0; j<nobs_; j++){
          std::vector<double> l(dim_);
          double s;
          d2poly(obspolys_[j], xref.data()+i*dim_, l, s);

          Lfull[i*nobs_+j][i*dim_] = l[0];
          Lfull[i*nobs_+j][i*dim_ + 1] = l[1];
          S[i*nobs_+j] = s - margin_;
        }
      }
      if (neq_) {
        // Get L_ and S_ for equality constraints;
        for (int i=2; i<nstep_; i++){
          double xk0[] = {xref[(i-0)*dim_], xref[(i-0)*dim_+1]};
          double xk1[] = {xref[(i-1)*dim_], xref[(i-1)*dim_+1]};
          double xk2[] = {xref[(i-2)*dim_], xref[(i-2)*dim_+1]};
          double refinput = xref[nstep_*dim_+i];
          if (refinput < 0) {refinput = -refinput;}

          double ltheta = (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2));
          double lk0[] = {2*refinput*(xk0[0]-xk1[0])+(xk1[1]-xk2[1]), 2*refinput*(xk0[1]-xk1[1])-(xk1[0]-xk2[0])};
          double lk1[] = {-2*refinput*(xk0[0]-xk1[0])-(xk1[1]-xk2[1])-(xk0[1]-xk1[1]), -2*refinput*(xk0[1]-xk1[1])+(xk1[0]-xk2[0])+(xk0[0]-xk1[0])};
          double lk2[] = {(xk0[1]-xk1[1]), -(xk0[0]-xk1[0])};

          Lfull[nstep_*nobs_+i-2][nstep_*dim_*nobs_+i] = -ltheta;

          Lfull[nstep_*nobs_+i-2][i*dim_] = -lk0[0];
          Lfull[nstep_*nobs_+i-2][i*dim_+1] = -lk0[1];

          Lfull[nstep_*nobs_+i-2][(i-1)*dim_] = -lk1[0];
          Lfull[nstep_*nobs_+i-2][(i-1)*dim_+1] = -lk1[1];

          Lfull[nstep_*nobs_+i-2][(i-2)*dim_] = -lk2[0];
          Lfull[nstep_*nobs_+i-2][(i-2)*dim_+1] = -lk2[1];

          S[nstep_*nobs_+i-2] = (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2))*refinput + (xk0[0]-xk1[0])*(xk1[1]-xk2[1]) - (xk0[1]-xk1[1])*(xk1[0]-xk2[0]) - ltheta*refinput - lk0[0]*xk0[0] - lk0[1]*xk0[1] - lk1[0]*xk1[0] - lk1[1]*xk1[1] - lk2[0]*xk2[0] - lk2[1]*xk2[1]; 
        }
      }
      return 0;
	}

	double getCost(const std::vector<double>& x, const std::vector<double>& xref, const std::vector< std::vector<double> > H, const std::vector<double> f){
	/*	std::vector<double> xref(nstep_*(dim_+neq_));
		setReferenceTrajectory(xref);
		std::vector< std::vector<double> > H(nstep_*(dim_+neq_), std::vector<double>(nstep_*(dim_+neq_)));
		std::vector<double> f(nstep_*(dim_+neq_));
		setObjective(xref, H, f);*/
		double obj = 0;
		for (int j=0; j<xref.size(); j++){
	        for (int i=0; i<xref.size(); i++){
	          obj += x[j]*H[j][i]*x[i];
	        }
	        obj += x[j]*f[j];
	    }
	   	return obj;
	}
};
#endif
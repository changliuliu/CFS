#ifndef _D2PROBLEM_H_
#define _D2PROBLEM_H_
#include <math.h>
#include <cmath>
#include <vector>

  double getYawRate(const double* xk0, const double* xk1, const double* xk2){
    double refinput = -(xk0[0]-xk1[0])*(xk1[1]-xk2[1])+(xk0[1]-xk1[1])*(xk1[0]-xk2[0]);
    //return refinput / sqrt(pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) / sqrt(pow(xk1[0]-xk2[0],2)+pow(xk1[1]-xk2[1],2));
    return refinput / (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2));
    //double refinput = (xk0[0]-xk1[0])*(xk1[0]-xk2[0])+(xk0[1]-xk1[1])*(xk1[1]-xk2[1]);
    //return refinput / sqrt(pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) / sqrt(pow(xk1[0]-xk2[0],2)+pow(xk1[1]-xk2[1],2)) - 1;
  }

  double getYawRateError(const double* xk0, const double* xk1, const double* xk2, const double* y){
    double refinput = -(xk0[0]-xk1[0])*(xk1[1]-xk2[1])+(xk0[1]-xk1[1])*(xk1[0]-xk2[0]);
    //return refinput / sqrt(pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) / sqrt(pow(xk1[0]-xk2[0],2)+pow(xk1[1]-xk2[1],2));
    return (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) * y[0] - refinput;
    //return refinput - (pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) * y[0];
    //double refinput = (xk0[0]-xk1[0])*(xk1[0]-xk2[0])+(xk0[1]-xk1[1])*(xk1[1]-xk2[1]);
    //return refinput / sqrt(pow(xk0[0]-xk1[0],2)+pow(xk0[1]-xk1[1],2)) / sqrt(pow(xk1[0]-xk2[0],2)+pow(xk1[1]-xk2[1],2)) - 1;
  }

  double triArea(const double* p1, const double* p2, const double* p3){
    double a = sqrt(pow((p1)[0] - (p2)[0] ,2) + pow((p1)[1] - (p2)[1] ,2));
    double b = sqrt(pow((p1)[0] - (p3)[0] ,2) + pow((p1)[1] - (p3)[1] ,2));
    double c = sqrt(pow((p3)[0] - (p2)[0] ,2) + pow((p3)[1] - (p2)[1] ,2));
    double half = (a + b + c) / 2;
    return sqrt(half*(half-a)*(half-b)*(half-c));
  }

  double d2poly(const std::vector<double>& obspoly, const double* point){
    double d = KTR_INFBOUND;
    int dim_ = 2;
    int nside = obspoly.size()/dim_;
    double x1,x2,y1,y2;
    std::vector<double> trid(3);
    int ii=0;

    //std::cout << "Calling d2poly" << std::endl;
    for (int i=0; i<nside; i++){
      x1 = obspoly[i*dim_]; y1 = obspoly[i*dim_+1];
      x2 = obspoly[((i+1) % nside)*dim_]; y2 = obspoly[((i+1) % nside)*dim_ + 1];

      trid[0] = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
      trid[1] = sqrt(pow(x1-(point)[0],2) + pow(y1-(point)[1],2));
      trid[2] = sqrt(pow(x2-(point)[0],2) + pow(y2-(point)[1],2));
      //trid[1] = sqrt(pow(x1-point->vector() ,2) + pow(y1-(point+1)->vector() ,2));
      //trid[2] = sqrt(pow(x2-point->vector() ,2) + pow(y2-(point+1)->vector() ,2));

      std::vector<double> Lr(2);
      Lr[0] = y1-y2; Lr[1] = x2-x1;
      double Sr = -x1*y2+x2*y1;
      double vd = (Lr[0]*(point)[0]+Lr[1]*(point)[1]-Sr)/trid[0];
      //double vd = (Lr[0]*point->vector() + Lr[1]*(point+1)->vector()-Sr)/trid[0];
      if (vd<0) { vd = -vd;}

      if (pow(trid[1],2) > pow(trid[0],2) + pow(trid[2],2))
      {
        vd = trid[2];
      }

      if (pow(trid[2],2) > pow(trid[0],2) + pow(trid[1],2))
      {
        vd = trid[1];
      }

      if (vd < d){
        d = vd;
        ii = i;
      }
    }

    if (d == 0) {return 0;}

    double area = 0;
    double polyarea = 0;
    for (int i = 0 ; i<nside; i++)
    {
      area = area + triArea(point, obspoly.data()+i*dim_, obspoly.data()+((i+1) % nside)*dim_);
    }
    for (int i = 1 ; i<nside-1; i++)
    {
      polyarea = polyarea + triArea(obspoly.data(), obspoly.data()+i*dim_, obspoly.data()+((i+1) % nside)*dim_);
    }
    if ((polyarea-area<0.01) && (polyarea-area>-0.01))
    {
      d = -d;
    }
    return d;
  }

  double d2poly(const std::vector<double>& obspoly, const double* point, std::vector<double>& L, double& S){
    double d = KTR_INFBOUND;
    int dim_ = 2;
    int nside = obspoly.size()/dim_;
    double x1,x2,y1,y2;
    std::vector<double> trid(3);
    int ii=0;

    //std::cout << "Calling d2poly" << std::endl;
    for (int i=0; i<nside; i++){
      x1 = obspoly[i*dim_]; y1 = obspoly[i*dim_+1];
      x2 = obspoly[((i+1) % nside)*dim_]; y2 = obspoly[((i+1) % nside)*dim_ + 1];

      trid[0] = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
      trid[1] = sqrt(pow(x1-(point)[0],2) + pow(y1-(point)[1],2));
      trid[2] = sqrt(pow(x2-(point)[0],2) + pow(y2-(point)[1],2));
      //trid[1] = sqrt(pow(x1-point->vector() ,2) + pow(y1-(point+1)->vector() ,2));
      //trid[2] = sqrt(pow(x2-point->vector() ,2) + pow(y2-(point+1)->vector() ,2));

      std::vector<double> Lr(2);
      Lr[0] = y1-y2; Lr[1] = x2-x1;
      double Sr = -x1*y2+x2*y1;
      double vd = (Lr[0]*(point)[0]+Lr[1]*(point)[1]-Sr)/trid[0];
      //double vd = (Lr[0]*point->vector() + Lr[1]*(point+1)->vector()-Sr)/trid[0];
      if (vd<0) { vd = -vd;}

      if (pow(trid[1],2) > pow(trid[0],2) + pow(trid[2],2))
      {
        vd = trid[2];
        Lr[0] = (point)[0]-x2;
        Lr[1] = (point)[1]-y2;
        //Lr[0] = point->vector() - x2;
        //Lr[1] = (point+1)->vector() - y2;
        Sr = Lr[0]*x2 + Lr[1]*y2;
      }

      if (pow(trid[2],2) > pow(trid[0],2) + pow(trid[1],2))
      {
        vd = trid[1];
        Lr[0] = (point)[0]-x1;
        Lr[1] = (point)[1]-y1;
        //Lr[0] = point->vector() - x1;
        //Lr[1] = (point+1)->vector() - y1;
        Sr = Lr[0]*x1 + Lr[1]*y1;
      }

      if (vd < d){
        d = vd;
        L = Lr;
        S = Sr;
        ii = i;
      }
    }

    double normL = sqrt(pow(L[0],2) + pow(L[1],2));
    L[0] = L[0]/normL; L[1] = L[1]/normL;
    S = S/normL;

    if (L[0]*obspoly[((ii+2) % nside)*dim_]+L[1]*obspoly[((ii+2) % nside)*dim_ + 1] < S)
    {
      L[0] = -L[0]; L[1] = -L[1];
      S = -S;
    }

    if (d == 0) {return 0;}

    double area = 0;
    double polyarea = 0;
    for (int i = 0 ; i<nside; i++)
    {
      area = area + triArea(point, obspoly.data()+i*dim_, obspoly.data()+((i+1) % nside)*dim_);
    }
    for (int i = 1 ; i<nside-1; i++)
    {
      polyarea = polyarea + triArea(obspoly.data(), obspoly.data()+i*dim_, obspoly.data()+((i+1) % nside)*dim_);
    }
    if ((polyarea-area<0.01) && (polyarea-area>-0.01))
    {
      d = -d;
    }
    return d;
  }
#endif
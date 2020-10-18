#include "CUSUM.h"

#include <iostream>
#include <algorithm>

CUSUM::CUSUM(double mu0, double mu1, double sigma, double h){
  //this->mu0 = mu0;
  //this->mu1 = mu1;
  //this->sigma = sigma;
  k1 = (mu1-mu0)/(sigma*sigma);
  k2 = 0.5*(mu1+mu0);

  this->h = h;

  gplus = 0.0;
  gminus = 0.0;
}

void CUSUM::zero(){
  gplus = 0.0;
  gminus = 0.0;
}

void CUSUM::setParams(double mu0, double mu1, double sigma, double h){
  k1 = (mu1-mu0)/(sigma*sigma);
  k2 = 0.5*(mu1+mu0);

  this->h = h;

  //std::cout << "New mu1: " << mu1 << std::endl;
}

void CUSUM::calc(double z){
  gplus = std::max(0.0, gplus + k1*(z-k2));
  gminus = std::max(0.0, gminus + k1*(-z-k2));
}

bool CUSUM::res(){
  return (gplus > h) || (gminus > h);
}

double CUSUM::valueplus(){
  return gplus;
}

double CUSUM::valueminus(){
  return gminus;
}
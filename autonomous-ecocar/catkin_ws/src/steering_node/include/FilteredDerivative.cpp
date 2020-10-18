#include "FilteredDerivative.h"


FilteredDerivative::FilteredDerivative(double wc, double Ts){
  k = 2.0*wc/(2.0+wc*Ts);
  F = (2.0-wc*Ts)/(2.0+wc*Ts);

  y = 0;
  ulast = 0;
}

void FilteredDerivative::init(double u){
  y = 0;
  ulast = u;
}

void FilteredDerivative::update(double u){
  y = F*y + k*(u-ulast);

  ulast = u;
}

double FilteredDerivative::value(){
  return y;
}

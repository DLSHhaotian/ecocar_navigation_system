/*
Discrete filter with the transfer function G = wc*s/(s+wc) (derivative + low pass)
Discretized with tustin approximation.

Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/

#ifndef FILTERED_DERIVATIVE_H
#define FILTERED_DERIVATIVE_H

class FilteredDerivative{
  public:
    FilteredDerivative(double wc, double Ts);
    void init(double u);
    void update(double u);
    double value();


  private:
    double k;
    double F;

    double y;
    double ulast;
};

#endif

/*
Cumulative sum (CUSUM) detector algorithm implementation (two-sided).

Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/

#ifndef CUSUM_H
#define CUSUM_H

class CUSUM{
  public:
    CUSUM(double mu0, double mu1, double sigma, double h);
    void calc(double z);
    bool res();
    void zero();
    void setParams(double mu0, double mu1, double sigma, double h);
    double valueplus();
    double valueminus();


  private:
    //double mu0, mu1, sigma;
    double k1, k2;
    double h;

    double gplus;
    double gminus;
};

#endif

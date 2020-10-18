#ifndef DRAG_H
#define DRAG_H

#include <cmath>

class Drag{
  public:
    Drag();
    double getA(double phi);
    double getCd(double phi);
    
  private:
    unsigned int calcI(double phi);
    double A[360];
    double Cd[360];
};

#endif

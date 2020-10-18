/*
  Low pass filter implementation (Tustin discretiztion)
  Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/
#ifndef LOWPASS_H
#define LOWPASS_H

template <typename T>
class LowPass{
  public:
    LowPass(double tau, double Ts);
    void update(T u);
    T value();
    void set(T val);

  private:
    T y;
    T prevu;
    double F, G;
};

#endif

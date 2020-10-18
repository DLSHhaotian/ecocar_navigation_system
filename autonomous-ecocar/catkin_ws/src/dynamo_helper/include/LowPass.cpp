#include "LowPass.h"
#include "math.h"

template <typename T>
LowPass<T>::LowPass(double tau, double Ts){
  F = (2*tau-Ts)/(2*tau+Ts);
  G = Ts/(2*tau+Ts);

  prevu = 0;
}

template <typename T>
void LowPass<T>::update(T u){
  y = F*y + G*(u+prevu);

  prevu = u;
}

template <typename T>
T LowPass<T>::value(){
  return y;
}

template <typename T>
void LowPass<T>::set(T val){
  y = val;
  prevu = val;
}

// Tell the compiler to explicitly compile
template class LowPass<float>;
template class LowPass<double>;

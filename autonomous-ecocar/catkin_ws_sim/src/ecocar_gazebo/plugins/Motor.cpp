/*
  Simple Dynamo motor simulation
  Lookup tables, gearing and up/down gear shift values are all derived from MATLAB simulations by Rune Hermansen (bachelor thesis)
  Check with someone if those are still the values in use (especially RPM up/down)
*/

#include "Motor.h"
#include <ignition/math.hh>

#define ETA_MECH 0.98
#define I_CRANK 965.94E-6
#define I_CLUTCH 510.95E-6
#define I_FLYWHEEL 551.83E-6
#define I_GEARIN 672.702E-6

#define I_GEARMID 952.345E-6

#define I_GEAROUT 2724.85E-6

Motor::Motor(double I)
: RPM_table{2000, 2500, 3000, 3500, 4000, 4500, 5000},
  torque_table{3.40500, 3.49798, 3.67421, 3.82250, 3.89206, 3.86583, 3.72919}
{
  I_wheel = I;

  RPM_up = 3135.7;
  RPM_down = 2717.6;

  u1 = 1.0/(18.0/109.0*19.0/68.0);
  u2 = 1.0/(31.0/96.0*19.0/68.0);
  u_sectrans = 68.0/19.0;

  I_motor = I_CRANK + I_CLUTCH + I_FLYWHEEL + I_GEARIN;
  I_wheelside = I_GEAROUT + I_wheel;

  omega_shiftup = 2*IGN_PI*RPM_up/60.0 / u1;
  omega_shiftdown = 2*IGN_PI*RPM_down/60.0 /u2;

  gear = u1;
}

double Motor::getTorque(double omega){
  // Wheel torque from angular wheel velocity
  //std::cout << omega << std::endl;
  // Shift gear up or down
  if(omega <= omega_shiftdown){
    gear = u1;
  }
  if(omega >= omega_shiftup){
    gear = u2;
  }

  // Motor crankshaft angular velocity (rad/s)
  double omega_crank = omega*gear;

  // Crankshaft RPM
  double RPM = omega_crank/(2*IGN_PI)*60;
  //std::cout << "Crankshaft RPM: " << RPM << std::endl;

  // Motor torque
  double torque_crank = this->crankTorque(RPM);
  //std::cout << "Crankshaft torque: " << torque_crank << std::endl;

  // Wheel torque
  // Effective torque on each wheel
  double torque_wheel = I_wheel/gear/(I_motor+I_GEARMID/(u_sectrans*u_sectrans) + I_wheelside/(gear*gear)) * ETA_MECH * torque_crank;

  return torque_wheel;
}


double Motor::crankTorque(double RPM){
  // Motor torque from RPM
  // Lookup table with linear interpolation

  // Put within table limits
  if(RPM < 2000){
    RPM = 2000;
  } else if(RPM > 4999){
    RPM = 4999;
  }

  unsigned int ind = 0;
  for(unsigned int i = 0; i<7; i++){
    if(RPM_table[i] > RPM){
      ind = (i==0) ? i : i-1;
      break;
    }
  }

  double tau = torque_table[ind+1] - (torque_table[ind+1]-torque_table[ind])/(RPM_table[ind+1]-RPM_table[ind]) * (RPM_table[ind+1]-RPM);
  return tau;
}

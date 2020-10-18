#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  public:
    Motor(double I);
    double getTorque(double omega);

  private:
    double I_wheel;
    double crankTorque(double RPM);
    double RPM_up;
    double RPM_down;
    double omega_shiftup;
    double omega_shiftdown;
    double RPM_table[7];
    double torque_table[7];
    double u1;
    double u2;
    double u_sectrans;

    double I_motor, I_wheelside;

    double gear;

};

#endif

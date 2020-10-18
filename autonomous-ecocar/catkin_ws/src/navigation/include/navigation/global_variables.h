// global_variables.h
#define PI 3.14159265358979

// Car dimensions
double car_length = 1.5;
double wheel_radius = 0.0254*22;
double full_turning_range = 12.2;   // degrees between full left and full right
double dist_between_wheels= 0.8;

// Odometry parameters
int amount_of_ticks = 60;
int odo_mounting = 1; // 0 for the left wheel and 1 for the right wheel
double odo_lag = 0.0;
double odo_drift = 0.0; //odo drift factor [standard deviations / m]

// Gyroscope parameters
double gyro_lag = 0.0;
double gyro_drift=0.0;
double gyro_offset=0.0;
int gyro_rate = 100; //gyro update rate in Hz

//GPS parameters
double GPS_fixed_1sigma = 0.1; //1-sigma error in [m] in fixed mode
double GPS_float_1sigma = 0.5; //1-sigma error in [m] in float mode
double GPS_noRTK_1sigma = 2.5; //1-sigma error in [m] without RTK
double GPS_rate = 10; //GPS update rate in [Hz]

//Initial car position
double pose_init[3] = {0,0,0}; // x-coordinate, y-coordinate, z-coordinate [m]

double ts = 0.001; // simulation step in [s]

//for a simple version

float speed = 2;
float angle = 30 * PI / 180;


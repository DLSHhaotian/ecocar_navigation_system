#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
// Message types
#include "actionlib_msgs/GoalID.h"
#include "auto_navi/commandMsg.h"
#include "auto_navi/driveMsg.h"
#include "auto_navi/emergencyMsg.h"
#include "auto_navi/flagMsg.h"
#include "auto_navi/motorMsg.h"
#include "dynamo_msgs/BrakeStepper.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "dynamo_msgs/TeensyRead.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "geometry_msgs/PolygonStamped.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include "lidar_package/point.h"
#include "voronoi.hpp"
#include <algorithm>
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;
// using namespace std;	// STRICTLY FORBIDDEN

int for_the_prince = 1;
int slow_fast[5] = {0};
int fast = 0;
// double sonar = 5.0;
double speed;

int parking = 0; // checks if parking spot is detected!
geometry_msgs::Point32 p_front;
geometry_msgs::Point32 p_center;
ros::Time t1; // undøvendig?
ros::Duration t2;
ros::Publisher voroPub;
ros::Publisher marker_pub;
ros::Publisher flag_pub;
ros::Publisher brake_pub;
ros::Publisher steerPub;
ros::Publisher teensyPub;
ros::Publisher emergency_publisher;
ros::Publisher motoPub;
double BW2L = 0.75; // Distance from backwheel to lidar. ADJUST TO CORRECT VALUE
                    // AFTER MEASUREMENT
double BW2FW = 1.516; // Distance from backwheels to front wheels. ADJUST TO
                      // CORRECT VALUE AFTER MEASUREMENT
double DIC = 0.0; // How far ahead should the algorithm look to assign walls as
                  // left or right? experimental
double dist_max = 0;
int flag, ct;
int max_brake_power;
double td, drivendist, theta, tempdist, current_td;
int state, current_flag, current_ct, brakestop;
int master_switch, timetemp, comcounter;
bool brakeEngage;
double brakePower;
double odox;     // odometry x in world coordinates
double odoy;     // odometry y in world coordinates
double odotheta; // odometry orientation
double avg_speed;
int emergency_count;

double line_x;
double line_y;
double line_th;
int bad = 0;

struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct edge {
  // Start vertice
  double x0;
  double y0;
  // distance from zero
  double dist;
  // End vertice
  double x1;
  double y1;
};

struct x_y_th {
  double x;
  double y;
  double th;
};

struct Pos {
  double x;
  double y;
};

struct vertex {
  Pos v0;
  std::vector<Pos> v_e;
  std::vector<int> v_e_ind;
  int edges;
};

struct Bank {
  double x;
  double y;
  double x_sum;
  double y_sum;
  int count;
  int tick;
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

template <> struct geometry_concept<Point> { typedef point_concept type; };

template <> struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(const Point &point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <> struct geometry_concept<Segment> { typedef segment_concept type; };

template <> struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment &segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};
} // polygon
} // boost

std::vector<Pos> posesGlobal;
std::vector<Pos> posesGlobalGlobal;
std::vector<Bank> global_points;
std::vector<double> xvec_global;
std::vector<double> yvec_global;
std::vector<Pos> poses_debug;
//DLSH add
std::vector<double> x_allMsg;
std::vector<double> y_allMsg;
//DLSH

std::vector<double> xvec_global2;
std::vector<double> yvec_global2;

std::vector<edge> iterate_primary_edges(const voronoi_diagram<double> &vd,
                                        std::vector<int> id, int factor) {
  std::vector<edge> edges;
  double dist_temp = 99999;
  int index;

  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    // This is convenient way to iterate edges around Voronoi vertex.

    if (it->vertex0() && it->vertex1() &&
        id[it->cell()->source_index()] !=
            id[it->twin()->cell()->source_index()]) {
      edge edge;
      edge.x0 = it->vertex0()->x() / factor;
      edge.y0 = it->vertex0()->y() / factor;
      edge.dist = sqrt(pow(edge.x0 - 0, 2) + pow(edge.y0, 2));
      edge.x1 = it->vertex1()->x() / factor;
      edge.y1 = it->vertex1()->y() / factor;

      if (edge.dist < dist_temp) {
        dist_temp = edge.dist;
        index = edges.size();
      }
      edges.push_back(edge);
    }
  }
  if (edges.size() != 0) {
    iter_swap(edges.begin(), edges.begin() + index);
  }
  return edges;
}

std::vector<vertex> sort_route(std::vector<edge> edges) {
  std::vector<vertex> vertices;
  Pos tempPos;
  std::vector<int> next_i;
  next_i.push_back(0);
  bool skip[10000] = {false};
  // kig alle edges igennem
  // Fjern duplicates - indlæs som ekstra destinationer
  // hvilken edge er først
  int counter = 0, i = 0, k = 0, found = 0;
  while (next_i.size() != 0) {
    vertex vertice;
    i = next_i[0];
    next_i.erase(next_i.begin());
    vertice.edges = 1;
    vertice.v0.x = edges[i].x0;
    vertice.v0.y = edges[i].y0;
    tempPos.x = edges[i].x1;
    tempPos.y = edges[i].y1;
    vertice.v_e.push_back(tempPos);
    vertices.push_back(vertice);
    k = -1;

    while (k != edges.size() - 1) {
      k++;
      if (skip[k] || k == i ||
          (edges[k].x1 == edges[i].x0 &&
           edges[k].y1 == edges[i].y0)) { // skip twin edges
        skip[k] = true;
        continue;
      } else if (edges[k].x0 == edges[i].x0 &&
                 edges[k].y0 ==
                     edges[i]
                         .y0) { // all edges assigned to respective vertices.
        vertices[counter].edges++;
        tempPos.x = edges[k].x1;
        tempPos.y = edges[k].y1;
        vertices[counter].v_e.push_back(tempPos);
        skip[k] = true;
      }
    }
    for (k = 0; k < edges.size(); k++) {
      if (skip[k])
        continue;
      for (int j = found; j != vertices[counter].edges; j++) {
        if (vertices[counter].v_e[j].y == edges[k].y0 &&
            vertices[counter].v_e[j].x ==
                edges[k].x0) { // find next point to look at
          if (i == 0 && edges[k].x0 < vertices[0].v0.x)
            continue;
          vertices[counter].v_e_ind.push_back(next_i.size() + vertices.size());
          next_i.push_back(k);
          found++;
        }
      }
      if (found == vertices[counter].edges)
        break;
    }
    vertices[counter].edges = found;
    found = 0;
    counter++;
  }
  return vertices;
}

std::vector<vertex> voronoi_run(std::vector<double> laserx,
                                std::vector<double> lasery,
                                std::vector<int> id) {
  int factor = 100; // voronoi takes integers. Multiply laser measurements to
                    // get 10 cm grid  accuracy

  // Voronoi Input Geometries.
  std::vector<Point> points;
  vertex vertice;
  Pos tempPos;
  vertice.edges = 0;
  vertice.v0.x = 0;
  vertice.v0.y = 0;
  tempPos.x = 0;
  tempPos.y = 0;
  vertice.v_e.push_back(tempPos);
  std::vector<vertex> drive_points;

  for (int i = 0; i != laserx.size(); i++) { // load all points for Voronoi
    points.push_back(Point(int(laserx[i] * factor), int(lasery[i] * factor)));
  }

  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(), &vd);

  std::vector<edge> edges = iterate_primary_edges(vd, id, factor);
  if (edges.size() != 0) {
    drive_points = sort_route(edges);
  } else {

    drive_points.push_back(vertice);
  }

  return drive_points;
}

x_y_th linreg(std::vector<Pos> x_y, int swing_indicator) {
  double sumx = 0.0;
  double sumy = 0.0;
  double sumxy = 0.0;
  double sumxx = 0.0;
  double SUMres = 0.0;
  double theta = 0.0;
  double avgx = 0.0;
  double avgy = 0.0;
  double slope = 0.0;
  double res = 0.0;         // residue squared
  double y_intercept = 0.0; // y intercept of regression line
  double SUM_Yres = 0.0;    // sum of squared of the discrepancies
  double Yres = 0.0;        // squared of the discrepancies
  double Rsqr = 0.0;        // coefficient of determination
  double SUM_reg = 0.0;     // sum of squared of the discrepancies
  double reg = 0.0;

  x_y_th x_y_th_result;
  x_y_th_result.x = 0.0;
  x_y_th_result.y = y_intercept;
  x_y_th_result.th = theta;
  if (x_y.size() < 2)
    return x_y_th_result;
  for (int i = 0; i < x_y.size(); i++) {
    sumx += x_y[i].x;
    sumy += x_y[i].y;
    sumxy += x_y[i].x * x_y[i].y;
    sumxx += pow(x_y[i].x, 2);
  }
  avgx = sumx / x_y.size();
  avgy = sumy / x_y.size();
  slope =
      (x_y.size() * sumxy - sumx * sumy) / (x_y.size() * sumxx - sumx * sumx);

  if (x_y[0].x <= x_y[1].x)
    theta = atan2(slope, 1);
  if (x_y[0].x > x_y[x_y.size() - 1].x)
    theta = atan2(-slope, -1);

  y_intercept = avgy - slope * avgx;
  x_y_th_result.x = 0.0;
  x_y_th_result.y = y_intercept;
  x_y_th_result.th = theta;

  // RESIDUALS
  // calculate squared residues, their sum etc.
  for (int i = 0; i < x_y.size(); i++) {
    // current (y_i - a0 - a1 * x_i)^2
    Yres = pow((x_y[i].y - (y_intercept + slope * x_y[i].x)), 2);

    // sum of (y_i - a0 - a1 * x_i)^2
    SUM_Yres += Yres;

    // SUM_REG
    reg = pow((y_intercept + slope * x_y[i].x) - avgy, 2);
    SUM_reg += reg;
    // current residue squared (y_i - AVGy)^2
    res = pow(x_y[i].y - avgy, 2);

    // sum of squared residues
    SUMres += res;
  }

  // calculate r^2 coefficient of determination
  //   Rsqr = 1-( SUM_Yres) / SUMres;
  int diag = 0;
  Rsqr = SUM_reg / SUMres;
  /* if(!swing_indicator){
           if(Rsqr <= 0.8 && fabs(slope) > 0.1 || fabs(y_intercept) > 6 )
  {	bad=1;}
  else
  {	bad=0;}
   }*/

  if (swing_indicator) {
    int N = sizeof(slow_fast) / sizeof(slow_fast[0]);
    if (N > 1) {
      for (int i = N - 1; i > 0; i--) {
        slow_fast[i] = slow_fast[i - 1];
      }
    }

    if (((Rsqr >= 0.97 && fabs(slope) < 0.5) || fabs(slope) < 0.1) &&
        fabs(y_intercept) < 1 && !bad) {
      diag = 1, slow_fast[0] = 1;
    } else if ((Rsqr < 0.95 || fabs(slope) > 0.5) && !bad) {
      diag = 2, slow_fast[0] = 0;
    }
    printf("\nSLOPE: %f, RSQR: %f, Bad: %d, Intercept: %f, Diag: %d\n", slope,
           Rsqr, bad, y_intercept, diag);

    double sum = 0;
    for (int i = 0; i < N; i++) {
      sum += slow_fast[i] / 5.0;
    }
    if (sum >= 0.8)
      fast = 1;
    if (sum < 0.8)
      fast = 0;
  }
  return x_y_th_result;
}

std::vector<double> quadreg(std::vector<double> x, std::vector<double> y,
                            int n) {
  int i; // loop iterator
  int j; // loop iterator
  int k; // loop

  //    int n=3; //Degree of polynomial
  double theta = 0.0;
  //   x.push_back(0);
  //   y.push_back(0);
  int N = x.size(); // Datasize
  double y_intercept = 0.0;
  double slope = 0.0;
  x_y_th x_y_th_result;
  double X[2 * n + 1]; // Array that will store the values of
                       // sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)

  for (i = 0; i < 2 * n + 1; i++) {
    X[i] = 0;
    for (j = 0; j < N; j++) {
      X[i] =
          X[i] + pow(x[j], i); // consecutive positions of the array will store
      // N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
  }

  double B[n + 1][n + 2], a[n + 1]; // B is the Normal matrix(augmented) that
                                    // will store the equations, 'a' is for
                                    // value of the final coefficients

  for (i = 0; i <= n; i++)
    for (j = 0; j <= n; j++) {
      B[i][j] = X[i + j]; // Build the Normal matrix by storing the
                          // corresponding coefficients at the right positions
                          // except the last column of the matrix
    }

  double Y[n + 1]; // Array to store the values of
                   // sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)

  for (i = 0; i < n + 1; i++) {
    Y[i] = 0;
    for (j = 0; j < N; j++) {
      Y[i] = Y[i] + pow(x[j], i) * y[j]; // consecutive positions will store
      // sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
  }
  for (i = 0; i <= n; i++) {
    B[i][n + 1] = Y[i];
  } // load the values of Y as the last column of B(Normal Matrix but augmented)

  n = n + 1; // n is made n+1 because the Gaussian Elimination part below was
             // for n equations, but here n is the degree of polynomial and for
             // n degree we get n+1 equations

  for (i = 0; i < n;
       i++) { // From now Gaussian Elimination starts(can be ignored) to solve
              // the set of linear equations (Pivotisation)
    for (k = i + 1; k < n; k++) {
      if (B[i][i] < B[k][i]) {
        for (j = 0; j <= n; j++) {
          double temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }
      }
    }
  }
  for (i = 0; i < n - 1; i++) { // loop to perform the gauss elimination
    for (k = i + 1; k < n; k++) {
      double t = B[k][i] / B[i][i];
      for (j = 0; j <= n; j++) {
        B[k][j] = B[k][j] - t * B[i][j]; // make the elements below the pivot
                                         // elements equal to zero or elimnate
                                         // the variables
      }
    }
  }
  for (i = n - 1; i >= 0; i--) { // back-substitution
    // x is an array whose values correspond to the values of x,y,z..
    a[i] = B[i][n]; // make the variable to be calculated equal to the rhs of
                    // the last equation
    for (j = 0; j < n; j++) {
      if (j != i) { // then subtract all the lhs values except the coefficient
                    // of the variable whose value is being calculated
        a[i] = a[i] - B[i][j] * a[j];
      }
    }
    a[i] = a[i] / B[i][i]; // now finally divide the rhs by the coefficient of
                           // the variable to be calculated
  }
  std::vector<double> factors;

  for (i = 0; i < n; i++) {
    factors.push_back(a[i]); // A[0]*x⁰ , A[1]*x¹ , a[2]*x² etc
  }

  return factors;
}

std::vector<Pos> smooth(std::vector<double> xvec, std::vector<double> yvec,
                        double precision, int globalBOOL) {
  std::vector<Pos> pose_return;
  if (xvec.size() == 0)
    return pose_return;
  double dist = sqrt(pow(xvec[0], 2) + pow(yvec[0], 2));
  double dist_thresh = 3.0;
  double tempx, tempy;
  Pos one_pose;
  std::vector<double> dists;
  dists.push_back(0.0);
  double dist_old = sqrt(pow(xvec[0], 2) + pow(yvec[0], 2));
  for (int i = 1; i < xvec.size(); i++) {
    dist += sqrt(pow(xvec[i] - xvec[i - 1], 2) + pow(yvec[i] - yvec[i - 1], 2));
    if (dist - dist_old > dist_thresh || dist > dist_max)
      break;
    dist_old = dist;
    dists.push_back(dist);
  }

  std::vector<double> factorsX = quadreg(dists, xvec, 5);
  std::vector<double> factorsY = quadreg(dists, yvec, 5);

  for (double i = 0; i < dist_old; i += precision) {
    one_pose.x = 0;
    one_pose.y = 0;

    for (int j = 0; j < factorsX.size(); j++) {
      one_pose.x += factorsX[j] * pow(i, j);
      one_pose.y += factorsY[j] * pow(i, j);
    }
    if (globalBOOL == 1) {
      tempx = cos(odotheta) * one_pose.x - sin(odotheta) * one_pose.y + odox;
      tempy = sin(odotheta) * one_pose.x + cos(odotheta) * one_pose.y + odoy;
      one_pose.x = tempx;
      one_pose.y = tempy;
    }
    pose_return.push_back(one_pose);
  }

  return pose_return;
}

void point_sender(double precision) {
  auto_navi::driveMsg messy;
  auto_navi::motorMsg max_motor;
  x_y_th line;
  double localx, localy, pointdist, temp = 99.0;
  int index = 0;

  // Initilize drive point message
  messy.header.stamp = ros::Time::now();
  messy.header.frame_id = "voronoi_node";

  if (posesGlobal.size() == 0)
    return;
  for (int i = 0; i < posesGlobal.size(); i++) {

    //		localx =
    //(cos(-odotheta)*(posesGlobal[i].x-odox))-(sin(-odotheta)*(posesGlobal[i].y-odoy));
    //// matrix transformation from world coordinates to local coordinates
    //		localy =
    //(sin(-odotheta)*(posesGlobal[i].x-odox))+(cos(-odotheta)*(posesGlobal[i].y-odoy));

    pointdist = sqrt(pow(posesGlobal[i].x - 2.5, 2) + pow(posesGlobal[i].y, 2));
    if (pointdist < temp) {
      temp = pointdist;
      index = i;
    }
  }
  // printf("\nindex: %d\n",index);
  double x, y, th;

  std::vector<Pos> poses;

  std::vector<Pos> poses2;

  int amount_points = 4.0 / precision; // driving line
  int amount_points2 =
      (dist_max * 0.9) / precision - index; // swing recognize line

  if (index + amount_points2 < posesGlobal.size() && amount_points2 >= 2) {

    poses2.insert(poses2.begin(), posesGlobal.begin() + index,
                  posesGlobal.begin() + index + amount_points2);
    linreg(poses2, 1);

    printf("\nmeters ahead: %f", poses2.size() * precision);
  }

  if (index + amount_points < posesGlobal.size()) {
    poses.insert(poses.begin(), posesGlobal.begin() + index,
                 posesGlobal.begin() + index + amount_points);
    poses_debug = poses;

    line = linreg(poses, 0);
    x = line.x;
    y = line.y;
    th = line.th;
    for (int i = 0; i < poses.size(); i++) {
      // printf("poses: x: %f y: %f\n",poses[i].x,poses[i].y);
    }
    // th =
    // atan2(posesGlobal[index+5].y-posesGlobal[index].y,posesGlobal[index+5].x-posesGlobal[index].x);

  } else {
    x = posesGlobal[index].x;
    y = posesGlobal[index].y;
    th = atan2(posesGlobal[index].y, posesGlobal[index].x);
  }
  // printf("\nPoint found at x: %f y: %f th: %f.\n",x,y,th);

  // calculate points to global coordinates
  double globalx = cos(odotheta) * x - sin(odotheta) * y + odox;
  double globaly = sin(odotheta) * x + cos(odotheta) * y + odoy;

  th = odotheta + th;

  double angleerr, d, refthl, refxl, refyl;
  double vnx, vny, c;

  refxl = globalx;
  refyl = globaly;
  refthl = th;

  vnx = -sin(refthl); // normal vector
  vny = cos(refthl);
  c = -(vnx * (refxl) + vny * (refyl)); // c = b*y + a*x

  // controller
  angleerr = refthl - odotheta;      // error between current th and target th
  d = (odox * vnx + odoy * vny + c); // steering dist error
  /*if(fabs(angleerr) > M_PI/2 || fabs(d)>6.0) {bad =1}
          else{bad=1;}*/
  if (bad == 1)
    return;
  line_x = globalx;
  line_y = globaly;
  line_th = th;

  // publish (send to drive)
  messy.x = globalx;
  messy.y = globaly;
  messy.theta = th;
  max_motor.fast = fast;
  voroPub.publish(messy);
  motoPub.publish(max_motor);
}

std::vector<vertex> point_handler(std::vector<vertex> drivepoints) {
  double globalx = 0.0;
  double globaly = 0.0;
  std::vector<vertex> drivepoints2;
  // choose right, left?
  int right_left = 0, index = 0, most_right = 0, most_left = 0;
  double x, y, th, x1, y1, angle, angleL = -1000.0, angleR = 1000.0;
  std::vector<double> xvec;
  std::vector<double> yvec;
  std::vector<double> xvec2;
  std::vector<double> yvec2;
  double dist = dist_max; // 20
  double accum_dist = 0;
  int k = 0;
  double x0, y0, localx, localy;

  std::vector<Bank> newbank;
  std::vector<Bank> tempbank;
  std::vector<Bank> drivebank;
  std::vector<int> sort_indicator;
  std::vector<int> global_points_indicator;
  std::vector<int> drivepoints_indicator;
  std::vector<Pos> drivepoints_global;
  double pointdist = 0.0;
  int indicator;

  x = drivepoints[0].v0.x;
  y = drivepoints[0].v0.y;

  while (dist > accum_dist) {
    angleL = -1000.0, angleR = 1000.0;

    // Get drive points
    xvec.push_back(drivepoints[index].v0.x); // TEST
    yvec.push_back(drivepoints[index].v0.y);
    drivepoints2.push_back(drivepoints[index]);
    if (k >= drivepoints.size() || drivepoints[index].edges == 0)
      break;

    for (int j = 0; j < drivepoints[index].edges; j++) {

      x1 = drivepoints[index].v_e[j].x;
      y1 = drivepoints[index].v_e[j].y;
      x0 = drivepoints[index].v0.x;
      y0 = drivepoints[index].v0.y;
      accum_dist += sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
      angle = atan2(y1 - y0, x1 - x0);

      if (angle >
          angleL) { // if angleL is less than measured angle, put as most left
        angleL = angle;
        most_left = j;
      }
      if (angle <
          angleR) { // if angleR is more than measured angle, put as most right
        angleR = angle;
        most_right = j;
      }
    }
    if (drivepoints[index].edges > 1 && right_left == 1) { // go left

      index = drivepoints[index].v_e_ind[most_left];

    } else {

      index = drivepoints[index].v_e_ind[most_right];
    }
    k++;
  }

  std::vector<int> skip(xvec.size(), 0);

  for (int i = 0; i < xvec.size() - 1; i++) {
    for (int j = i + 1; j < xvec.size(); j++) {
      if (skip[j] == 1)
        continue;
      double pointdist =
          sqrt(pow(xvec[j] - xvec[i], 2) + pow(yvec[j] - yvec[i], 2));
      // if(pointdist<0.2){
      // skip[j]=1;
      //}
    }
  }

  // # - # - # - # - # - # -  compare local drive points to global drive point
  // memory  - # - # - # - # - # - # - # - # - # - # - # - # - # - # - # - # - #
  // - #

  //	std::cout<<"\nxvec size: " <<xvec.size();
  // first transform drivepoints from local to global coordinates
  for (int i = 0; i < xvec.size(); i++) {
    if (skip[i] == 1)
      continue;
    Pos possen;
    x = xvec[i];
    y = yvec[i];
    globalx = cos(odotheta) * x - sin(odotheta) * y + odox;
    globaly = sin(odotheta) * x + cos(odotheta) * y + odoy;
    possen.x = globalx;
    possen.y = globaly;
    drivepoints_global.push_back(possen);
    drivepoints_indicator.push_back(0);
    //	std::cout<<"\nfirst loop i: "<<i << "  xvec: " << xvec[i] << "  yvec: "
    //<< yvec[i];
  }

  //	std::cout<<"\nglobal_points size:  " << global_points.size();

  if (global_points.size() == 0) { // if global_points is empty: let it be the
                                   // current drivepoints vector
    std::cout << "\ninside first if\n";
    for (int i = 0; i < drivepoints_global.size(); i++) {
      Bank tempbank;
      tempbank.x = drivepoints_global[i].x;
      tempbank.y = drivepoints_global[i].y;
      tempbank.tick = 5; // maybe
      tempbank.count = 1;
      tempbank.x_sum = drivepoints_global[i].x;
      tempbank.y_sum = drivepoints_global[i].y;

      global_points.push_back(tempbank);
      // use this opportunity to initialize global_points_indicator
      global_points_indicator.push_back(0);
    }
  } else { // if global_points is not empty: do a summed average
    //	std::cout<<"\nelse";

    for (int i = 0; i < global_points.size(); i++) {
      global_points[i].tick--; // first we let all elements' tick be decreased
                               // with 1. Later they can be =5 if they're
                               // matched with drivepoints.
      //	std::cout<<"\nglobal_points i: " << i << "  x: " <<
      // global_points[i].x<< "   y: " << global_points[i].y << "  tick: " <<
      // global_points[i].tick;
      // use this opportunity to initialize global_points_indicator
      global_points_indicator.push_back(0);
    }

    for (int i = 0; i < drivepoints_global.size(); i++) {
      pointdist = 900.0;
      for (int j = 0; j < global_points.size(); j++) {
        if (drivepoints_indicator[i] == 1)
          break;
        pointdist = sqrt(pow(drivepoints_global[i].x - global_points[j].x, 2) +
                         pow(drivepoints_global[i].y - global_points[j].y, 2));
        if (pointdist < 0.2) {
          global_points_indicator[j] = 1;
          drivepoints_indicator[i] = 1;
          global_points[j].tick = 5;

          // moving sum
          global_points[j].x_sum =
              global_points[j].x_sum + drivepoints_global[i].x;
          global_points[j].y_sum =
              global_points[j].y_sum + drivepoints_global[i].y;
          global_points[j].count++;
          global_points[j].x = global_points[j].x_sum / global_points[j].count;
          global_points[j].y = global_points[j].y_sum / global_points[j].count;

          // push_back this point to temp vector
          tempbank.push_back(global_points[j]);
        }
      }
    }

    // std::cout<<"\nglobal_points size: " << global_points.size();

    // check if a global point has not been summed into a drive_point. In that
    // case it still needs to be added to newbank, although at the end.
    for (int j = 0; j < global_points.size(); j++) {
      if (global_points_indicator[j] == 0) {
        tempbank.push_back(global_points[j]);
      }
    }
    for (int j = 0; j < drivepoints_global.size(); j++) {
      if (drivepoints_indicator[j] == 0) {
        Bank baba;
        baba.x = drivepoints_global[j].x;
        baba.y = drivepoints_global[j].y;
        baba.tick = 5;
        baba.x_sum = baba.x;
        baba.y_sum = baba.y;
        baba.count = 1;
        tempbank.push_back(baba);
      }
    }

    for (int j = 0; j < tempbank.size(); j++) {
      if (tempbank[j].tick >= 1) {
        newbank.push_back(tempbank[j]);
      }
    }

    // std::cout << "\nnewbank size:  " << newbank.size();		// debug

    // which element in newbank is closest to carpos in global coordinates?
    double temp = 100.0;
    for (int i = 0; i < newbank.size(); i++) {
      pointdist =
          sqrt(pow(newbank[i].x - odox, 2) + pow(newbank[i].y - odoy, 2));
      if (pointdist < temp) {
        temp = pointdist;
        indicator = i;
      }
    }
    drivebank.push_back(newbank[indicator]);
    iter_swap(newbank.begin(), newbank.begin() + indicator);

    for (int i = 0; i < newbank.size() - 1; i++) {
      temp = 100.0;
      pointdist = 900.0;
      for (int j = i + 1; j < newbank.size(); j++) {
        pointdist = sqrt(pow(newbank[j].x - newbank[i].x, 2) +
                         pow(newbank[j].y - newbank[i].y, 2));
        if (pointdist < temp) {
          temp = pointdist;
          indicator = j;
        }
      }
      iter_swap(newbank.begin() + i + 1, newbank.begin() + indicator);
    }
    /*
    std::cout<<"\nodox: " << odox << "   odoy: " << odoy << "\n";
    for(int i=0;i<newbank.size();i++){
            std::cout<<"\nnewbank i: " << i << "  x: " <<newbank[i].x << "  y:
    " << newbank[i].y << "   tick: " << newbank[i].tick;
    }
    */
    global_points = newbank;

  } // else

  // std::cout<<"\nglobal_points size: " << global_points.size();

  // transform to local
  for (int i = 0; i < global_points.size(); i++) {
    x = global_points[i].x;
    y = global_points[i].y;
    localx = (cos(-odotheta) * (x - odox)) -
             (sin(-odotheta) * (y - odoy)); // matrix transformation from world
                                            // coordinates to local coordinates
    localy = (sin(-odotheta) * (x - odox)) + (cos(-odotheta) * (y - odoy));
    xvec2.push_back(localx);
    yvec2.push_back(localy);
  }

  // Done
  xvec_global = xvec2;
  yvec_global = yvec2;

  ros::Duration t2 = ros::Time::now() - t1;
  std::cout << "\n\n runtime: " << t2.toSec();

  th = 0.0;
  double temp = 99;
  double precision = 0.4; // distance between points
  int num = 3.0 / precision - 1;
  if (xvec2.size() > 1) {
    posesGlobal = smooth(xvec2, yvec2, precision, 0);
    posesGlobalGlobal = smooth(xvec2, yvec2, precision, 1);
    /*
                    if(posesGlobal.size()>num){
                            th=atan2(posesGlobal[num].y-posesGlobal[num-1].y,posesGlobal[num].x-posesGlobal[num-1].x);
                            x=posesGlobal[num].x;
                            y=posesGlobal[num].y;
                    }
            */
  }

  return drivepoints2;
}

void rvizzer(std::vector<vertex> drive_points) {

  uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;
  ros::Duration one_sec(.1);

  for (int i = 0; i < drive_points.size() - 1; i++) {
    if (drive_points[i].edges == 0)
      continue;
    // Set the frame ID and timestamp.  See the TF tutorials for information on
    // these.
    // marker.header.frame_id = "visualization_frame";
    marker.header.frame_id = "visualization_frame"; //"base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "drive_point_arrow";
    marker.id = i; // i
    // Set the marker type.
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    marker.pose.position.x = drive_points[i].v0.x;
    marker.pose.position.y = drive_points[i].v0.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = drive_points[i].v_e[0].x;
    marker.pose.orientation.y = drive_points[i].v_e[0].y;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- depends whether using marker.points, or
    // marker.pose and marker.orientation
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a =
        1.0; // different from 0, unless you want it to be invisible.

    marker.lifetime = one_sec; // ros::Duration means it will never auto-delete.

    marker_pub.publish(marker);
  }
}

void rvizzer2(std::vector<vertex> drive_points) {
  std::vector<double> xvec;
  std::vector<double> yvec;

  uint32_t shape = visualization_msgs::Marker::POINTS;
  visualization_msgs::Marker points, points2, points3, points_line,
      points_line2;
  ros::Duration one_sec(.4);

  points.header.frame_id = points2.header.frame_id = points3.header.frame_id =
      points_line.header.frame_id = points_line2.header.frame_id =
          "visualization_frame"; //"base_link";
  points.header.stamp = points2.header.stamp = points3.header.stamp =
      points_line.header.stamp = points_line2.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  // Any marker sent with the same namespace and id will overwrite the old one
  points.ns = "points";
  points2.ns = "points_corrected";
  points3.ns = "points_corrected_global";
  points_line.ns = "points_line";
  points_line2.ns = "points_line_debug";

  points.action = points2.action = points3.action = points_line.action =
      points_line2.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = points2.pose.orientation.w =
      points3.pose.orientation.w = points_line.pose.orientation.w =
          points_line2.pose.orientation.w = 1.0;

  points.id = 0;
  points2.id = 1;
  points3.id = 2;
  points_line.id = 3;
  points_line2.id = 4;

  // Set the marker type.

  points.type = visualization_msgs::Marker::POINTS;
  points2.type = visualization_msgs::Marker::POINTS;
  points3.type = visualization_msgs::Marker::POINTS;
  points_line.type = visualization_msgs::Marker::POINTS;
  points_line2.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  points2.scale.x = 0.2;
  points2.scale.y = 0.2;

  points3.scale.x = 0.2;
  points3.scale.y = 0.2;

  points_line.scale.x = 0.2;
  points_line.scale.y = 0.2;

  points_line2.scale.x = 0.2;
  points_line2.scale.y = 0.2;

  points.color.b = 1.0; // blue
  points.color.a = 1.0;

  points2.color.g = 1.0f; // green
  points2.color.a = 1.0;

  points3.color.r = 1.0f; // red
  points3.color.a = 1.0;

  if (fast)
    points_line.color.g = 1.0f; // green
  if (!fast)
    points_line.color.b = 1.0f; // blue
  if (bad == 1)
    points_line.color.b = points_line.color.g = 1.0f;

  points_line.color.a = 1.0;

  points_line2.color.r = 1.0f; // red
  points_line2.color.a = 1.0;

  if (xvec_global2.size() == 0)
    return;
  for (int i = 0; i < x_allMsg.size(); i++) {//i < xvec_global2.size(); i++) { DLSH changed
    // Set the frame ID and timestamp.  See the TF tutorials for information on
    // these.
    // marker.header.frame_id = "visualization_frame";

    geometry_msgs::Point p;

    //p.x = xvec_global2[i];
    //p.y = yvec_global2[i];
//DLSH ADD
    p.x=x_allMsg[i];
    p.y=y_allMsg[i];
    //DLSH
    //		  p.x = drive_points[i].v0.x;
    //		  p.y = drive_points[i].v0.y;
    //xvec.push_back(drive_points[i].v0.x);
    //yvec.push_back(drive_points[i].v0.y);
    p.z = 0;

    points.points.push_back(p);
  }
  double precision = 0.4; // distance between points
  // std::vector<Pos> poses = smooth(xvec,yvec,precision,0);
  std::vector<Pos> poses = posesGlobal;

  for (int i = 0; i < poses.size(); i++) {
    geometry_msgs::Point p2;
    p2.x = poses[i].x;
    p2.y = poses[i].y;
    p2.z = 0;
    points2.points.push_back(p2);
  }
  std::vector<Pos> poses2 = posesGlobalGlobal;
  for (int i = 2.9 / precision; i < poses2.size();
       i++) { // i=29, since starting from 3m ahead
    geometry_msgs::Point p2;
    p2.x = poses2[i].x;
    p2.y = poses2[i].y;
    p2.z = 0;
    points3.points.push_back(p2);
  }

  for (int i = 0; i < poses_debug.size(); i++) {
    geometry_msgs::Point p2;
    // calculate points to global coordinates
    double globalx = cos(odotheta) * poses_debug[i].x -
                     sin(odotheta) * poses_debug[i].y + odox;
    double globaly = sin(odotheta) * poses_debug[i].x +
                     cos(odotheta) * poses_debug[i].y + odoy;

    p2.x = globalx;
    p2.y = globaly;
    p2.z = 0;

    points_line2.points.push_back(p2);
  }

  // points_line
  std::vector<Pos> poses_line = posesGlobalGlobal;
  int amount = 15;
  double resol = 0.25;
  for (int i = 2.5 / resol; i < 2.5 / resol + amount; i++) {
    geometry_msgs::Point p2;
    p2.x = line_x + cos(line_th) * resol * i;
    p2.y = line_y + sin(line_th) * resol * i;
    p2.z = 0;
    points_line.points.push_back(p2);
  }

  // points.lifetime = points2.lifetime = points3.lifetime =
  // points_line.lifetime = one_sec;
  // points.lifetime = points2.lifetime = points3.lifetime =
  // points_line.lifetime = one_sec;
  marker_pub.publish(points);
  marker_pub.publish(points2);
  marker_pub.publish(points3);
  marker_pub.publish(points_line);
  marker_pub.publish(points_line2);
}

void carPoseCallback(
    const std_msgs::Float32MultiArray::ConstPtr &msg_carPoseEstimate) {
  // carVal[0]: x
  // carVal[1]: y
  // carVal[3]: theta [rad] (orientation of car)
  // carVal[6]: drivendist

  // drivendist = msg_carPoseEstimate->data[6];   // updates global variable
  theta = msg_carPoseEstimate->data[3];
  avg_speed = msg_carPoseEstimate->data[4];
}

void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg) {
  speed = msg->speed_wheel;
  drivendist = msg->distance_wheel;
  //	if (msg->distance_front < 5.0) sonar = msg->distance_front;
}

void masterCallback(const auto_navi::commandMsg::ConstPtr &msg_data) {
  ct = msg_data->comtype;
  td = msg_data->targetdist;

  std::cout << "\nReceived command: " << ct << "   " << td;

  if (ct == 99) {
    master_switch = 0;
    std::cout << "\n\n\n * * * * * * * * TERMINAKTOR * * * * * * * *\n\n";
    // ros::shutdown();    // kills the node (comcount is not updated)
  }

  if (timetemp == 0 && td > 0.0) {
    // drive until drivendist-tempdist >= targetdist ( done in flagSender() )
    std::cout << "\nGoing for " << td << " m";
    tempdist = drivendist;
    timetemp = 1;
  }
}

void shutter() {
  // shutdown drive and motortester nodes
  /*
          auto_navi::emergencyMsg emer;
          emer.master_switch = false;
          emergency_publisher.publish(emer);

          // maybe delay here

          // cut the engine
          dynamo_msgs::TeensyWrite teensyMsg;
          teensyMsg.burn = false;
          //teensyMsg.autopilot_active = false;
          teensyPub.publish(teensyMsg);

          // maybe delay here

  */
  // brake hard
  dynamo_msgs::BrakeStepper bremse;
  bremse.brake_stepper_engaged = true;
  bremse.brake_power = 10.0;
  brake_pub.publish(bremse);
  /*
          // cut the steering stepper

          dynamo_msgs::SteeringStepper steeringmsg;
          steeringmsg.steering_stepper_engaged = 0;
          steerPub.publish(steeringmsg);

          ROS_WARN("emergency shutdown");
          ros::shutdown();

  */
}

void scanfixer(const lidar_package::obsts::ConstPtr &msg) {
    //DLSH ADD
for(int index_test_i=0;index_test_i<msg->vector_len;++index_test_i){
    for (int index_test_j = 0; index_test_j < msg->vector_obst[index_test_i].vector_len; ++index_test_j) {
        x_allMsg.push_back(msg->vector_obst[index_test_i].vector_point[index_test_j].x);
        y_allMsg.push_back(msg->vector_obst[index_test_i].vector_point[index_test_j].y);
    }
}
//DLSH



  // This function takes the lidar data from vlp16_obst_detection and prepares
  // them for Voronoi

  // The important thresholds * * * * * * * * * * * * * * * *
  double id_radius =
      2.0;         // points within this distance will be in the same wall-id
  double RW = 6.0; // roadwidth

  // * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

  // Ini
  uint8_t type2 = 0;

  // debug mandag
  // std::ofstream myfile;
  // myfile.open("log_mandag.txt", std::ios::out | std::ios::app); // filename,
  // append lines

  std::vector<double> x1; // contains coordinates
  std::vector<double> y1;
  std::vector<double> x2; // contains coordinates
  std::vector<double> y2;
  std::vector<double> xl; // contains coordinates
  std::vector<double> yl;
  std::vector<double> xr; // contains coordinates
  std::vector<double> yr;
  std::vector<double> xobst1; // contains coordinates
  std::vector<double> yobst1;
  std::vector<double> xobst2; // contains coordinates
  std::vector<double> yobst2;
  std::vector<double> xall; // contains coordinates
  std::vector<double> yall;
  std::vector<int> idall; // contains ids

  std::vector<int> id1; // contains ids
  std::vector<int> id2; // contains ids
  double laserx;        // msg->vector_obst.vector_point.x
  double lasery;        // msg->vector_obst.vector_point.y
  double localx;        // laserx in local coordinates
  double localy;        // lasery in local coordinates
  double xtemp = 0.0;
  double ytemp = 0.0;
  double temp1;
  double temp2;
  int index1;
  int index2;
  int scantemp = 0;
  int i = 0; // for loop values
  int j = 0;
  int k = 0;
  int indicator = 0;
  double maxtemp = 50.0;  // positive
  double mintemp = -50.0; // negative
  double temp = 0.0;
  int maxindex = 0;        // positive
  int minindex = 0;        // negative
  int groupcount = 0;      // used to count no. of groups
  int idcount = 0;         // used to count no. of walls
  double pointdist1 = 0.0; // used to calculate distance between two points
  double pointdist2 = 0.0;
  // std::ofstream myfile;
  // myfile.open("log_tirsdag2.txt", std::ios::out | std::ios::app); //
  // filename, append lines

  t1 = ros::Time::now(); // un-comment this for program runtime
  // ros::Duration t2 = ros::Time::now() - t1;	// debug mandag

  // t2 = ros::Time::now() - t1; //TAGER TID

  odox = msg->x;               // odometry x
  odoy = msg->y;               // odometery y
  odotheta = msg->orientation; // odometry theta
  emergency_count = 0;

  // if (sonar-0.46 <= 1.2) shutter();

  for (int i = 0; i < msg->vector_len; i++) {
    for (int j = 0; j < msg->vector_obst[i].vector_len; j++) {
      // calculate from world coordinates to local coordinates
      laserx = msg->vector_obst[i].vector_point[j].x;
      lasery = msg->vector_obst[i].vector_point[j].y;
      localx = (cos(-odotheta) * (laserx - odox)) -
               (sin(-odotheta) * (lasery - odoy)); // matrix transformation from
                                                   // world coordinates to local
                                                   // coordinates
      localy = (sin(-odotheta) * (laserx - odox)) +
               (cos(-odotheta) * (lasery - odoy));

      // Emergency stop: stop if object within x=0..1.0, y=-0.65..0.65 and turn
      // off everything
      /*PARKING CHALLENGE*/

      if (parking == 1) {
        if ((localx <= 5.2 && localx >= 1.7) && abs(localy) <= 0.8) {
          std::cout << "\n\n localx: " << localx << "   localy: " << localy;
          shutter();
          emergency_count++;
          // if (emergency_count > 5) shutter();
        }
      }

      // then push back
      x1.push_back(localx + BW2L); // adding .point.x to x1 in local coordinates
      y1.push_back(localy);        // adding .point.y to y1 in local coordinates

      // issue id
      if (localy >= 0) {
        id1.push_back(1);
        // std::cout<<"\n"<<localx<<" "<<localy << " 1";
      } else {
        id1.push_back(2);
        // std::cout<<"\n"<<localx<<" "<<localy << " 2";
      }
    }
  }

  if (x1.size() == 0)
    return;
  x2.push_back(x1[0]);
  y2.push_back(y1[0]);
  id2.push_back(id1[0]);

  temp1 = 9999;

  // Points are sorted according to x
  for (int i = 1; i < x1.size(); i++) {
    for (int j = 0; j < x2.size(); j++) {
      if (x1[i] <= x2[j]) {
        x2.insert(x2.begin() + j, x1[i]); // insert object if x1[i] <= x2[j]
        y2.insert(y2.begin() + j, y1[i]);
        id2.insert(id2.begin() + j, id1[i]);
        break;
      }
      if (j == x2.size() - 1) { // push to end if end of array
        x2.push_back(x1[i]);
        y2.push_back(y1[i]);
        id2.push_back(id1[i]);
        break;
      }
    }
  }
  int indexid1 = x2.size();
  int indexid2 = x2.size();
  double distancetempL = 99.0;
  double distancetempR = 99.0;
  int counter = 0;
  for (int i = 0; i < x2.size(); i++) {
    pointdist1 = sqrt(pow(x2[i], 2) + pow(y2[i], 2));
    if (id2[i] == 1 && distancetempL > pointdist1 &&
        x2[i] < BW2FW + DIC) { // Find first part of wall on left (maybe find
                               // left part that is closest to )
      indexid1 = i;
      distancetempL = pointdist1; // meget vigtig linje
    } else if (id2[i] == 2 && distancetempR > pointdist1 &&
               x2[i] < BW2FW + DIC) { // Find first part of wall on right
      indexid2 = i;
      distancetempR = pointdist1;
    } else {
      counter++;
    }

    if (counter > 50)
      break; // since array is sorted for x. stop as soon as first parts have
             // been found (no need to check all data)
  }

  xl.push_back(x2[indexid1]);
  yl.push_back(y2[indexid1]);
  xr.push_back(x2[indexid2]);
  yr.push_back(y2[indexid2]);
  double distL = 0, distR = 0;
  double furthest_dist1 = 0;
  double furthest_dist2 = 0;
  // indexid1 now shows the first element with id=1. likewise for indexid2

  std::vector<int> skip(x2.size(), 0);
  int flag;
  for (int i = 0; i < x2.size(); i++) {
    flag = 0;
    for (int j = 0; j < x2.size(); j++) {
      if (skip[i] == 1)
        break;
      if (j < xl.size()) {
        pointdist1 =
            sqrt(pow((x2.at(i) - xl.at(j)), 2) + pow((y2.at(i) - yl.at(j)), 2));

        if (pointdist1 < id_radius) {
          distL = sqrt(pow((x2.at(i)), 2) + pow((y2.at(i)), 2));
          if (furthest_dist1 < distL)
            furthest_dist1 = distL;
          xl.push_back(x2[i]);
          yl.push_back(y2[i]);
          xall.push_back(x2[i]);
          yall.push_back(y2[i]);
          idall.push_back(1);
          id2[i] = 1;
          skip[i] = 1;
          break;
          // myfile<<x2[i]<<" "<<y2[i]<<" "<<1<<" "<<t2.toSec()<< " 0 " <<"\n";
        }
      }
      if (j < xr.size()) {
        pointdist2 =
            sqrt(pow((x2.at(i) - xr.at(j)), 2) + pow((y2.at(i) - yr.at(j)), 2));
        if (pointdist2 < id_radius) {
          distR = sqrt(pow((x2.at(i)), 2) + pow((y2.at(i)), 2));
          if (furthest_dist1 < distR)
            furthest_dist1 = distR;
          xr.push_back(x2[i]);
          yr.push_back(y2[i]);
          xall.push_back(x2[i]);
          yall.push_back(y2[i]);
          idall.push_back(2);
          id2[i] = 2;
          skip[i] = 1;
          break;
          // myfile<<x2[i]<<" "<<y2[i]<<" "<<2<<" "<<t2.toSec()<< " 0 "<<"\n";
        }
      }
      if (pointdist1 < RW + id_radius &&
          pointdist2 < RW + id_radius) { // if close to both left and right
                                         // (<7m) but not part of left nor
                                         // right, must be obstacle on track!
        flag = 1;
      }
      if (j >= xr.size() - 1 && j >= xl.size() - 1 && flag == 1) {
        xobst1.push_back(x2[i]);
        yobst1.push_back(y2[i]);
        skip[i] = 1;
      }
    }
  }

  dist_max = (distL + distR) / 2;
  if (dist_max < 4.4)
    dist_max = 4.4;
  for (int i = 0; i < x2.size(); i++) {
    for (int j = 0; j < xobst1.size(); j++) {
      if (skip[i] == 1)
        break;

      pointdist1 = sqrt(pow((x2.at(i) - xobst1.at(j)), 2) +
                        pow((y2.at(i) - yobst1.at(j)), 2));
      if (pointdist1 < id_radius) {
        xobst1.push_back(x2[i]);
        yobst1.push_back(y2[i]);
        skip[i] = 1;
      }
    }
  }
  // INITIALIZE first obstacle as ID3
  int idcounter = 3;
  if (xobst1.size() > 0) { // if any obstacles were found
    xobst2.push_back(xobst1[0]);
    yobst2.push_back(yobst1[0]);
    std::vector<int> idobst2(xobst2.size(), 3); // contains ids
    xall.push_back(xobst1[0]);
    yall.push_back(yobst2[0]);
    idall.push_back(idobst2[0]);

    // HANDLE IDs of obstacles.
    for (int i = 0; i < xobst1.size(); i++) {
      for (int j = 0; j < xobst2.size(); j++) {

        pointdist1 = sqrt(pow((xobst1.at(i) - xobst2.at(j)), 2) +
                          pow((yobst1.at(i) - yobst2.at(j)), 2));
        if (pointdist1 <
            id_radius) { // IF CLOSE TO EXISTING OBSTACLE ADD AS SAME OBSTACLE
          xobst2.push_back(xobst1[i]);
          yobst2.push_back(yobst1[i]);
          idobst2.push_back(idobst2[j]);
          xall.push_back(xobst1[i]);
          yall.push_back(yobst1[i]);
          idall.push_back(idobst2[j]);
          break;
        }
        if (j == xobst2.size() - 1) { // IF NOT CLOSE TO ANY EXISTING OBSTACLE
                                      // ADD AS NEW OBSTACLE
          idcounter++;
          xobst2.push_back(xobst1[i]);
          yobst2.push_back(yobst1[i]);
          idobst2.push_back(idcounter);
          xall.push_back(xobst1[i]);
          yall.push_back(yobst1[i]);
          idall.push_back(idcounter);
        }
      }
    }
  }

  xvec_global2 = xall;
  yvec_global2 = yall;

  std::vector<vertex> drive_points = voronoi_run(xall, yall, idall);
  drive_points = point_handler(drive_points);
  rvizzer2(drive_points); // to rviz		(was commented during C1 lap)
}

void parker(const geometry_msgs::PolygonStamped::ConstPtr &msg) {
  // this node receives message from camera node
  parking = 1;
  // printf("PARKING");
}

int main(int argc, char **argv) {
  // Initialization
  ros::init(argc, argv, "voronoi_node");
  ros::NodeHandle n;
  emergency_count = 0;
  flag = 0;  // first flag set to 0
  state = 0; // first state ini to 0 (to await new state from drivemaster)
  master_switch = 1;    // maybe unnecessary, used to shut things off
  timetemp = 0;         // used to save globally targetdist for at given command
  comcounter = 0;       // no of completed commands
  brakestop = 0;        // global, ini to 0
  max_brake_power = 25; // [bar]

  current_ct = 77; // temp. values, overwritten at first new command.
  current_flag = 77;
  current_td = 777.0;
  t1 = ros::Time::now(); // time at node start

  // Subscribers
  ros::Subscriber sub = n.subscribe("obstacle_hulls", 1000, scanfixer);
  ros::Subscriber sub_odo =
      n.subscribe("car_pose_estimate", 1000, carPoseCallback);
  ros::Subscriber parking_sub =
      n.subscribe("parking_spot_detection/corners", 10, parker);
  ros::Subscriber teensyreadsub =
      n.subscribe("teensy_read", 10, teensyCallback);

  // Publishers
  brake_pub = n.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power", 10);
  steerPub =
      n.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle", 100);
  emergency_publisher =
      n.advertise<auto_navi::emergencyMsg>("cmd_master_switch", 10);
  teensyPub = n.advertise<dynamo_msgs::TeensyWrite>("teensy_write", 100);
  // flag_pub = n.advertise<auto_navi::flagMsg>("scanmaker_flags", 10);
  // //
  // til missions og states, vent med denne
  voroPub = n.advertise<auto_navi::driveMsg>("drive_points", 1);
  motoPub = n.advertise<auto_navi::motorMsg>("fast", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("arrow_marker", 0);

  float loopRate = 20; // loop rate in [Hz]
  ros::Rate r(loopRate);
  ROS_INFO("Starting main loop.");

  double precision = 0.4;
  while (ros::ok()) {
    ros::spinOnce();

    if (parking == 0)
      point_sender(precision);
    // flagSender();
    r.sleep();
    // printf("PARKING IS %d",parking);
  }

  //  ros::spin();

  std::cout << "\n\n\nROS no longer ok \n\n\n";
  return 0;
}
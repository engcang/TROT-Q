#ifndef PEACOCK_H
#define PEACOCK_H

#include "utility.h" // for refinement -> Euclidean

///// common headers for peacock
#include <iostream> //cout
#include <chrono> 
#include <math.h> // pow
#include <vector> //std::reverse

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <unsupported/Eigen/CXX11/Tensor> // for Tensor!!

#define d2r M_PI/180.0
#define r2d 180.0/M_PI

using namespace std::chrono; 
using namespace std;
using namespace Eigen;

///// for local planner
int g_row = 3;
int g_col = 37;
int g_branch = 7;

////// V*T = length of trajectory, recommend half of the sensor range
double g_v = 0.5; // Linear velocity
double g_T = 1.75/0.5; // time for next step

double g_yaw_unit = 5.0;
double g_pitch_unit = 5.0;
double g_branch_yaw_unit = 5.0;

double g_yaw_curr = 0.0; // current yaw of uav
MatrixXd g_yaw_init = MatrixXd::Zero(g_row,g_col); // +floor(g_col/2)*g_yaw_unit ~ -floor(g_col/2)*g_yaw_unit yaw -> in radian by multiplying d2r
MatrixXd g_yaw = MatrixXd::Zero(g_row,g_col); // g_yaw_init + g_yaw_curr

MatrixXd g_pitch = MatrixXd::Zero(g_row,g_col); // -floor(g_row/2)*g_pitch_unit ~ +floor(g_row/2)*g_pitch_unit -> in radian by multiplying d2r
MatrixXd g_pitch_z = MatrixXd::Zero(g_row,1); // -floor(g_row/2)*g_pitch_unit ~ +floor(g_row/2)*g_pitch_unit -> in radian by multiplying d2r

MatrixXd g_x_1 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_x_2 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_y_1 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_y_2 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_vx_1 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_vx_2 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_vy_1 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_vy_2 = MatrixXd::Zero(g_row,g_col);
MatrixXd g_z_1 = MatrixXd::Zero(g_row,1);
MatrixXd g_z_2 = MatrixXd::Zero(g_row,1);
MatrixXd g_vz_1 = MatrixXd::Zero(g_row,1); // zero
MatrixXd g_vz_2 = MatrixXd::Zero(g_row,1); // zero

Tensor<double, 3> g_yaw_bar(g_row,g_col,g_branch); // for higher than 2-dimentional rank matrix.
Tensor<double, 3> g_sx_2(g_row,g_col,g_branch);
Tensor<double, 3> g_sy_2(g_row,g_col,g_branch);
MatrixXd g_sz_2 = MatrixXd::Zero(g_row,1);
Tensor<double, 3> g_svx_2(g_row,g_col,g_branch);
Tensor<double, 3> g_svy_2(g_row,g_col,g_branch);
MatrixXd g_svz_2 = MatrixXd::Zero(g_row,1); // zero

MatrixXd g_A = MatrixXd::Zero(6,6);
Tensor<double, 3> g_cx(g_row,g_col,6); // for higher than 2-dimentional rank matrix.
Tensor<double, 3> g_cy(g_row,g_col,6);
MatrixXd g_cz = MatrixXd::Zero(g_row,6);
Tensor<double, 4> g_scx(g_row,g_col,g_branch,6); // for higher than 2-dimentional rank matrix.
Tensor<double, 4> g_scy(g_row,g_col,g_branch,6);
MatrixXd g_scz = MatrixXd::Zero(g_row,6);
bool g_local_init=false;

MatrixXd g_loc_score = MatrixXd::Zero(g_row,g_col);   //score for counting feasible paths
MatrixXd g_distance_score = MatrixXd::Zero(g_row,g_col);   //score for counting feasible paths
MatrixXf::Index g_best_row, g_best_col;
float g_max_score;


/////////// For path planning
void best_score_path(MatrixXd score_mat, MatrixXd dist_score_mat){
  // max_frontier_score = frontier_score_mat.maxCoeff(&g_best_row, &g_best_col);
  // if (max_frontier_score>1.2) 
  score_mat = score_mat.array() * dist_score_mat.array();
  g_max_score = score_mat.maxCoeff(&g_best_row, &g_best_col);

  vector<int> tmp_cols, tmp_rows;
  for (int i = 0; i < g_row; i++){
    for (int j = 0; j < g_col; j++){
      if (abs(score_mat(i,j)-g_max_score)<0.01){  // numerical error tolerance
        tmp_rows.push_back(i);
        tmp_cols.push_back(j);
      }
    }
  }

  sort(tmp_cols.begin(), tmp_cols.end());
  sort(tmp_rows.begin(), tmp_rows.end());
  g_best_col = tmp_cols.at(floor(tmp_cols.size()/2)); //median
  g_best_row = tmp_rows.at(floor(tmp_rows.size()/2)); //median

  return;
}


void minimum_jerk_trajectories_initialize(){

  auto t_start = high_resolution_clock::now();

  ////// yaw, pitch init
  for (int i=0; i<g_row; i++)
  {
    for (int j=0; j<g_col; j++)
    {
       g_yaw_init(i,j) = -(j-floor(g_col/2))*g_yaw_unit * d2r; // yaw : + --> -
       g_pitch(i,j) = (i-floor(g_row/2))*g_pitch_unit * d2r;
    }
    g_pitch_z(i,0) = (i-floor(g_row/2))*g_pitch_unit * d2r;
  } // should be run once
  g_yaw = g_yaw_init.array() + g_yaw_curr;

  ///// x,y,z vx,vy,vz constraints set
  g_x_2 = (g_x_1.array() + g_v*g_T*cos(g_yaw.array())) * cos(g_pitch.array());
  g_y_2 = (g_y_1.array() + g_v*g_T*sin(g_yaw.array())) * cos(g_pitch.array());
  g_z_2 = g_z_1.array() - g_v*g_T*sin(g_pitch_z.array());
  g_sz_2 = g_z_2.array() - g_v*g_T*sin(g_pitch_z.array());

  g_vx_1 = g_vx_1.array() + g_v*cos(g_yaw_curr);
  g_vy_1 = g_vy_1.array() + g_v*sin(g_yaw_curr);

  g_vx_2 = g_v*cos(g_yaw.array()) * cos(g_pitch.array()); 
  g_vy_2 = g_v*sin(g_yaw.array()) * cos(g_pitch.array());
  g_vz_2 = g_vz_1.array() - g_v*sin(g_pitch_z.array());
  g_svz_2 = g_vz_2;

  ///// Ax=b linear systems solving, x = g_cx, g_cy, g_cz
  g_A << 0, 0, 0, 0, 0, 1,
    pow(g_T,5), pow(g_T,4), pow(g_T,3), pow(g_T,2), g_T, 1,
    0, 0, 0, 0, 1, 0,
    5*pow(g_T,4), 4*pow(g_T,3), 3*pow(g_T,2), 2*g_T, 1, 0,
    0, 0, 0, 2, 0, 0,
    20*pow(g_T,3), 12*pow(g_T,2), 6*g_T, 2, 0, 0;

  for (int i=0; i<g_row; i++)
  {
    VectorXd b_z(6);
    VectorXd z_(6);
    b_z << g_z_1(i), g_z_2(i), g_vz_1(i), g_vz_2(i), 0, 0; //g_vz_1=g_vz_2=0, a=0
    z_ = g_A.lu().solve(b_z);
    g_cz.row(i) = z_;
    for (int j=0; j<g_col; j++)
    {
       VectorXd b_x(6);
       VectorXd b_y(6);
       VectorXd x_(6);
       VectorXd y_(6);
       b_x << g_x_1(i,j), g_x_2(i,j), g_vx_1(i,j), g_vx_2(i,j), 0, 0; //a=0
       b_y << g_y_1(i,j), g_y_2(i,j), g_vy_1(i,j), g_vy_2(i,j), 0, 0; //a=0
       x_ = g_A.lu().solve(b_x);
       y_ = g_A.lu().solve(b_y);
       g_cx(i,j,0) = x_(0);
       g_cx(i,j,1) = x_(1);
       g_cx(i,j,2) = x_(2);
       g_cx(i,j,3) = x_(3);
       g_cx(i,j,4) = x_(4);
       g_cx(i,j,5) = x_(5);
       g_cy(i,j,0) = y_(0);
       g_cy(i,j,1) = y_(1);
       g_cy(i,j,2) = y_(2);
       g_cy(i,j,3) = y_(3);
       g_cy(i,j,4) = y_(4);
       g_cy(i,j,5) = y_(5);
    }
  }

  auto t_stop = high_resolution_clock::now();
  auto t_duration = duration_cast<microseconds>(t_stop - t_start);
  ROS_WARN("Big branches,,, %.3f ms spent...", t_duration.count()/1000.0);
  // cout <<"       Big branches,,, " << t_duration.count()/1000.0 << " ms spent" << endl;
  ///// end of First big branches /////////////////////////////////////////////////////////////////////////////////

  ///// Second branches /////////////////////////////////
  auto t_start2 = high_resolution_clock::now();

  for (int i = 0; i < g_row; ++i)
  {
    VectorXd b_z(6);
    VectorXd z_(6);
    b_z << g_z_2(i), g_sz_2(i), g_vz_2(i), g_svz_2(i), 0, 0; //g_z_2=g_z_2, g_vz_2=g_svz_2=0 //a=0
    z_ = g_A.lu().solve(b_z);
    g_scz.row(i) = z_;
    for (int j = 0; j < g_col; ++j)
    {
       for (int b = 0; b < g_branch; b++)
       {
          g_yaw_bar(i,j,b) = g_yaw(i,j) - (b-floor(g_branch/2)) * g_branch_yaw_unit *d2r; // yaw : + --> -
          g_sx_2(i,j,b) = g_x_2(i,j) + g_v*g_T*cos(g_yaw_bar(i,j,b)) * cos(g_pitch(i,j));
          g_sy_2(i,j,b) = g_y_2(i,j) + g_v*g_T*sin(g_yaw_bar(i,j,b)) * cos(g_pitch(i,j));
          g_svx_2(i,j,b) = g_v*cos(g_yaw_bar(i,j,b)) * cos(g_pitch(i,j));
          g_svy_2(i,j,b) = g_v*sin(g_yaw_bar(i,j,b)) * cos(g_pitch(i,j));

          VectorXd b_x(6);
          VectorXd b_y(6);
          VectorXd x_(6);
          VectorXd y_(6);
          b_x << g_x_2(i,j), g_sx_2(i,j,b), g_vx_2(i,j), g_svx_2(i,j,b), 0, 0; //a=0
          b_y << g_y_2(i,j), g_sy_2(i,j,b), g_vy_2(i,j), g_svy_2(i,j,b), 0, 0; //a=0
          x_ = g_A.lu().solve(b_x);
          y_ = g_A.lu().solve(b_y);
          g_scx(i,j,b,0) = x_(0);
          g_scx(i,j,b,1) = x_(1);
          g_scx(i,j,b,2) = x_(2);
          g_scx(i,j,b,3) = x_(3);
          g_scx(i,j,b,4) = x_(4);
          g_scx(i,j,b,5) = x_(5);
          g_scy(i,j,b,0) = y_(0);
          g_scy(i,j,b,1) = y_(1);
          g_scy(i,j,b,2) = y_(2);
          g_scy(i,j,b,3) = y_(3);
          g_scy(i,j,b,4) = y_(4);
          g_scy(i,j,b,5) = y_(5);
       }
    }
  }
  auto t_stop2 = high_resolution_clock::now();
  auto t_duration2 = duration_cast<microseconds>(t_stop2 - t_start2);
  ROS_WARN("Second branches,,, %.3f ms spent...", t_duration2.count()/1000.0);
  // cout <<"       Second branches,,, " << t_duration2.count()/1000.0 << " ms spent" << endl;

  g_local_init=true;
}



#endif
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

#define sind(x) (sin(fmod((x),360.0) * M_PI / 180.0)) //cannot broadcast array, not used
#define cosd(x) (cos(fmod((x),360.0) * M_PI / 180.0)) //cannot broadcast array, not used
#define d2r M_PI/180.0
#define r2d 180.0/M_PI

using namespace std::chrono; 
using namespace std;
using namespace Eigen;

////// for refinement
vector<VectorXd> x_coeff, y_coeff, z_coeff;
vector<double> time_each;

///// for local planner
int row = 5;
int col = 37;
int branch = 7;

////// V*T = length of trajectory, recommend half of the sensor range
double v = 0.5; // Linear velocity
double T = 1.75/0.5; // time for next step

double yaw_unit = 5.0;
double pitch_unit = 5.0;
double branch_yaw_unit = 5.0;

double yaw_curr = 0.0; // current yaw of uav
MatrixXd yaw_init = MatrixXd::Zero(row,col); // +floor(col/2)*yaw_unit ~ -floor(col/2)*yaw_unit yaw -> in radian by multiplying d2r
MatrixXd yaw = MatrixXd::Zero(row,col); // yaw_init + yaw_curr

MatrixXd pitch = MatrixXd::Zero(row,col); // -floor(row/2)*pitch_unit ~ +floor(row/2)*pitch_unit -> in radian by multiplying d2r
MatrixXd pitch_z = MatrixXd::Zero(row,1); // -floor(row/2)*pitch_unit ~ +floor(row/2)*pitch_unit -> in radian by multiplying d2r

MatrixXd x_1 = MatrixXd::Zero(row,col);
MatrixXd x_2 = MatrixXd::Zero(row,col);
MatrixXd y_1 = MatrixXd::Zero(row,col);
MatrixXd y_2 = MatrixXd::Zero(row,col);
MatrixXd vx_1 = MatrixXd::Zero(row,col);
MatrixXd vx_2 = MatrixXd::Zero(row,col);
MatrixXd vy_1 = MatrixXd::Zero(row,col);
MatrixXd vy_2 = MatrixXd::Zero(row,col);
MatrixXd z_1 = MatrixXd::Zero(row,1);
MatrixXd z_2 = MatrixXd::Zero(row,1);
MatrixXd vz_1 = MatrixXd::Zero(row,1); // zero
MatrixXd vz_2 = MatrixXd::Zero(row,1); // zero

Tensor<double, 3> yaw_bar(row,col,branch); // for higher than 2-dimentional rank matrix.
Tensor<double, 3> sx_2(row,col,branch);
Tensor<double, 3> sy_2(row,col,branch);
MatrixXd sz_2 = MatrixXd::Zero(row,1);
Tensor<double, 3> svx_2(row,col,branch);
Tensor<double, 3> svy_2(row,col,branch);
MatrixXd svz_2 = MatrixXd::Zero(row,1); // zero

MatrixXd A = MatrixXd::Zero(6,6);
Tensor<double, 3> cx(row,col,6); // for higher than 2-dimentional rank matrix.
Tensor<double, 3> cy(row,col,6);
MatrixXd cz = MatrixXd::Zero(row,6);
Tensor<double, 4> scx(row,col,branch,6); // for higher than 2-dimentional rank matrix.
Tensor<double, 4> scy(row,col,branch,6);
MatrixXd scz = MatrixXd::Zero(row,6);
bool local_init=false;

MatrixXd loc_score = MatrixXd::Zero(row,col);   //score for counting feasible paths
MatrixXd straight_filter = MatrixXd::Zero(row,col);
MatrixXf::Index best_row, best_col;
float max_score;


/////////// For path planning
void best_score_path(MatrixXd score_mat){
  // max_frontier_score = frontier_score_mat.maxCoeff(&best_row, &best_col);
  // if (max_frontier_score>1.2) score_mat = score_mat.array() * frontier_score_mat.array();
  max_score = score_mat.maxCoeff(&best_row, &best_col);

  vector<int> tmp_cols, tmp_rows;
  for (int i = 0; i < row; i++){
    for (int j = 0; j < col; j++){
      if (abs(score_mat(i,j)-max_score)<0.01){  // numerical error tolerance
        tmp_rows.push_back(i);
        tmp_cols.push_back(j);
      }
    }
  }
  // if ((tmp_cols.size()>floor(col*row/2.0)) && (tmp_rows.size()>floor(col*row/2.0))){ // when score does not matter, straight
  //   score_mat = score_mat.array() * straight_filter.array();
  //   max_score = score_mat.maxCoeff(&best_row, &best_col);
  // }
  // else{
    sort(tmp_cols.begin(), tmp_cols.end());
    sort(tmp_rows.begin(), tmp_rows.end());
    best_col = tmp_cols.at(floor(tmp_cols.size()/2)); //median
    best_row = tmp_rows.at(floor(tmp_rows.size()/2)); //median
  // }
  return;
}


void minimum_jerk_trajectories_initialize(){

  auto t_start = high_resolution_clock::now();

  ////// yaw, pitch init
  for (int i=0; i<row; i++)
  {
    for (int j=0; j<col; j++)
    {
       yaw_init(i,j) = -(j-floor(col/2))*yaw_unit * d2r; // yaw : + --> -
       pitch(i,j) = (i-floor(row/2))*pitch_unit * d2r;
       straight_filter(i,j) = 1 - (abs(i-floor(row/2))+abs(j-floor(col/2)))*0.01;
    }
    pitch_z(i,0) = (i-floor(row/2))*pitch_unit * d2r;
  } // should be run once
  yaw = yaw_init.array() + yaw_curr;

  ///// x,y,z vx,vy,vz constraints set
  x_2 = (x_1.array() + v*T*cos(yaw.array())) * cos(pitch.array());
  y_2 = (y_1.array() + v*T*sin(yaw.array())) * cos(pitch.array());
  z_2 = z_1.array() - v*T*sin(pitch_z.array());
  sz_2 = z_2.array() - v*T*sin(pitch_z.array());

  vx_1 = vx_1.array() + v*cos(yaw_curr);
  vy_1 = vy_1.array() + v*sin(yaw_curr);
  // vx_2 = v*cos(yaw.array());
  // vy_2 = v*sin(yaw.array());
  vx_2 = v*cos(yaw.array()) * cos(pitch.array()); 
  vy_2 = v*sin(yaw.array()) * cos(pitch.array());
  vz_2 = vz_1.array() - v*sin(pitch_z.array());
  svz_2 = vz_2;

  ///// Ax=b linear systems solving, x = cx, cy, cz
  A << 0, 0, 0, 0, 0, 1,
    pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1,
    0, 0, 0, 0, 1, 0,
    5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0,
    0, 0, 0, 2, 0, 0,
    20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0;

  for (int i=0; i<row; i++)
  {
    VectorXd b_z(6);
    VectorXd z_(6);
    b_z << z_1(i), z_2(i), vz_1(i), vz_2(i), 0, 0; //vz_1=vz_2=0, a=0
    z_ = A.lu().solve(b_z);
    cz.row(i) = z_;
    for (int j=0; j<col; j++)
    {
       VectorXd b_x(6);
       VectorXd b_y(6);
       VectorXd x_(6);
       VectorXd y_(6);
       b_x << x_1(i,j), x_2(i,j), vx_1(i,j), vx_2(i,j), 0, 0; //a=0
       b_y << y_1(i,j), y_2(i,j), vy_1(i,j), vy_2(i,j), 0, 0; //a=0
       x_ = A.lu().solve(b_x);
       y_ = A.lu().solve(b_y);
       cx(i,j,0) = x_(0);
       cx(i,j,1) = x_(1);
       cx(i,j,2) = x_(2);
       cx(i,j,3) = x_(3);
       cx(i,j,4) = x_(4);
       cx(i,j,5) = x_(5);
       cy(i,j,0) = y_(0);
       cy(i,j,1) = y_(1);
       cy(i,j,2) = y_(2);
       cy(i,j,3) = y_(3);
       cy(i,j,4) = y_(4);
       cy(i,j,5) = y_(5);
    }
  }

  auto t_stop = high_resolution_clock::now();
  auto t_duration = duration_cast<microseconds>(t_stop - t_start);
  ROS_WARN("Big branches,,, %.3f ms spent...", t_duration.count()/1000.0);
  // cout <<"       Big branches,,, " << t_duration.count()/1000.0 << " ms spent" << endl;
  ///// end of First big branches /////////////////////////////////////////////////////////////////////////////////

  ///// Second branches /////////////////////////////////
  auto t_start2 = high_resolution_clock::now();

  for (int i = 0; i < row; ++i)
  {
    VectorXd b_z(6);
    VectorXd z_(6);
    b_z << z_2(i), sz_2(i), vz_2(i), svz_2(i), 0, 0; //z_2=z_2, vz_2=svz_2=0 //a=0
    z_ = A.lu().solve(b_z);
    scz.row(i) = z_;
    for (int j = 0; j < col; ++j)
    {
       for (int b = 0; b < branch; b++)
       {
          yaw_bar(i,j,b) = yaw(i,j) - (b-floor(branch/2)) * branch_yaw_unit *d2r; // yaw : + --> -
          sx_2(i,j,b) = x_2(i,j) + v*T*cos(yaw_bar(i,j,b)) * cos(pitch(i,j));
          sy_2(i,j,b) = y_2(i,j) + v*T*sin(yaw_bar(i,j,b)) * cos(pitch(i,j));
          svx_2(i,j,b) = v*cos(yaw_bar(i,j,b)) * cos(pitch(i,j));
          svy_2(i,j,b) = v*sin(yaw_bar(i,j,b)) * cos(pitch(i,j));
          // sx_2(i,j,b) = x_2(i,j) + v*T*cos(yaw_bar(i,j,b));
          // sy_2(i,j,b) = y_2(i,j) + v*T*sin(yaw_bar(i,j,b));
          // svx_2(i,j,b) = v*cos(yaw_bar(i,j,b));
          // svy_2(i,j,b) = v*sin(yaw_bar(i,j,b));

          VectorXd b_x(6);
          VectorXd b_y(6);
          VectorXd x_(6);
          VectorXd y_(6);
          b_x << x_2(i,j), sx_2(i,j,b), vx_2(i,j), svx_2(i,j,b), 0, 0; //a=0
          b_y << y_2(i,j), sy_2(i,j,b), vy_2(i,j), svy_2(i,j,b), 0, 0; //a=0
          x_ = A.lu().solve(b_x);
          y_ = A.lu().solve(b_y);
          scx(i,j,b,0) = x_(0);
          scx(i,j,b,1) = x_(1);
          scx(i,j,b,2) = x_(2);
          scx(i,j,b,3) = x_(3);
          scx(i,j,b,4) = x_(4);
          scx(i,j,b,5) = x_(5);
          scy(i,j,b,0) = y_(0);
          scy(i,j,b,1) = y_(1);
          scy(i,j,b,2) = y_(2);
          scy(i,j,b,3) = y_(3);
          scy(i,j,b,4) = y_(4);
          scy(i,j,b,5) = y_(5);
       }
    }
  }
  auto t_stop2 = high_resolution_clock::now();
  auto t_duration2 = duration_cast<microseconds>(t_stop2 - t_start2);
  ROS_WARN("Second branches,,, %.3f ms spent...", t_duration2.count()/1000.0);
  // cout <<"       Second branches,,, " << t_duration2.count()/1000.0 << " ms spent" << endl;

  local_init=true;
}



#endif
#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <chrono> 
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std::chrono; 
using namespace std;
using namespace Eigen;

/////////// utils
//////////// common
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

high_resolution_clock::time_point start; //global, to use tic()
void tic(){
   start = high_resolution_clock::now();
}
void toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%.3f ms spent", duration.count()/1000.0);
}
void toc(string text){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%s %.3f ms spent", text.c_str(), duration.count()/1000.0);
}

sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}

geometry_msgs::Point tf_point(geometry_msgs::Point point, Matrix4f tf){
  MatrixXf point_vec(4,1);
  point_vec << point.x, point.y, point.z, 1.0;
  point_vec = tf * point_vec; //transform

  geometry_msgs::Point point_tf;
  point_tf.x = point_vec(0,0);
  point_tf.y = point_vec(1,0);
  point_tf.z = point_vec(2,0);
  return point_tf;
}

geometry_msgs::PoseStamped tf_pose(geometry_msgs::PoseStamped pose, Matrix4f tf){
  Matrix4f pose_mat = Matrix4f::Identity();

  pose_mat(0,3) = pose.pose.position.x;
  pose_mat(1,3) = pose.pose.position.y;
  pose_mat(2,3) = pose.pose.position.z;
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  // q.normalize();
  tf::Matrix3x3 m(q); // quaternion to rotation matrix
  pose_mat(0,0) = m[0][0];
  pose_mat(0,1) = m[0][1];
  pose_mat(0,2) = m[0][2];
  pose_mat(1,0) = m[1][0];
  pose_mat(1,1) = m[1][1];
  pose_mat(1,2) = m[1][2];
  pose_mat(2,0) = m[2][0];
  pose_mat(2,1) = m[2][1];
  pose_mat(2,2) = m[2][2];
  
  Matrix4f tf_pose_mat = tf * pose_mat; //transform

  tf::Matrix3x3 m2;
  m2[0][0] = tf_pose_mat(0,0);
  m2[0][1] = tf_pose_mat(0,1);
  m2[0][2] = tf_pose_mat(0,2);
  m2[1][0] = tf_pose_mat(1,0);
  m2[1][1] = tf_pose_mat(1,1);
  m2[1][2] = tf_pose_mat(1,2);
  m2[2][0] = tf_pose_mat(2,0);
  m2[2][1] = tf_pose_mat(2,1);
  m2[2][2] = tf_pose_mat(2,2);

  tf::Quaternion q2;
  m2.getRotation(q2); // rotation matrix to quaternion
  // q2.normalize();

  geometry_msgs::PoseStamped pose_tf;
  pose_tf.pose.position.x = tf_pose_mat(0,3);
  pose_tf.pose.position.y = tf_pose_mat(1,3);
  pose_tf.pose.position.z = tf_pose_mat(2,3);
  pose_tf.pose.orientation.x = q2.getX();
  pose_tf.pose.orientation.y = q2.getY();
  pose_tf.pose.orientation.z = q2.getZ();
  pose_tf.pose.orientation.w = q2.getW();
  return pose_tf;
}

double euclidean_dist(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}
double euclidean_dist(double x1, double y1, double z1, double x2, double y2, double z2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}
double euclidean_dist(geometry_msgs::Point pos1, geometry_msgs::Point pos2){
  return sqrt(pow(pos1.x-pos2.x,2) + pow(pos1.y-pos2.y,2) + pow(pos1.z-pos2.z,2));
}
double euclidean_dist(geometry_msgs::Point pos1, octomap::point3d pos2){
  return sqrt(pow(pos1.x-pos2.x(),2) + pow(pos1.y-pos2.y(),2) + pow(pos1.z-pos2.z(),2));
}
double euclidean_dist(octomap::point3d pos2, geometry_msgs::Point pos1){
  return sqrt(pow(pos1.x-pos2.x(),2) + pow(pos1.y-pos2.y(),2) + pow(pos1.z-pos2.z(),2));
}
double euclidean_dist(octomap::point3d pos1, octomap::point3d pos2){
  return sqrt(pow(pos1.x()-pos2.x(),2) + pow(pos1.y()-pos2.y(),2) + pow(pos1.z()-pos2.z(),2));
}
double euclidean_dist(octomap::point3d pos1, double x2, double y2, double z2){
  return sqrt(pow(pos1.x()-x2,2) + pow(pos1.y()-y2,2) + pow(pos1.z()-z2,2));
}


/*
-----------------------------------------------------------------------------
  Name: CylTest_CapsFirst
  Orig: Greg James -
  gjames@NVIDIA.com Lisc:
  Free code - no warranty & no money back. Use it all you want Desc:
     This function tests if the 3D point 'pt' lies within an arbitrarily oriented cylinder.
  The cylinder is defined by an axis from 'pt1' to 'pt2', the axis having a length squared of 'lsq'
  (pre-compute for each cylinder to avoid repeated work!), and radius squared of 'rsq'.
     The function tests against the end caps first, which is cheap 
  -> only a single dot product to test against the parallel cylinder caps.  
  If the point is within these, more work is done to find the distance of the point from
  the cylinder axis.
     Fancy Math (TM) makes the whole test possible with only two dot-products a subtract, and two multiplies.
  For clarity, the 2nd mult is kept as a divide. 
  It might be faster to change this to a mult by also passing in 1/lengthsq and using that instead.
     Elminiate the first 3 subtracts by specifying the cylinder as a base point on one end cap and a
  vector to the other end cap (pass in {dx,dy,dz} instead of 'pt2' ).

     The dot product is constant along a plane perpendicular to a vector. 
  The magnitude of the cross product divided by one vector length is constant along a cylinder surface
  defined by the other vector as axis.

  Return:  -1.0 if point is outside the cylinder 
  Return:  distance squared from cylinder axis if point is inside.
-----------------------------------------------------------------------------
*/
float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy, dz;  // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.
  dy = pt2.y() - pt1.y();  // Make vector from pt1 to pt2. Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies
  // behind the cylinder cap at pt1.x, pt1.y, pt1.z
  dot = pdx * dx + pdy * dy + pdz * dz;
    // If dot is less than zero the point is behind the pt1 cap.
    // If greater than the cylinder axis line segment length squared
    // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
  /*
    Point lies within the parallel caps, so find distance squared from point to line,
    using the fact that sin^2 + cos^2 = 1 , the dot = cos() * |d||pd|,
    Carefull: '*' means mult for scalars and dotproduct for vectors 
    In short, where dist is pt distance to cyl axis: 
    dist = sin( pd to d ) * |pd| (from triangle, try to draw it!)
    dist^2 = dsq = (1 - cos^2( pd to d)) * |pd|^2 
    dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2 
    dsq = pd * pd - dot * dot / lengthsq
    where lengthsq is d*d or |d|^2 that is passed into this function
    distance squared to the cylinder axis:
  */
    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return distance squared to axis
    }
  }
}

bool collisionLine_and_traversable(octomap::OcTree *octree, Vector3d p1, Vector3d p2, double r){
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r, std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r, std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  octomap::point3d query(p2[0], p2[1], p2[2]);
  octomap::OcTreeNode* octree_res = octree->search(query);
  if (!octree_res) // if unknown, treat as occupied
  {
    return true;
  }
  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max); it != octree->end_leafs_bbx(); ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    // if (it->getLogOdds() > 0) // occupied, or to be occupied
    if (octree->isNodeOccupied(*it)) // occupied
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
    else //free
    {
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        octomap::point3d end_point;
        if (!octree->castRay(pt, octomap::point3d(0, 0, -1), end_point, false, 0.8)){ //free or unknown
          return true;
        }
      }
    }
  }
  return false;
}

bool collisionLine(octomap::OcTree *octree, Vector3d p1, Vector3d p2, double r){
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r, std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r, std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  // octomap::point3d p2_g;
  // octomap::point3d query(p2[0], p2[1], p1[2]);
  octomap::point3d query2(p2[0], p2[1], p2[2]);
    // if(octree->castRay(end, octomap::point3d(0, 0, -1), p2_g)){
    //   // query.z() = p2_g.z() + 0.2;
    //   if( p1[2] - p2_g.z() > 0.2)
    //     return true;
    // }
    // else
    //   return true;

  octomap::OcTreeNode* ot_res = octree->search(query2);
  if (!ot_res)
  {
    return true;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max); it != octree->end_leafs_bbx(); ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    // if (octree->isNodeOccupied(*it)) // occupied
    if (it->getLogOdds() > 0) // occupied, or to be occupied
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
  }
  return false;
}


#endif
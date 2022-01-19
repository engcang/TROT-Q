#ifndef REAL_IROS_H
#define REAL_IROS_H

#include "utility.h"
#include "peacock.h"

///// common headers for depth_to_pcl/octo and local_planner
#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <unsupported/Eigen/CXX11/Tensor> // for Tensor!!
#include <iostream> //cout
#include <fstream> // ofstream, file writing
#include <math.h> // pow
#include <vector>
#include <chrono> 
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// headers for local_planner + controller
#include <std_msgs/Bool.h>
#include <tf2_msgs/TFMessage.h> //for tf between frames
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

///// headers for pcl + octo
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////


class trot_q_class{
  public:

    std::string m_pcl_topic, m_depth_topic, m_pcl_base, m_depth_base, m_body_base, m_fixed_frame, m_octomap_topic; 
    bool m_depth_check=false, m_pcl_check=false, m_tf_check=false, m_octo_check=false, m_body_t_cam_check=false, m_body_t_lidar_check=false;

    ///// octomap
    double m_pcl_max_range=0.0, m_f_x, m_f_y, m_c_x, m_c_y, m_hfov;
    pcl::PointCloud<pcl::PointXYZ> m_depth_cvt_pcl, m_depth_cvt_pcl_empty, m_pcl_in;
    octomap::OcTree *m_octree;
    double m_octomap_resolution, m_octomap_hit, m_octomap_miss, m_octomap_hz;
    double m_collision_r;
    int m_search_depth;

    ///// local planner
    double m_unknown_score_local, m_free_score_local;
    bool m_new_path=true, m_score_debug;

    //// states
    Matrix4f m_map_t_lidar = Matrix4f::Identity();
    Matrix4f m_map_t_cam = Matrix4f::Identity();
    Matrix4f m_map_t_body = Matrix4f::Identity();
    Matrix4f m_body_t_lidar = Matrix4f::Identity();
    Matrix4f m_body_t_cam = Matrix4f::Identity();
    double m_curr_roll=0.0, m_curr_pitch=0.0, m_curr_yaw=0.0, m_cvt_quat_x=0.0, m_cvt_quat_y=0.0, m_cvt_quat_z=0.0, m_cvt_quat_w=1.0;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber m_pcl_sub;
    ros::Subscriber m_depth_sub;
    ros::Subscriber m_tf_sub;
    ros::Subscriber m_new_path_sub;
    ros::Publisher m_octomap_pub, m_octomap_empty_pub;
    ros::Publisher m_best_branch_pub;
    ros::Publisher m_local_branch_pub;
    ros::Publisher m_local_branch_pub2;
    ros::Publisher m_local_branch_pub10;
    ros::Publisher m_local_branch_pub11;
    ros::Publisher m_local_branch_pub12;
    ros::Publisher m_local_branch_pub13;
    ros::Publisher m_local_branch_pub14;
    ros::Publisher m_local_branch_pub15;
    ros::Publisher m_local_branch_pub16;
    ros::Publisher m_contact_points_pub;

    ros::Timer m_octo_update_and_publisher;
    ros::Timer m_local_planner_thread;

    void depth_callback(const sensor_msgs::Image::ConstPtr& msg);
    void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void new_path_callback(const std_msgs::Bool::ConstPtr& msg);
    void octomap_Timer(const ros::TimerEvent& event);
    void local_planner_Timer(const ros::TimerEvent& event);

    trot_q_class(ros::NodeHandle& n) : nh(n){
      ROS_WARN("Class generating...");
      ///// params
      nh.param("/pcl_max_range", m_pcl_max_range, 5.0);
      nh.param("/hfov", m_hfov, 1.5708);
      nh.param("/f_x", m_f_x, 320.0);
      nh.param("/f_y", m_f_y, 320.0);
      nh.param("/c_x", m_c_x, 320.5);
      nh.param("/c_y", m_c_y, 240.5);

      nh.param<std::string>("/pcl_topic", m_pcl_topic, "/velodyne_points");
      nh.param<std::string>("/depth_topic", m_depth_topic, "/d455/depth/image_raw");
      nh.param<std::string>("/pcl_base", m_pcl_base, "velodyne_base_link");
      nh.param<std::string>("/depth_base", m_depth_base, "camera_link");
      nh.param<std::string>("/body_base", m_body_base, "body_base");
      nh.param<std::string>("/fixed_frame", m_fixed_frame, "map");

      nh.param("/octomap_resolution", m_octomap_resolution, 0.4);
      nh.param("/octomap_hit", m_octomap_hit, 0.65);
      nh.param("/octomap_miss", m_octomap_miss, 0.3);
      nh.param("/octomap_hz", m_octomap_hz, 6.0);
      nh.param("/search_depth", m_search_depth, 0);
      nh.param<std::string>("/octomap_topic", m_octomap_topic, "/octomap");

      nh.param("/score_debug", m_score_debug, false);

      nh.param("/unknown_score_local", m_unknown_score_local, 0.11);
      nh.param("/free_score_local", m_free_score_local, 0.1);

      nh.param("/collision_r", m_collision_r, 0.2);

      ///// sub pub
      m_depth_sub = nh.subscribe<sensor_msgs::Image>(m_depth_topic, 10, &trot_q_class::depth_callback, this);
      m_pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>(m_pcl_topic, 10, &trot_q_class::pcl_callback, this);
      m_tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 10, &trot_q_class::tf_callback, this);
      m_new_path_sub = nh.subscribe<std_msgs::Bool>("/new_path", 10, &trot_q_class::new_path_callback, this);

      m_octomap_pub = nh.advertise<sensor_msgs::PointCloud2>(m_octomap_topic, 10);
      m_octomap_empty_pub = nh.advertise<sensor_msgs::PointCloud2>(m_octomap_topic+"_empty", 10);
      m_contact_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/contact_points", 10);

      m_octo_update_and_publisher = nh.createTimer(ros::Duration(1/m_octomap_hz), &trot_q_class::octomap_Timer, this); // every hz
      m_local_planner_thread = nh.createTimer(ros::Duration(1/20.0), &trot_q_class::local_planner_Timer, this); // every 1/20 second.

      m_best_branch_pub = nh.advertise<nav_msgs::Path>("/best_path", 10);
      m_local_branch_pub = nh.advertise<visualization_msgs::Marker>("/local_planner_1", 150);
      m_local_branch_pub2 = nh.advertise<visualization_msgs::Marker>("/local_planner_2", 150);
      m_local_branch_pub10 = nh.advertise<visualization_msgs::Marker>("/local_planner_10", 150);
      m_local_branch_pub11 = nh.advertise<visualization_msgs::Marker>("/local_planner_11", 150);
      m_local_branch_pub12 = nh.advertise<visualization_msgs::Marker>("/local_planner_12", 150);
      m_local_branch_pub13 = nh.advertise<visualization_msgs::Marker>("/local_planner_13", 150);
      m_local_branch_pub14 = nh.advertise<visualization_msgs::Marker>("/local_planner_14", 150);
      m_local_branch_pub15 = nh.advertise<visualization_msgs::Marker>("/local_planner_15", 150);
      m_local_branch_pub16 = nh.advertise<visualization_msgs::Marker>("/local_planner_16", 150);

      ///// octomap
      m_octree = new octomap::OcTree(m_octomap_resolution);
      m_octree->setProbHit(m_octomap_hit);
      m_octree->setProbMiss(m_octomap_miss);

      ///// local_planner - minimum jerk trajectories are initialized
      minimum_jerk_trajectories_initialize();
      ROS_WARN("Minimum jerk peacock generated...");
      ROS_WARN("Class heritated, starting node...");
    }
};








/////////////////////////////////////////// can be separated into .cpp source file from here

void trot_q_class::new_path_callback(const std_msgs::Bool::ConstPtr& msg){
  m_new_path=true;
}

void trot_q_class::depth_callback(const sensor_msgs::Image::ConstPtr& msg){
  sensor_msgs::Image depth=*msg;

  try {
    pcl::PointXYZ p3d, p3d_empty;
    // tic(); 
    m_depth_cvt_pcl.clear();
    m_depth_cvt_pcl_empty.clear();
    if (depth.encoding=="32FC1"){
      cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth, "32FC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
      for (int i=0; i<depth_ptr->image.rows; i++){
        for (int j=0; j<depth_ptr->image.cols; j++){
          float temp_depth = depth_ptr->image.at<float>(i,j); //float!!! double makes error here!!! because encoding is "32FC", float
          if (std::isnan(temp_depth)){
            p3d_empty.z = m_pcl_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*m_hfov/2.0);
            p3d_empty.x = ( j - m_c_x ) * p3d_empty.z / m_f_x;
            p3d_empty.y = ( i - m_c_y ) * p3d_empty.z / m_f_y;
            m_depth_cvt_pcl_empty.push_back(p3d_empty);
          }
          else if (temp_depth >= 0.1 and temp_depth <= m_pcl_max_range){
            p3d.z = temp_depth; 
            p3d.x = ( j - m_c_x ) * p3d.z / m_f_x;
            p3d.y = ( i - m_c_y ) * p3d.z / m_f_y;
            m_depth_cvt_pcl.push_back(p3d);
          }
        }
      }
    }
    else if (depth.encoding=="16UC1"){ // uint16_t (stdint.h) or ushort or unsigned_short
      cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth, "16UC1"); // == sensor_msgs::image_encodings::TYPE_16UC1
      for (int i=0; i<depth_ptr->image.rows; i++){
        for (int j=0; j<depth_ptr->image.cols; j++){
          float temp_depth = depth_ptr->image.at<ushort>(i,j); //ushort!!! other makes error here!!! because encoding is "16UC"
          if (std::isnan(temp_depth)){
            p3d_empty.z = m_pcl_max_range * cos(abs(depth_ptr->image.cols/2.0 - j)/(depth_ptr->image.cols/2.0)*m_hfov/2.0);
            p3d_empty.x = ( j - m_c_x ) * p3d_empty.z / m_f_x;
            p3d_empty.y = ( i - m_c_y ) * p3d_empty.z / m_f_y;
            m_depth_cvt_pcl_empty.push_back(p3d_empty);
          }
          else if (temp_depth/1000.0 >= 0.1 and temp_depth/1000.0 <= m_pcl_max_range){
            p3d.z = (temp_depth/1000.0); 
            p3d.x = ( j - m_c_x ) * p3d.z / m_f_x;
            p3d.y = ( i - m_c_y ) * p3d.z / m_f_y;
            m_depth_cvt_pcl.push_back(p3d);
          }
        }
      }
    }
    m_depth_check=true;
    // toc();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error to cvt depth img");
    return;
  }
}

void trot_q_class::pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  m_pcl_in.clear();
  m_pcl_in = cloudmsg2cloud(*msg);
  m_pcl_check=true;
}

void trot_q_class::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  for (int l=0; l < msg->transforms.size(); l++){
    if (msg->transforms[l].header.frame_id==m_fixed_frame && msg->transforms[l].child_frame_id==m_body_base){
      ///// for tf between map and body
      tf::Quaternion q(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m(q);
      m_map_t_body(0,0) = m[0][0];
      m_map_t_body(0,1) = m[0][1];
      m_map_t_body(0,2) = m[0][2];
      m_map_t_body(1,0) = m[1][0];
      m_map_t_body(1,1) = m[1][1];
      m_map_t_body(1,2) = m[1][2];
      m_map_t_body(2,0) = m[2][0];
      m_map_t_body(2,1) = m[2][1];
      m_map_t_body(2,2) = m[2][2];

      m_map_t_body(0,3) = msg->transforms[l].transform.translation.x;
      m_map_t_body(1,3) = msg->transforms[l].transform.translation.y;
      m_map_t_body(2,3) = msg->transforms[l].transform.translation.z;
      m_map_t_body(3,3) = 1.0;
      m.getRPY(m_curr_roll, m_curr_pitch, m_curr_yaw);

      m_cvt_quat_x = q.getX();
      m_cvt_quat_y = q.getY();
      m_cvt_quat_z = q.getZ();
      m_cvt_quat_w = q.getW();
    }
    if (msg->transforms[l].child_frame_id==m_pcl_base && !m_body_t_lidar_check){
      tf::Quaternion q2(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m2(q2);
      m_body_t_lidar(0,0) = m2[0][0];
      m_body_t_lidar(0,1) = m2[0][1];
      m_body_t_lidar(0,2) = m2[0][2];
      m_body_t_lidar(1,0) = m2[1][0];
      m_body_t_lidar(1,1) = m2[1][1];
      m_body_t_lidar(1,2) = m2[1][2];
      m_body_t_lidar(2,0) = m2[2][0];
      m_body_t_lidar(2,1) = m2[2][1];
      m_body_t_lidar(2,2) = m2[2][2];

      m_body_t_lidar(0,3) = msg->transforms[l].transform.translation.x;
      m_body_t_lidar(1,3) = msg->transforms[l].transform.translation.y;
      m_body_t_lidar(2,3) = msg->transforms[l].transform.translation.z;
      m_body_t_lidar(3,3) = 1.0;

      m_body_t_lidar_check = true; // fixed!!!
    }
    if (msg->transforms[l].child_frame_id==m_depth_base && !m_body_t_cam_check){
      tf::Quaternion q2(msg->transforms[l].transform.rotation.x, msg->transforms[l].transform.rotation.y, msg->transforms[l].transform.rotation.z, msg->transforms[l].transform.rotation.w);
      tf::Matrix3x3 m2(q2);
      m_body_t_cam(0,0) = m2[0][0];
      m_body_t_cam(0,1) = m2[0][1];
      m_body_t_cam(0,2) = m2[0][2];
      m_body_t_cam(1,0) = m2[1][0];
      m_body_t_cam(1,1) = m2[1][1];
      m_body_t_cam(1,2) = m2[1][2];
      m_body_t_cam(2,0) = m2[2][0];
      m_body_t_cam(2,1) = m2[2][1];
      m_body_t_cam(2,2) = m2[2][2];

      m_body_t_cam(0,3) = msg->transforms[l].transform.translation.x;
      m_body_t_cam(1,3) = msg->transforms[l].transform.translation.y;
      m_body_t_cam(2,3) = msg->transforms[l].transform.translation.z;
      m_body_t_cam(3,3) = 1.0;

      m_body_t_cam_check = true; // fixed!!!
    }
  }
  
  m_map_t_lidar = m_map_t_body * m_body_t_lidar ;
  m_map_t_cam = m_map_t_body * m_body_t_cam;
  if (m_body_t_cam_check && m_body_t_lidar_check)
    m_tf_check=true;
}


void trot_q_class::octomap_Timer(const ros::TimerEvent& event){
  if (m_pcl_check && m_depth_check && m_tf_check){

    pcl::PointCloud<pcl::PointXYZ> depth_cvt_pcl_map, depth_cvt_pcl_map_empty, pcl_in_map;

    pcl::transformPointCloud(m_depth_cvt_pcl, depth_cvt_pcl_map, m_map_t_cam); //essential for agg_pcl and octomap(based on world naturally!)
    pcl::transformPointCloud(m_depth_cvt_pcl_empty, depth_cvt_pcl_map_empty, m_map_t_cam); //essential for agg_pcl and octomap(based on world naturally!)
    pcl::transformPointCloud(m_pcl_in, pcl_in_map, m_map_t_lidar); //essential for octomap(based on world naturally!)

    /////// octomap update with RGB-D empty cell within sensor range
    octomap::KeySet free_cells_keyset;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = depth_cvt_pcl_map_empty.begin(); it!=depth_cvt_pcl_map_empty.end(); ++it){
      octomap::KeyRay ray;
      m_octree->computeRayKeys(octomap::point3d(m_map_t_cam(0,3),m_map_t_cam(1,3),m_map_t_cam(2,3)), octomap::point3d(it->x, it->y, it->z), ray);
      free_cells_keyset.insert(ray.begin(), ray.end());
    } 
    for (octomap::KeySet::iterator it = free_cells_keyset.begin(); it!=free_cells_keyset.end(); it++){
      m_octree->updateNode(*it, false); // set as free
    }

    /////// octomap updating with RGB-D occupied cell
    octomap::Pointcloud temp_pcl2; 
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = depth_cvt_pcl_map.begin(); it!=depth_cvt_pcl_map.end(); ++it){
      temp_pcl2.push_back(it->x, it->y, it->z);
    } 
    m_octree->insertPointCloud(temp_pcl2, octomap::point3d(m_map_t_cam(0,3),m_map_t_cam(1,3),m_map_t_cam(2,3))); 

    /////// octomap updating with RGB-D occupied cell
    octomap::Pointcloud temp_pcl; 
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pcl_in_map.begin(); it!=pcl_in_map.end(); ++it){
      temp_pcl.push_back(it->x, it->y, it->z);
    } 
    m_octree->insertPointCloud(temp_pcl, octomap::point3d(m_map_t_lidar(0,3),m_map_t_lidar(1,3),m_map_t_lidar(2,3))); 
    // octomap is originally based on world(0,0,0)!!! ray should be from sensing robot, or occupancy will be calculated from the point to sensing, wrong occupancy
    // does not remove using rays, but just stack

    pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr octo_pcl_empty_pub(new pcl::PointCloud<pcl::PointXYZ>());
    for (octomap::OcTree::iterator it=m_octree->begin(); it!=m_octree->end(); ++it){
      if(m_octree->isNodeOccupied(*it))
      {
        // if (it.getCoordinate().z()<2.2)
        octo_pcl_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
      }
      else
      {
        octo_pcl_empty_pub->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        // free_oc++;
      }
    }
    m_octo_check=true;
    m_octomap_pub.publish(cloud2msg(*octo_pcl_pub, m_fixed_frame));
    m_octomap_empty_pub.publish(cloud2msg(*octo_pcl_empty_pub, m_fixed_frame));
  }
}


void trot_q_class::local_planner_Timer(const ros::TimerEvent& event){
  if (m_octo_check && local_init){
    // tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr contact_pcl_pub(new pcl::PointCloud<pcl::PointXYZ>());
    int id = 1;
    int sid = 1000;
    bool big_branch_ok = false;
    loc_score = MatrixXd::Zero(row,col); 
    for (int i=0; i < row; i++){
      for (int j=0; j < col; j++){
        visualization_msgs::Marker path;
        path.ns = "loc";
        path.header.frame_id = m_fixed_frame;
        path.type=4;
        path.scale.x = 0.01; //width
        path.id = id;
        path.color.r = 1.0;
        path.color.b = 1.0;
        path.color.a = 1.0;
        path.lifetime = ros::Duration(1/20.0);
        path.pose.position.x = m_map_t_body(0,3);
        path.pose.position.y = m_map_t_body(1,3);
        path.pose.position.z = m_map_t_body(2,3);
        path.pose.orientation.x = m_cvt_quat_x;
        path.pose.orientation.y = m_cvt_quat_y;
        path.pose.orientation.z = m_cvt_quat_z;
        path.pose.orientation.w = m_cvt_quat_w;
        // path.header.stamp = ros::Time::now();

        float t = 0.4*T;
        geometry_msgs::Point p;
        p.x = cx(i,j,0)*pow(t,5) + cx(i,j,1)*pow(t,4) + cx(i,j,2)*pow(t,3) + cx(i,j,3)*pow(t,2) + cx(i,j,4)*t + cx(i,j,5);
        p.y = cy(i,j,0)*pow(t,5) + cy(i,j,1)*pow(t,4) + cy(i,j,2)*pow(t,3) + cy(i,j,3)*pow(t,2) + cy(i,j,4)*t + cy(i,j,5);
        p.z = cz(i,0)*pow(t,5) + cz(i,1)*pow(t,4) + cz(i,2)*pow(t,3) + cz(i,3)*pow(t,2) + cz(i,4)*t + cz(i,5);
        geometry_msgs::Point tf_p = tf_point(p, m_map_t_body);
        Vector3d p1(tf_p.x, tf_p.y, tf_p.z);

        t = T;
        p.x = cx(i,j,0)*pow(t,5) + cx(i,j,1)*pow(t,4) + cx(i,j,2)*pow(t,3) + cx(i,j,3)*pow(t,2) + cx(i,j,4)*t + cx(i,j,5);
        p.y = cy(i,j,0)*pow(t,5) + cy(i,j,1)*pow(t,4) + cy(i,j,2)*pow(t,3) + cy(i,j,3)*pow(t,2) + cy(i,j,4)*t + cy(i,j,5);
        p.z = cz(i,0)*pow(t,5) + cz(i,1)*pow(t,4) + cz(i,2)*pow(t,3) + cz(i,3)*pow(t,2) + cz(i,4)*t + cz(i,5);
        tf_p = tf_point(p, m_map_t_body);
        Vector3d p2(tf_p.x, tf_p.y, tf_p.z);
        big_branch_ok = !(collisionLine_and_traversable(m_octree, p1, p2, m_collision_r));

        if(big_branch_ok){
          for (double t=0.1*T; t<= T; t+=0.1*T){
            geometry_msgs::Point p;
            p.x = cx(i,j,0)*pow(t,5) + cx(i,j,1)*pow(t,4) + cx(i,j,2)*pow(t,3) + cx(i,j,3)*pow(t,2) + cx(i,j,4)*t + cx(i,j,5);
            p.y = cy(i,j,0)*pow(t,5) + cy(i,j,1)*pow(t,4) + cy(i,j,2)*pow(t,3) + cy(i,j,3)*pow(t,2) + cy(i,j,4)*t + cy(i,j,5);
            p.z = cz(i,0)*pow(t,5) + cz(i,1)*pow(t,4) + cz(i,2)*pow(t,3) + cz(i,3)*pow(t,2) + cz(i,4)*t + cz(i,5);

            geometry_msgs::Point tf_p = tf_point(p, m_map_t_body);
            path.points.push_back(p);
          }

          if (id%2==0) m_local_branch_pub.publish(path); else m_local_branch_pub2.publish(path);
          id++;

          bool s_branch_ok = false;
          for (int b=0; b < branch; b++){
            visualization_msgs::Marker spath;
            spath.ns = "s_loc";
            spath.header.frame_id = m_fixed_frame;
            spath.type=4;
            spath.scale.x = 0.01; //width
            spath.id = sid;
            spath.color.g = 1.0;
            spath.color.b = 1.0;
            spath.color.a = 1.0;
            spath.lifetime = ros::Duration(1/20.0);
            spath.pose.position.x = m_map_t_body(0,3);
            spath.pose.position.y = m_map_t_body(1,3);
            spath.pose.position.z = m_map_t_body(2,3);
            spath.pose.orientation.x = m_cvt_quat_x;
            spath.pose.orientation.y = m_cvt_quat_y;
            spath.pose.orientation.z = m_cvt_quat_z;
            spath.pose.orientation.w = m_cvt_quat_w;
            // spath.header.stamp = ros::Time::now();

            for (double t=0.0; t<= T; t+=0.1*T){
              geometry_msgs::Point p;
              p.x = scx(i,j,b,0)*pow(t,5) + scx(i,j,b,1)*pow(t,4) + scx(i,j,b,2)*pow(t,3) + scx(i,j,b,3)*pow(t,2) + scx(i,j,b,4)*t + scx(i,j,b,5);
              p.y = scy(i,j,b,0)*pow(t,5) + scy(i,j,b,1)*pow(t,4) + scy(i,j,b,2)*pow(t,3) + scy(i,j,b,3)*pow(t,2) + scy(i,j,b,4)*t + scy(i,j,b,5);
              p.z = scz(i,0)*pow(t,5) + scz(i,1)*pow(t,4) + scz(i,2)*pow(t,3) + scz(i,3)*pow(t,2) + scz(i,4)*t + scz(i,5);

              geometry_msgs::Point tf_p = tf_point(p, m_map_t_body);

              auto *ptr = m_octree->search(tf_p.x,tf_p.y,tf_p.z,m_search_depth); //x,y,z,depth=0(full)
              if (ptr){ // if not NULL : searched before
                if (m_octree->isNodeOccupied(ptr)) // occupied
                {
                  s_branch_ok=false;
                  break;
                }
                else{ // not occupied -> free space
                  octomap::point3d end_point;
                  if (m_octree->castRay(octomap::point3d(tf_p.x, tf_p.y, tf_p.z), octomap::point3d(0, 0, -1), end_point, false, 0.8)){
                    loc_score(i,j)+=m_free_score_local;
                    spath.points.push_back(p);
                    s_branch_ok=true;
                    contact_pcl_pub->push_back(pcl::PointXYZ(end_point.x(), end_point.y(), end_point.z()+m_octomap_resolution));
                  }
                  else{
                    s_branch_ok=false;
                    break;
                  }
                }
              }
              else{ // not searched yet -> unknown space
                octomap::point3d end_point;
                if (m_octree->castRay(octomap::point3d(tf_p.x, tf_p.y, tf_p.z), octomap::point3d(0, 0, -1), end_point, false, 0.8)){
                  loc_score(i,j)+=m_unknown_score_local;
                  spath.points.push_back(p);
                  s_branch_ok=true;
                  contact_pcl_pub->push_back(pcl::PointXYZ(end_point.x(), end_point.y(), end_point.z()+m_octomap_resolution));
                }
                else{
                  s_branch_ok=false;
                  break;
                }
              }
            }
            if (s_branch_ok){
              if (sid<1000+row*col)
                m_local_branch_pub10.publish(spath);
              else if(sid<1000+row*col*2)
                m_local_branch_pub11.publish(spath);
              else if(sid<1000+row*col*3)
                m_local_branch_pub12.publish(spath);
              else if(sid<1000+row*col*4)
                m_local_branch_pub13.publish(spath);
              else if(sid<1000+row*col*5)
                m_local_branch_pub14.publish(spath);
              else if(sid<1000+row*col*6)
                m_local_branch_pub15.publish(spath);
              else
                m_local_branch_pub16.publish(spath);
              sid++;
            }
          }
        }
      }
    }
    if (!contact_pcl_pub->empty())
      m_contact_points_pub.publish(cloud2msg(*contact_pcl_pub, m_fixed_frame));
    // toc("peacock");

    if(m_new_path){
      best_score_path(loc_score);
      if (m_score_debug) 
        {cout << max_score << " " << best_row << " " << best_col << endl<< loc_score << endl <<endl;}

      nav_msgs::Path path;
      path.header.frame_id = m_fixed_frame;
      geometry_msgs::PoseStamped p;
      for (double t=0.0; t<= T; t+=0.1*T){
        p.pose.position.x = cx(best_row,best_col,0)*pow(t,5) + cx(best_row,best_col,1)*pow(t,4) + cx(best_row,best_col,2)*pow(t,3) + cx(best_row,best_col,3)*pow(t,2) + cx(best_row,best_col,4)*t + cx(best_row,best_col,5);
        p.pose.position.y = cy(best_row,best_col,0)*pow(t,5) + cy(best_row,best_col,1)*pow(t,4) + cy(best_row,best_col,2)*pow(t,3) + cy(best_row,best_col,3)*pow(t,2) + cy(best_row,best_col,4)*t + cy(best_row,best_col,5);
        p.pose.position.z = cz(best_row,0)*pow(t,5) + cz(best_row,1)*pow(t,4) + cz(best_row,2)*pow(t,3) + cz(best_row,3)*pow(t,2) + cz(best_row,4)*t + cz(best_row,5);
        tf::Quaternion q;
        q.setRPY(0, 0, yaw(best_row,best_col)*t/T);
        p.pose.orientation.x = q.getX();
        p.pose.orientation.y = q.getY();
        p.pose.orientation.z = q.getZ();
        p.pose.orientation.w = q.getW();

        geometry_msgs::PoseStamped tf_p = tf_pose(p, m_map_t_body);
        path.poses.push_back(tf_p);
      }
      m_best_branch_pub.publish(path);
      m_new_path=false;
    }
  }
}





#endif
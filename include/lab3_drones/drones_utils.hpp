#ifndef DRONES_UTILS_HPP
#define DRONES_UTILS_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <numeric>  // Para std::accumulate y std::inner_product
#include <tf2/utils.h>  // Para tf2::getYaw

geometry_msgs::Quaternion RPYToQuat(double roll, double pitch, double yaw);
Eigen::Matrix<double, 3, 3> quatToRMatrix(const geometry_msgs::Quaternion& quat);
Eigen::Matrix<double, 3, 3> RPYtoRMatrix(double roll, double pitch, double yaw);
double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);

void calculateMetrics(
    const std::vector<nav_msgs::Odometry>& gt_poses,
    const std::vector<geometry_msgs::PoseStamped>& goal_list_,
    const std::vector<geometry_msgs::Twist>& goal_vel_list_,
    std::string yaw_control);

Eigen::Vector3d extractOrientationAsRPY(const geometry_msgs::PoseStamped& pose_stamped);
double angularDistance(double angle1, double angle2);
double normalizeAngle(double angle);

void normalizeVector(double& x, double& y, double& z);
geometry_msgs::Quaternion calculateQuaternionFromVector(double x, double y, double z);

#endif // UTILS_HPP
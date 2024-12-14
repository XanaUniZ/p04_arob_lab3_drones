#ifndef DRAWING_HPP
#define DRAWING_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Function declarations
void drawGates(const std::vector<geometry_msgs::Pose>& gates, ros::Publisher& pub_gate_markers);
void drawGateMarkers(geometry_msgs::Pose gate, int &id, ros::Publisher& pub_gate_markers);
void drawTrajectoryMarkers(const mav_trajectory_generation::Trajectory& trajectory, ros::Publisher& pub_traj_markers);

#endif // DRAWING_H
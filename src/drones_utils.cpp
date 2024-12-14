#include "lab3_drones/drones_utils.hpp"
#include <ros/ros.h>

geometry_msgs::Quaternion RPYToQuat(double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

Eigen::Matrix<double, 3, 3> quatToRMatrix(const geometry_msgs::Quaternion& quat) {
    Eigen::Quaterniond eigen_quat(quat.w, quat.x, quat.y, quat.z);
    return eigen_quat.toRotationMatrix();
}

Eigen::Matrix<double, 3, 3> RPYtoRMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
    Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
    Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

    Eigen::Matrix<double, 3, 3> R;

    Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

    R = q.matrix();

    return (R);
}


double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    q.normalize();
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

void calculateMetrics(
    const std::vector<nav_msgs::Odometry>& gt_poses,
    const std::vector<geometry_msgs::PoseStamped>& goal_list_,
    const std::vector<geometry_msgs::Twist>& goal_vel_list_)
{
    if (gt_poses.empty()) {
        ROS_WARN("Ground truth is empty. Cannot compute metrics.");
        return;
    }
    if (goal_list_.empty()) {
        ROS_WARN("Goal list is empty.");
        return;
    }

    std::vector<double> lowest_distances, angular_errors, velocity_errors;
    double lowest_dist = 100000; 
    int index = 0;

    for (int i = 0; i < gt_poses.size(); i++) {
        for (int j = 0; j < goal_list_.size(); j++) {
            double dist = sqrt(
                pow(gt_poses[i].pose.pose.position.x - goal_list_[j].pose.position.x, 2) +
                pow(gt_poses[i].pose.pose.position.y - goal_list_[j].pose.position.y, 2) +
                pow(gt_poses[i].pose.pose.position.z - goal_list_[j].pose.position.z, 2));

            if (dist < lowest_dist) {
                index = j;
                lowest_dist = dist;
            }
        }

        lowest_distances.push_back(lowest_dist);

        double velocity_error = sqrt(
            pow(gt_poses[i].twist.twist.linear.x - goal_vel_list_[index].linear.x, 2) +
            pow(gt_poses[i].twist.twist.linear.y - goal_vel_list_[index].linear.y, 2) +
            pow(gt_poses[i].twist.twist.linear.z - goal_vel_list_[index].linear.z, 2));

        velocity_errors.push_back(velocity_error);

        geometry_msgs::Quaternion gt_orientation = gt_poses[i].pose.pose.orientation;
        tf2::Quaternion q_gt;
        tf2::fromMsg(gt_orientation, q_gt);
        q_gt.normalize();

        tf2::Vector3 gt_velocity_world(
            gt_poses[i].twist.twist.linear.x, 
            gt_poses[i].twist.twist.linear.y, 
            gt_poses[i].twist.twist.linear.z);
        tf2::Vector3 gt_velocity_body = tf2::quatRotate(q_gt.inverse(), gt_velocity_world);

        geometry_msgs::Twist goal_velocity = goal_vel_list_[index];
        tf2::Vector3 goal_velocity_world(
            goal_velocity.linear.x, 
            goal_velocity.linear.y, 
            goal_velocity.linear.z);
        tf2::Vector3 goal_velocity_body = tf2::quatRotate(q_gt.inverse(), goal_velocity_world);

        double gt_yaw = atan2(gt_velocity_body.y(), gt_velocity_body.x());
        double goal_yaw = atan2(goal_velocity_body.y(), goal_velocity_body.x());

        double angular_error = gt_yaw - goal_yaw;
        angular_error = atan2(sin(angular_error), cos(angular_error));
        angular_errors.push_back(angular_error);
    }

    double sum_distance = 0.0, sum_distance_squared = 0.0;
    double sum_angular = 0.0, sum_angular_squared = 0.0;
    double sum_velocity = 0.0, sum_velocity_squared = 0.0;

    for (int i = 0; i < lowest_distances.size(); i++) {
        sum_distance += lowest_distances[i];
        sum_distance_squared += lowest_distances[i] * lowest_distances[i];
        sum_angular += angular_errors[i];
        sum_angular_squared += angular_errors[i] * angular_errors[i];
        sum_velocity += velocity_errors[i];
        sum_velocity_squared += velocity_errors[i] * velocity_errors[i];
    }

    double mean_distance = sum_distance / lowest_distances.size();
    double std_dev_distance = sqrt((sum_distance_squared / lowest_distances.size()) - (mean_distance * mean_distance));
    double mean_angular = sum_angular / angular_errors.size();
    double std_dev_angular = sqrt((sum_angular_squared / angular_errors.size()) - (mean_angular * mean_angular));
    double mean_velocity = sum_velocity / velocity_errors.size();
    double std_dev_velocity = sqrt((sum_velocity_squared / velocity_errors.size()) - (mean_velocity * mean_velocity));

    ROS_INFO("Distance Errors - Mean: %f, Std Dev: %f", mean_distance, std_dev_distance);
    ROS_INFO("Angular Errors - Mean: %f, Std Dev: %f", mean_angular, std_dev_angular);
    ROS_INFO("Velocity Errors - Mean: %f, Std Dev: %f", mean_velocity, std_dev_velocity);
}



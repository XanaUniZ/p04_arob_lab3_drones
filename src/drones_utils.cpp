#include "lab3_drones/drones_utils.hpp"
#include <ros/ros.h>

// #define M_PI 3.141592653589793238462643383279502884197

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

// void calculateMetrics(
//     const std::vector<nav_msgs::Odometry>& gt_poses,
//     const std::vector<geometry_msgs::PoseStamped>& goal_list_,
//     const std::vector<geometry_msgs::Twist>& goal_vel_list_)
// {
//     if (gt_poses.empty()) {
//         ROS_WARN("Ground truth is empty. Cannot compute metrics.");
//         return;
//     }
//     if (goal_list_.empty()) {
//         ROS_WARN("Goal list is empty.");
//         return;
//     }

//     std::vector<double> lowest_distances, angular_errors, velocity_errors;

//     std::cout << "gt_poses.size() = " << gt_poses.size() << std::endl;
//     std::cout << "goal_list_.size() = " << goal_list_.size() << std::endl;
//     for (std::size_t i = 0; i < gt_poses.size(); i++) {
//         if (i==1147){
//             std::cout << "gt_poses "<< i << " --> " << gt_poses[i].pose.pose.position.x << " " << gt_poses[i].pose.pose.position.y  << " " << gt_poses[i].pose.pose.position.z << std::endl;
//         }
//         double lowest_dist = 100000; 
//         std::size_t index = 0;
//         for (std::size_t j = 0; j < goal_list_.size(); j++) {
//             double dist = sqrt(
//                 pow(gt_poses[i].pose.pose.position.x - goal_list_[j].pose.position.x, 2) +
//                 pow(gt_poses[i].pose.pose.position.y - goal_list_[j].pose.position.y, 2) +
//                 pow(gt_poses[i].pose.pose.position.z - goal_list_[j].pose.position.z, 2));
            
//             if (i == 1147 && j > 700 && j < 1000){
//                 std::cout << "goal_list_ "<< j << " --> " << goal_list_[j].pose.position.x << " " << goal_list_[j].pose.position.y  << " " << goal_list_[j].pose.position.z << std::endl;
//                 std::cout << "dist = "<< dist << std::endl;
//             }
//             if (dist < lowest_dist) {
//                 index = j;
//                 lowest_dist = dist;

//                 // if (i == 1147){
//                 //     std::cout << "Updating min dist: " << dist << " Index: " << j  << std::endl;
//                 // }
//             }
//         }
//         // if (i > 1000 && i <1500 ){
//         if (i==1147 ){
//             std::cout << "gt index = " << i << " --> " << "goal_idx = " << index << " --> dist = " << lowest_dist << std::endl;
//         }
//         lowest_distances.push_back(lowest_dist);

//         double velocity_error = sqrt(
//             pow(gt_poses[i].twist.twist.linear.x - goal_vel_list_[index].linear.x, 2) +
//             pow(gt_poses[i].twist.twist.linear.y - goal_vel_list_[index].linear.y, 2) +
//             pow(gt_poses[i].twist.twist.linear.z - goal_vel_list_[index].linear.z, 2));

//         velocity_errors.push_back(velocity_error);


//         // Extract the orientation of the robot in the world frame
//         Eigen::Quaterniond robot_orientation(gt_poses[i].pose.pose.orientation.x,
//                                             gt_poses[i].pose.pose.orientation.y,
//                                             gt_poses[i].pose.pose.orientation.z,
//                                             gt_poses[i].pose.pose.orientation.w);
//         robot_orientation.normalize();

//         // Eigen::Vector3d goal_orientation_world = extractOrientationAsRPY(goal_list_[index]);
//         // Eigen::Vector3d gt_velocity_world(
//         //     gt_poses[i].twist.twist.linear.x, 
//         //     gt_poses[i].twist.twist.linear.y, 
//         //     gt_poses[i].twist.twist.linear.z);
//         // Eigen::Vector3d gt_velocity_body = robot_orientation.inverse() * gt_velocity_world;

//         geometry_msgs::Twist goal_velocity = goal_vel_list_[index];
//         Eigen::Vector3d goal_velocity_world(
//             goal_velocity.linear.x, 
//             goal_velocity.linear.y, 
//             goal_velocity.linear.z);
//         Eigen::Vector3d goal_velocity_body = robot_orientation.inverse() * goal_velocity_world;

//         // double gt_yaw = atan2(gt_velocity_body.y(), gt_velocity_body.x());
//         //double goal_yaw = atan2(goal_velocity_body.y(), goal_velocity_body.x());
//         // double gt_yaw = atan2(gt_poses[i].twist.twist.linear.y, gt_poses[i].twist.twist.linear.x);
//         //ROS_INFO("Ground Truth Yaw [%lu]: %f degrees", i, gt_yaw * 180 / M_PI);

//         // double goal_yaw = atan2(goal_vel_list_[i].linear.y, goal_vel_list_[i].linear.x);
//         //ROS_INFO("Trajectory Yaw [%lu]: %f degrees", i, goal_yaw * 180/M_PI);

//         if (i == 0) {
//             ROS_INFO("Initial Ground Truth Orientation: %f degrees", gt_yaw * 180 / M_PI);
//         }
//         double gt_yaw = atan2(gt_poses[i].twist.twist.linear.y, gt_poses[i].twist.twist.linear.x);

//         // Check for zero velocities in the trajectory
//         double goal_yaw;
//         if (fabs(goal_vel_list_[index].linear.x) > epsilon || fabs(goal_vel_list_[index].linear.y) > epsilon) {
//             goal_yaw = atan2(goal_vel_list_[index].linear.y, goal_vel_list_[index].linear.x);
//         } else {
//             ROS_WARN("Trajectory velocity near zero at index %lu, setting goal_yaw to 0", i);
//             goal_yaw = 0.0;
//         }

//         gt_yaw = normalizeAngle(gt_yaw);
//         goal_yaw = normalizeAngle(goal_yaw);

//         double angular_error = normalizeAngle(gt_yaw - goal_yaw);

//         ROS_INFO("Ground Truth Yaw [%lu]: %f degrees", i, gt_yaw * 180 / M_PI);
//         ROS_INFO("Trajectory Yaw [%lu]: %f degrees", i, goal_yaw * 180 / M_PI);
//         ROS_INFO("Angular Error [%lu]: %f degrees", i, angular_error * 180 / M_PI);





//         //angular_errors.push_back(angularDistance(0., goal_yaw));


//         // geometry_msgs::Quaternion gt_orientation = gt_poses[i].pose.pose.orientation;
//         // tf2::Quaternion q_gt;
//         // tf2::fromMsg(gt_orientation, q_gt);
//         // q_gt.normalize();

//         // tf2::Vector3 gt_velocity_world(
//         //     gt_poses[i].twist.twist.linear.x, 
//         //     gt_poses[i].twist.twist.linear.y, 
//         //     gt_poses[i].twist.twist.linear.z);
//         // tf2::Vector3 gt_velocity_body = tf2::quatRotate(q_gt.inverse(), gt_velocity_world);

//         // geometry_msgs::Twist goal_velocity = goal_vel_list_[index];
//         // tf2::Vector3 goal_velocity_world(
//         //     goal_velocity.linear.x, 
//         //     goal_velocity.linear.y, 
//         //     goal_velocity.linear.z);
//         // tf2::Vector3 goal_velocity_body = tf2::quatRotate(q_gt.inverse(), goal_velocity_world);

//         // double gt_yaw = atan2(gt_velocity_body.y(), gt_velocity_body.x());
//         // double goal_yaw = atan2(goal_velocity_body.y(), goal_velocity_body.x());

//         // double angular_error = gt_yaw - goal_yaw;
//         // angular_error = atan2(sin(angular_error), cos(angular_error));
//         // angular_errors.push_back(angular_error);
//     }

//     double sum_distance = 0.0, sum_distance_squared = 0.0;
//     double sum_angular = 0.0, sum_angular_squared = 0.0;
//     double sum_velocity = 0.0, sum_velocity_squared = 0.0;

//     for (int i = 0; i < lowest_distances.size(); i++) {
//         sum_distance += lowest_distances[i];
//         sum_distance_squared += lowest_distances[i] * lowest_distances[i];
//         sum_angular += angular_errors[i];
//         sum_angular_squared += angular_errors[i] * angular_errors[i];
//         sum_velocity += velocity_errors[i];
//         sum_velocity_squared += velocity_errors[i] * velocity_errors[i];
//     }

//     double mean_distance = sum_distance / lowest_distances.size();
//     double std_dev_distance = sqrt((sum_distance_squared / lowest_distances.size()) - (mean_distance * mean_distance));
//     double mean_angular = sum_angular / angular_errors.size();
//     double std_dev_angular = sqrt((sum_angular_squared / angular_errors.size()) - (mean_angular * mean_angular));
//     double mean_velocity = sum_velocity / velocity_errors.size();
//     double std_dev_velocity = sqrt((sum_velocity_squared / velocity_errors.size()) - (mean_velocity * mean_velocity));

//     ROS_INFO("Distance Errors - Mean: %f, Std Dev: %f", mean_distance, std_dev_distance);
//     ROS_INFO("Angular Errors - Mean: %f, Std Dev: %f", mean_angular, std_dev_angular);
//     ROS_INFO("Angular Errors Degrees - Mean: %f, Std Dev: %f", mean_angular*180/M_PI, std_dev_angular*180/M_PI);
//     ROS_INFO("Velocity Errors - Mean: %f, Std Dev: %f", mean_velocity, std_dev_velocity);
// }

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
    const double epsilon = 1e-6;

    std::cout << "gt_poses.size() = " << gt_poses.size() << std::endl;
    std::cout << "goal_list_.size() = " << goal_list_.size() << std::endl;

    for (std::size_t i = 0; i < gt_poses.size(); i++) {
        double lowest_dist = std::numeric_limits<double>::max();
        std::size_t index = 0;  

        // Find the closest trajectory point
        for (std::size_t j = 0; j < goal_list_.size(); j++) {
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

        // Calculate velocity error
        double velocity_error = sqrt(
            pow(gt_poses[i].twist.twist.linear.x - goal_vel_list_[index].linear.x, 2) +
            pow(gt_poses[i].twist.twist.linear.y - goal_vel_list_[index].linear.y, 2) +
            pow(gt_poses[i].twist.twist.linear.z - goal_vel_list_[index].linear.z, 2));

        velocity_errors.push_back(velocity_error);

        // Calculate ground truth yaw in the global frame
        double gt_yaw = atan2(gt_poses[i].twist.twist.linear.y, gt_poses[i].twist.twist.linear.x);
        static double last_valid_goal_yaw = 0.0;  // Keep track of the last valid goal_yaw

        // Check for zero velocities in the trajectory
        double goal_yaw;
        if (fabs(goal_vel_list_[index].linear.x) > epsilon || fabs(goal_vel_list_[index].linear.y) > epsilon) {
            goal_yaw = atan2(goal_vel_list_[index].linear.y, goal_vel_list_[index].linear.x);
            last_valid_goal_yaw = goal_yaw;  // Update last valid goal_yaw

        } else {
            ROS_WARN("Trajectory velocity near zero at index %lu, setting goal_yaw to 0", i);
            goal_yaw = last_valid_goal_yaw;
        }

        // Normalize yaw angles to the range [-π, π]
        gt_yaw = normalizeAngle(gt_yaw);
        goal_yaw = normalizeAngle(goal_yaw);

        // Calculate angular error
        double angular_error = normalizeAngle(gt_yaw - goal_yaw);

        // Logging for verification
        ROS_INFO("Ground Truth Yaw [%lu]: %f degrees", i, gt_yaw * 180 / M_PI);
        ROS_INFO("Trajectory Yaw [%lu]: %f degrees", i, goal_yaw * 180 / M_PI);
        ROS_INFO("Angular Error [%lu]: %f degrees", i, angular_error * 180 / M_PI);

        angular_errors.push_back(fabs(angular_error));
    }

    // Compute mean and standard deviation for each metric
    double sum_distance = 0.0, sum_distance_squared = 0.0;
    double sum_angular = 0.0, sum_angular_squared = 0.0;
    double sum_velocity = 0.0, sum_velocity_squared = 0.0;

    for (size_t i = 0; i < lowest_distances.size(); i++) {
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

    // Final metrics output
    ROS_INFO("Distance Errors - Mean: %f, Std Dev: %f", mean_distance, std_dev_distance);
    ROS_INFO("Angular Errors - Mean: %f, Std Dev: %f", mean_angular, std_dev_angular);
    ROS_INFO("Angular Errors Degrees - Mean: %f, Std Dev: %f", mean_angular * 180 / M_PI, std_dev_angular * 180 / M_PI);
    ROS_INFO("Velocity Errors - Mean: %f, Std Dev: %f", mean_velocity, std_dev_velocity);
}


double angularDistance(double angle1, double angle2) {
    double diff = angle2 - angle1;
    // Normalize the angle to the range [-pi, pi]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return std::fabs(diff);
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

Eigen::Vector3d extractOrientationAsRPY(const geometry_msgs::PoseStamped& pose_stamped) {
    // Extract the quaternion from the PoseStamped message
    const auto& q = pose_stamped.pose.orientation;

    // Convert the ROS quaternion to a tf2::Quaternion
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);

    // Convert the quaternion to roll, pitch, and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // Return as an Eigen::Vector3d
    return Eigen::Vector3d(roll, pitch, yaw);
}


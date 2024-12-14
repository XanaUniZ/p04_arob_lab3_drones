#include "lab3_drones/drawing.hpp"
#include "lab3_drones/drones_utils.hpp"
#include <ros/ros.h>

void drawGates(const std::vector<geometry_msgs::Pose>& gates, ros::Publisher& pub_gate_markers) {
    int id = 0;
    for (geometry_msgs::Pose gate : gates) {
        drawGateMarkers(gate,id,pub_gate_markers);
    }
}

void drawGateMarkers(geometry_msgs::Pose gate, int &id, ros::Publisher& pub_gate_markers){
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker line_marker;
    //std::vector<visualization_msgs::Marker> line_marker_vector;
    
    Eigen::Matrix<double, 3, 3> rotate_gate = quatToRMatrix(gate.orientation);
    Eigen::Matrix<double, 3, 1> pos_gate(gate.position.x, gate.position.y, gate.position.z);

    marker.header.frame_id = "world";  // Change this frame_id according to your setup
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "corner";
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration();

    line_marker.header.frame_id = "world";  // Change this frame_id according to your setup
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line";
    line_marker.id = id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.05;  // Line width
    line_marker.pose.orientation.w = 1.0;
    line_marker.lifetime = ros::Duration();

    // Set the color (green in this case)
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    float gate_size = 0.75;

    //Generate the gate corners and edges
    Eigen::Matrix<double, 3, 1> move_gate;
    move_gate << 0.0, gate_size, gate_size;
    Eigen::Matrix<double, 3, 1> position2 = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position2(0);
    marker.pose.position.y = position2(1);
    marker.pose.position.z = position2(2);
    // Change left gate vertex color to yellow
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // Change shape to sphere radius 25cm
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.id = id + 1;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, -gate_size, gate_size;
    Eigen::Matrix<double, 3, 1> position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    // Change left gate vertex color to yellow
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // Change shape to sphere radius 25cm
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.id = id + 2;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, -gate_size, -gate_size;
    position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    marker.id = id + 3;
    line_marker.points.push_back(marker.pose.position);
    marker_array.markers.push_back(marker);

    move_gate << 0.0, gate_size, -gate_size;
    position = pos_gate + rotate_gate * move_gate;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    // Change right gate vertex color back to red
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // Change right gate vertex color back to cube
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.id = id + 4;
    marker_array.markers.push_back(marker);
    line_marker.points.push_back(marker.pose.position);

    marker.pose.position.x = position2(0);
    marker.pose.position.y = position2(1);
    marker.pose.position.z = position2(2);
    line_marker.points.push_back(marker.pose.position);
    id+=5;
    marker_array.markers.push_back(line_marker);
    pub_gate_markers.publish(marker_array);
}

void drawTrajectoryMarkers(const mav_trajectory_generation::Trajectory& trajectory, ros::Publisher& pub_traj_markers){
    visualization_msgs::MarkerArray markers;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    double sampling_time = 0.1;
    mav_msgs::EigenTrajectoryPoint::Vector states;
    double sampling_interval = 0.1;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);   
    for(int i=0; i< states.size(); i++) {
        visualization_msgs::Marker marker_aux;
        marker_aux.header.frame_id = "world";
        //marker_aux.header.stamp = ros::Time::now();
        marker_aux.header.stamp = ros::Time(0);
        marker_aux.id = 1000+i;
        marker_aux.ns = "point";
        marker_aux.type = visualization_msgs::Marker::ARROW;
        marker_aux.pose.position.x = states[i].position_W[0] ;
        marker_aux.pose.position.y = states[i].position_W[1] ;
        marker_aux.pose.position.z = states[i].position_W[2] ;

        // // Extract the orientation of the robot in the world frame
        Eigen::Quaterniond robot_orientation(states[i].orientation_W_B.x(),
                                                states[i].orientation_W_B.y(),
                                                states[i].orientation_W_B.z(),
                                                states[i].orientation_W_B.w());
        robot_orientation.normalize();

        // // No orientation enforced
        // marker_aux.pose.orientation.x = states[i].orientation_W_B.x();
        // marker_aux.pose.orientation.y = states[i].orientation_W_B.y();
        // marker_aux.pose.orientation.z = states[i].orientation_W_B.z();
        // marker_aux.pose.orientation.w = states[i].orientation_W_B.w();

        // // Velocity orientation enforced
        Eigen::Vector3d velocity_W(states[i].velocity_W.x(),
                                    states[i].velocity_W.y(),
                                    states[i].velocity_W.z());

        // Transform the velocity to the robot frame
        Eigen::Vector3d velocity_R = robot_orientation.inverse() * velocity_W;

        // Calculate the yaw based on the velocity in the robot frame
        float yaw = atan2(-velocity_R.y(), -velocity_R.x());

        marker_aux.pose.orientation = RPYToQuat(0, 0, yaw);

        marker_aux.scale.x = 0.03;
        marker_aux.scale.y = 0.03;
        marker_aux.scale.z = 0.03;
        marker_aux.color.r = 0.0f;
        marker_aux.color.g = 0.0f;
        marker_aux.color.b = 1.0f;
        marker_aux.color.a = 1.0;
        marker_aux.scale.x = 0.75; // Length arrow
        marker_aux.scale.y = 0.01; // Diameter cilinder
        marker_aux.scale.z = 0.01; // Diameter head
        marker_aux.lifetime = ros::Duration();
        markers.markers.push_back(marker_aux);
    }
    pub_traj_markers.publish(markers);
}
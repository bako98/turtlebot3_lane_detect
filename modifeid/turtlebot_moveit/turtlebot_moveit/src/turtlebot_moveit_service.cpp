/**
 * @file 
 * @brief 
 *
 * This program implements a MoveIt interface for TurtleBot that waits for
 * service calls to specify target poses and then executes the movement.
 *
 * Author: Rujin Kim
 * Date: 2025-05-17
 */

// // send service like this
// ros2 service call /moveit_control turtlebot_cosmo_interface/srv/MoveitControl "{
//   cmd: 0,
//   posename: 'target_pose',
//   waypoints: {
//     header: {
//       stamp: {sec: 0, nanosec: 0},
//       frame_id: 'base_link'
//     },
//     poses: [
//       {
//         position: {x: 0.4, y: 0.0, z: 0.3},
//         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
//       }
//     ]
//   }
// }"



#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  
#include <turtlebot_cosmo_interface/srv/moveit_control.hpp>

#define PI 3.14159265358979323846

using std::placeholders::_1;
using std::placeholders::_2;

geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = q.w();
    quaternion.x = q.x();
    quaternion.y = q.y();
    quaternion.z = q.z();
    return quaternion;
}

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, 
                   moveit::planning_interface::MoveGroupInterface::Plan& plan, 
                   rclcpp::Logger logger) {
    bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q2, 
                                       const geometry_msgs::msg::Quaternion& q1) {
    geometry_msgs::msg::Quaternion out;
    out.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
    out.x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
    out.y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
    out.z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
    return out;
}

class MoveItControlNode : public rclcpp::Node {
public:
    MoveItControlNode() : Node("turtlebot_moveit") {
        // Initialize service
        service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveitControl>(
            "/moveit_control",
            std::bind(&MoveItControlNode::handle_service_request, this, _1, _2));

        // Initialize MoveGroup interfaces
        arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        arm_interface_->setPlanningPipelineId("move_group");
        arm_interface_->setPlanningTime(10.0);
        arm_interface_->setGoalPositionTolerance(0.01);
        arm_interface_->setGoalOrientationTolerance(0.05);

        gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        RCLCPP_INFO(this->get_logger(), "MoveIt control node initialized and ready for service calls");
    }

private:
    void handle_service_request(
        const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Request> request,
        const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received service request with cmd: %d", request->cmd);

        if (request->cmd == 1) {  // Move to pose command
            if (request->waypoints.poses.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No poses provided in waypoints");
                response->success = false;
                response->message = "No poses provided";
                return;
            }

            // Get the first pose from waypoints
            auto target_pose = request->waypoints.poses[0];

            // Calculate yaw from position
            double yaw = atan2(target_pose.position.y, target_pose.position.x);
            RCLCPP_INFO(this->get_logger(), "Calculated yaw: %f", yaw);

            // Calculate orientation
            geometry_msgs::msg::Quaternion base_orientation = rpyToQuaternion(0, PI/2, 0);
            geometry_msgs::msg::Quaternion yaw_orientation = rpyToQuaternion(0, 0, yaw);
            geometry_msgs::msg::Quaternion final_orientation = multiply(yaw_orientation, base_orientation);

            // Set target pose
            target_pose.orientation = final_orientation;
            arm_interface_->setPoseTarget(target_pose);

            // Plan and execute
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = planAndExecute(*arm_interface_, plan, this->get_logger());

            response->success = success;
            response->message = success ? "Movement executed successfully" : "Movement failed";
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported command received: %d", request->cmd);
            response->success = false;
            response->message = "Unsupported command";
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface_;
    rclcpp::Service<turtlebot_cosmo_interface::srv::MoveitControl>::SharedPtr service_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItControlNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlebot_cosmo_interface/srv/move_to_position.hpp>  // 생성한 서비스 헤더


using namespace std::chrono_literals;

class MoveToPositionServer : public rclcpp::Node {
public:
  MoveToPositionServer() : Node("move_to_position_server") {
    // 서비스 서버 생성
    service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveToPosition>(
      "move_to_position",
      std::bind(&MoveToPositionServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    // MoveIt 인터페이스 초기화
    arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    arm_interface_->setPlanningTime(10.0);
    
    RCLCPP_INFO(this->get_logger(), "MoveToPosition 서비스 서버 준비 완료");
  }

private:
  void handle_service(
    const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Request> request,
    const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Response> response) {
    
    // 현재 pose 가져오기
    geometry_msgs::msg::Pose current_pose = arm_interface_->getCurrentPose().pose;
    
    // 목표 pose 설정 (position만 변경, orientation은 현재 값 유지)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;
    target_pose.orientation = current_pose.orientation;

    RCLCPP_INFO(this->get_logger(), "Moving to position: x=%.2f, y=%.2f, z=%.2f",
                request->x, request->y, request->z);

    // 목표 설정 및 이동 실행
    arm_interface_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_interface_->plan(plan));

    if (success) {
      arm_interface_->execute(plan);
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Movement succeeded");
    } else {
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "Movement planning failed");
    }
  }

  rclcpp::Service<turtlebot_cosmo_interface::srv::MoveToPosition>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPositionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
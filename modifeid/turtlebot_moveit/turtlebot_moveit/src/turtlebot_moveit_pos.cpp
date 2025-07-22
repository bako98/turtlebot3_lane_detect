#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlebot_cosmo_interface/srv/move_to_position.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

class MoveToPositionServer : public rclcpp::Node {
public:
  MoveToPositionServer() : Node("move_to_position_server") {
    // 관절 이름 사전 정의 (실제 로봇 구성에 맞춰 수정 필요)
    arm_joint_names_ = {"joint1", "joint2", "joint3", "joint4"};
    
    // 강력한 QoS 설정
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 
      rclcpp::QoS(10).reliable().durability_volatile().keep_last(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->process_joint_states(msg);
      });

    // 서비스 초기화
    service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveToPosition>(
      "move_to_position",
      [this](const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Request> req,
             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Response> res) {
        this->handle_service(req, res);
      });

    // MoveIt 초기화 타이머
    init_timer_ = this->create_wall_timer(
      1s, [this]() { this->initialize_moveit(); });
  }

private:
  std::vector<std::string> arm_joint_names_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<turtlebot_cosmo_interface::srv::MoveToPosition>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr last_valid_joint_state_;

  void process_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 타임스탬프 검증
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Invalid timestamp in joint_states");
      return;
    }

    // 관절 상태 필터링
    auto filtered = std::make_shared<sensor_msgs::msg::JointState>();
    filtered->header = msg->header;
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (std::find(arm_joint_names_.begin(), arm_joint_names_.end(), msg->name[i]) != arm_joint_names_.end()) {
        filtered->name.push_back(msg->name[i]);
        filtered->position.push_back(msg->position[i]);
        if (i < msg->velocity.size()) filtered->velocity.push_back(msg->velocity[i]);
        if (i < msg->effort.size()) filtered->effort.push_back(msg->effort[i]);
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    last_valid_joint_state_ = filtered;
  }

  void initialize_moveit() {
    if (arm_interface_) return;

    try {
      RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");

      // 1. MoveGroupInterface 생성
      arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");

      // 2. 필수 파라미터 설정
      arm_interface_->setPlanningTime(10.0);
      arm_interface_->setNumPlanningAttempts(20);
      arm_interface_->setGoalPositionTolerance(0.01);
      arm_interface_->setGoalOrientationTolerance(0.05);
      arm_interface_->setMaxVelocityScalingFactor(0.3);

      // 3. 현재 상태 강제 업데이트 (수동으로 joint states 주입)
      if (!update_robot_state()) {
        throw std::runtime_error("Failed to update robot state");
      }

      RCLCPP_INFO(this->get_logger(), "MoveIt initialized successfully");
      init_timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
    }
  }

  bool update_robot_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!last_valid_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "No valid joint states received yet");
      return false;
    }

    // 수동으로 로봇 상태 업데이트
    auto robot_state = arm_interface_->getCurrentState();
    for (size_t i = 0; i < last_valid_joint_state_->name.size(); ++i) {
      const auto& name = last_valid_joint_state_->name[i];
      if (i < last_valid_joint_state_->position.size()) {
        robot_state->setJointPositions(name, &last_valid_joint_state_->position[i]);
      }
    }
    arm_interface_->setStartState(*robot_state);
    return true;
  }

  bool move_to_pose(const geometry_msgs::msg::Pose& target_pose) {
    if (!update_robot_state()) return false;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    arm_interface_->setPoseTarget(target_pose);

    auto plan_result = arm_interface_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed (code: %d)", plan_result.val);
      return false;
    }

    auto exec_result = arm_interface_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Execution failed (code: %d)", exec_result.val);
      return false;
    }

    return true;
  }

  void handle_service(
    const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Request> req,
    std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Response> res) {
    
    if (!arm_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt not initialized");
      res->success = false;
      return;
    }

    try {
      // 1. 현재 포즈 가져오기
      geometry_msgs::msg::Pose current_pose;
      if (update_robot_state()) {
        current_pose = arm_interface_->getCurrentPose().pose;
        RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z);
      } else {
        throw std::runtime_error("Cannot get current pose");
      }

      // 2. 목표 포즈 설정
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = std::clamp(req->x, -0.25, 0.25);
      target_pose.position.y = std::clamp(req->y, -0.15, 0.15);
      target_pose.position.z = std::clamp(req->z, 0.05, 0.3);

      // 3. 방향 계산
      double yaw = atan2(target_pose.position.y, target_pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, M_PI/2, yaw);
      target_pose.orientation = tf2::toMsg(q.normalize());

      // 4. 이동 실행
      res->success = move_to_pose(target_pose);

      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "Movement succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Movement failed");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Service error: %s", e.what());
      res->success = false;
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPositionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
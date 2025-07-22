#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlebot_cosmo_interface/srv/move_to_position.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class MoveToPositionServer : public rclcpp::Node {
public:
  MoveToPositionServer() : Node("move_to_position_server") {
    // Joint states 수신 확인용 서브스크라이버
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = msg;
      });

    // MoveIt 인터페이스 초기화 (타이머 사용)
    init_timer_ = this->create_wall_timer(
      500ms, [this]() {
        if (!arm_interface_ && last_joint_state_) {
          try {
            arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              shared_from_this(), "arm");
            
            // 파라미터 설정
            arm_interface_->setPlanningTime(15.0);
            arm_interface_->setNumPlanningAttempts(20);  // 시도 횟수 증가
            arm_interface_->allowReplanning(true);
            arm_interface_->setGoalPositionTolerance(0.03);
            arm_interface_->setGoalOrientationTolerance(0.1);  // 허용 오차 완화
            arm_interface_->setMaxVelocityScalingFactor(0.5);
            arm_interface_->setMaxAccelerationScalingFactor(0.5);
            
            RCLCPP_INFO(this->get_logger(), "MoveIt 인터페이스 초기화 완료");
            init_timer_->cancel();
            setup_service();
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt 초기화 실패: %s", e.what());
          }
        } else if (!last_joint_state_) {
          RCLCPP_WARN_ONCE(this->get_logger(), "joint_states 메시지 대기 중...");
        }
      });
  }

private:
  geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion quaternion;
    tf2::convert(q, quaternion);
    return quaternion;
  }

  void setup_service() {
    service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveToPosition>(
      "move_to_position",
      [this](const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Request> req,
             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Response> res) {
        this->handle_service(req, res);
      });
    RCLCPP_INFO(this->get_logger(), "서비스 서버 준비 완료");
  }

  void handle_service(
    const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Request> request,
    std::shared_ptr<turtlebot_cosmo_interface::srv::MoveToPosition::Response> response) {
    
    if (!arm_interface_ || !last_joint_state_) {
      RCLCPP_ERROR(this->get_logger(), "시스템이 준비되지 않았습니다");
      response->success = false;
      return;
    }

    try {
      // 1. 현재 상태 확인
      auto current_pose = arm_interface_->getCurrentPose();
      RCLCPP_INFO(this->get_logger(), "현재 위치: x=%.3f, y=%.3f, z=%.3f",
                 current_pose.pose.position.x,
                 current_pose.pose.position.y,
                 current_pose.pose.position.z);

      // 2. 목표 Pose 설정 (안전 범위 확인)
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = std::clamp(request->x, -0.3, 0.3);
      target_pose.position.y = std::clamp(request->y, -0.3, 0.3);
      target_pose.position.z = std::clamp(request->z, 0.02, 0.4);

      // 3. Orientation 계산 (Yaw만 적용)
      double yaw = atan2(target_pose.position.y, target_pose.position.x);
      
      // 기본 pitch(90도) 회전에 yaw 회전을 추가
      tf2::Quaternion q_pitch, q_yaw, q_result;
      q_pitch.setRPY(0, M_PI/2, 0);  // pitch 90도
      q_yaw.setRPY(0, 0, yaw);       // yaw 회전
      q_result = q_pitch * q_yaw;     // 회전 조합 (pitch 먼저, then yaw)
      q_result.normalize();           // 정규화
      
      tf2::convert(q_result, target_pose.orientation);

      RCLCPP_INFO(this->get_logger(), "목표 위치: x=%.3f, y=%.3f, z=%.3f (yaw=%.2f rad)",
                 target_pose.position.x, target_pose.position.y, 
                 target_pose.position.z, yaw);
      RCLCPP_INFO(this->get_logger(), "목표 Orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                 target_pose.orientation.w, target_pose.orientation.x,
                 target_pose.orientation.y, target_pose.orientation.z);

      // 4. 작업 공간 설정
      arm_interface_->setWorkspace(-0.5, -0.5, 0.0, 0.5, 0.5, 0.5);
      arm_interface_->setPoseTarget(target_pose);

      // 5. 계획 및 실행
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode plan_result = arm_interface_->plan(plan);

      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "계획 성공, 실행 시도 중...");
        moveit::core::MoveItErrorCode execute_result = arm_interface_->execute(plan);
        response->success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "이동 성공");
        } else {
          RCLCPP_ERROR(this->get_logger(), "실행 실패: %s", execute_result.message.c_str());
        }
      } else {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "계획 실패: %s", plan_result.message.c_str());
        RCLCPP_ERROR(this->get_logger(), "가능한 이유:");
        RCLCPP_ERROR(this->get_logger(), "- 작업 공간 벗어남");
        RCLCPP_ERROR(this->get_logger(), "- 충돌 가능성");
        RCLCPP_ERROR(this->get_logger(), "- 물리적 제약 조건");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "처리 중 오류: %s", e.what());
      response->success = false;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  rclcpp::Service<turtlebot_cosmo_interface::srv::MoveToPosition>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPositionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
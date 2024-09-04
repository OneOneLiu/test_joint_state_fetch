#include <rclcpp/rclcpp.hpp>
#include "test_joint_state_fetch/srv/print_pose.hpp"  // Header file for the custom service
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

// Definition of the RobotMover class
class RobotMover {
public:
  RobotMover(const rclcpp::Node::SharedPtr& node)
  : move_group_interface_(node, "panda_arm"), node_(node) {
    // Intentionally left empty
  }

  void printCurrentPose() {
    auto current_pose = move_group_interface_.getCurrentPose().pose;
    std::cout << "Current Pose:" << std::endl;
    std::cout << "Position: (" << current_pose.position.x << ", "
              << current_pose.position.y << ", "
              << current_pose.position.z << ")" << std::endl;
    std::cout << "Orientation: (" << current_pose.orientation.x << ", "
              << current_pose.orientation.y << ", "
              << current_pose.orientation.z << ", "
              << current_pose.orientation.w << ")" << std::endl;
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Node::SharedPtr node_;
};

// Definition of the RobotMoverService class
class RobotMoverService {
public:
  RobotMoverService(const rclcpp::Node::SharedPtr& node, RobotMover& robot_mover)
    : node_(node), robot_mover_(robot_mover) {
    // Create a service to print the current pose
    service_ = node_->create_service<test_joint_state_fetch::srv::PrintPose>(
      "print_current_pose", std::bind(&RobotMoverService::handlePrintRequest, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Call in public context
    RCLCPP_INFO(node_->get_logger(), "About to call printCurrentPose in Public");
    robot_mover_.printCurrentPose();
  }

private:
    void handlePrintRequest(const std::shared_ptr<test_joint_state_fetch::srv::PrintPose::Request> /*request*/,
                            std::shared_ptr<test_joint_state_fetch::srv::PrintPose::Response> response) {
        // Call in service callback (private context)
        RCLCPP_INFO(node_->get_logger(), "Service Callback: About to call printCurrentPose in Private");
        robot_mover_.printCurrentPose();
        
        // Set response to success
        response->success = true;
    }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<test_joint_state_fetch::srv::PrintPose>::SharedPtr service_;
  RobotMover& robot_mover_;
};

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("robot_control");
  RobotMover robot_mover(node);
  RobotMoverService robot_mover_service(node, robot_mover);

  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });
  // Call from main
  RCLCPP_INFO(node->get_logger(), "About to call printCurrentPose in Main");
  robot_mover.printCurrentPose();
  
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}

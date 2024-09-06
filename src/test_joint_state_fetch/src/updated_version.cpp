#include <rclcpp/rclcpp.hpp>
#include "test_joint_state_fetch/srv/print_pose.hpp"  // Header file for the custom service
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

// Combined RobotMover class, which includes service and pose printing functionality
class RobotMover : public rclcpp::Node {
public:
  // Constructor to initialize the node, MoveGroupInterface, and executor
  RobotMover(const rclcpp::NodeOptions &options)
  : rclcpp::Node("robot_control", options), // Initialize the node with the name "robot_control"
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "panda_arm"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
  {
    // Create the service for printing the current pose
    service_ = this->create_service<test_joint_state_fetch::srv::PrintPose>(
      "print_current_pose", 
      std::bind(&RobotMover::handlePrintRequest, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Add the node to the executor and start the executor thread
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
      executor_->spin(); // Run the executor to process callbacks
    });
  }

  // Function to print the current end-effector pose
  void printCurrentPose() {
    auto current_pose = move_group_interface_.getCurrentPose().pose; // Get the current pose
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
  // Service callback function to handle pose printing requests
  void handlePrintRequest(const std::shared_ptr<test_joint_state_fetch::srv::PrintPose::Request> /*request*/,
                          std::shared_ptr<test_joint_state_fetch::srv::PrintPose::Response> response) {
    // Print the current pose in the service callback
    RCLCPP_INFO(node_->get_logger(), "Service Callback: About to call printCurrentPose in Private");
    printCurrentPose(); // Print the robot's current pose

    // Set the response to indicate success
    response->success = true;
  }

  // Member variables
  rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
  moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
  rclcpp::Service<test_joint_state_fetch::srv::PrintPose>::SharedPtr service_;  // Service pointer for pose requests
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
  std::thread executor_thread_;  // Thread to run the executor
};

// Main function - Entry point of the program
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); // Initialize ROS 2

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true); // Allow automatic parameter declaration
  node_options.use_intra_process_comms(false); // Disable intra-process communication

  auto node = std::make_shared<RobotMover>(node_options); // Create the RobotMover object and start the node

  rclcpp::spin(node); // Spin the main thread to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 system
  return 0; // Exit the program
}

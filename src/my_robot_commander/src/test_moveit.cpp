#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp> 
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_moveit_node");
  RCLCPP_INFO(node->get_logger(), "Test MoveIt! node has started.");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 获取moveit的group
  auto left_arm=moveit::planning_interface::MoveGroupInterface(node, "arm_left");
  left_arm.setMaxVelocityScalingFactor(0.8);
  left_arm.setMaxAccelerationScalingFactor(1.0);

  // named goal
  left_arm.setStartStateToCurrentState();
  left_arm.setNamedTarget("raise_left_hand");

  // 规划并执行
  moveit::planning_interface::MoveGroupInterface::Plan plan_raise_left_hand;
  bool success1=(left_arm.plan(plan_raise_left_hand) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success1)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to raise_left_hand successful, executing...");
    left_arm.execute(plan_raise_left_hand);
  }
  for(int i=0;i<5;i++)
  {
    if(i%2==1)
    {
      left_arm.setStartStateToCurrentState();
      left_arm.setNamedTarget("leftwave_left_hand");

      // 规划并执行
      moveit::planning_interface::MoveGroupInterface::Plan plan_leftwave_left_hand;
      bool success2=(left_arm.plan(plan_leftwave_left_hand) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success2)
      {
        RCLCPP_INFO(node->get_logger(), "Planning to leftwave_left_hand successful, executing...");
        left_arm.execute(plan_leftwave_left_hand);
      }
    }
    else
    {
      left_arm.setStartStateToCurrentState();
      left_arm.setNamedTarget("waveright_left_hand");

      // 规划并执行
      moveit::planning_interface::MoveGroupInterface::Plan plan_waveright_left_hand;
      bool success3=(left_arm.plan(plan_waveright_left_hand) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success3)
      {
        RCLCPP_INFO(node->get_logger(), "Planning to waveright_left_hand successful, executing...");
        left_arm.execute(plan_waveright_left_hand);
      }
    }
  }
  
  // 最后回到初始位置
  // named goal
  left_arm.setStartStateToCurrentState();
  left_arm.setNamedTarget("raise_left_hand");

  // 规划并执行
  moveit::planning_interface::MoveGroupInterface::Plan plan_final_left_hand;
  bool success4=(left_arm.plan(plan_final_left_hand) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success4)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to final_left_hand successful, executing...");
    left_arm.execute(plan_final_left_hand);
  }




  rclcpp::shutdown(); 
  spinner.join();
  return 0;
}
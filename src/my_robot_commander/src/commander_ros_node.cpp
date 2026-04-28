#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class Commander
{
public:
    // 修正构造函数
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_left");
        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_right");

        left_arm_->setMaxVelocityScalingFactor(0.8);
        left_arm_->setMaxAccelerationScalingFactor(1.0);
        right_arm_->setMaxVelocityScalingFactor(0.8);
        right_arm_->setMaxAccelerationScalingFactor(1.0);

        left_arm_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm_joint_trajectory_position_controller/joint_trajectory", 10);
        right_arm_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm_joint_trajectory_position_controller/joint_trajectory", 10);

        // 服务回调绑定到成员函数
        hello_left_srv_ = node_->create_service<std_srvs::srv::Trigger>("hello_left",
            std::bind(&Commander::handleHelloLeft, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    void goToNamedTarget(const std::string& group_name, const std::string& target_name)
    {
        // 统一使用 arm_left / arm_right
        if (group_name == "arm_left")
        {
            left_arm_->setStartStateToCurrentState();
            left_arm_->setNamedTarget(target_name);
            planAndPublish(left_arm_, left_arm_traj_pub_, target_name);
        }
        else if (group_name == "arm_right")
        {
            right_arm_->setStartStateToCurrentState();
            right_arm_->setNamedTarget(target_name);
            planAndPublish(right_arm_, right_arm_traj_pub_, target_name);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid group name: %s", group_name.c_str());
        }
    }

    void hello_action_left()
    {
        goToNamedTarget("arm_left", "raise_left_hand");
        for (int i = 0; i < 5; i++)
        {
            if (i % 2 == 1)
                goToNamedTarget("arm_left", "leftwave_left_hand");
            else
                goToNamedTarget("arm_left", "waveright_left_hand");
        }
        // 修正：回到左臂初始姿态
        goToNamedTarget("arm_left", "raise_left_hand");
    }

private:
    // 重命名并增加 publisher 参数
    void planAndPublish(
        const std::shared_ptr<MoveGroupInterface>& arm,
        const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub,
        const std::string& target_name)
    {
        MoveGroupInterface::Plan plan;
        bool success = (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            // 正确访问 plan.trajectory
            if (!plan.trajectory.joint_trajectory.points.empty())
            {
                pub->publish(plan.trajectory.joint_trajectory);
                RCLCPP_INFO(node_->get_logger(), "Trajectory for '%s' published.", target_name.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Empty trajectory planned for '%s'.", target_name.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for target '%s'.", target_name.c_str());
        }
    }

    // 服务回调函数：触发左手打招呼动作
    void handleHelloLeft(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        hello_action_left();
        res->success = true;
        res->message = "Hello left action finished.";
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_traj_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hello_left_srv_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander_node");
    auto commander = std::make_shared<Commander>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp> 
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>
#include <map>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>




using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    {
        // 初始化机械臂MoveGroup
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_left");
        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "arm_right");
        
        // 设置速度缩放
        left_arm_->setMaxVelocityScalingFactor(0.8);
        left_arm_->setMaxAccelerationScalingFactor(1.0);
        right_arm_->setMaxVelocityScalingFactor(0.8);
        right_arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建服务，用于触发挥手动作
        wave_left_service_ = node_->create_service<std_srvs::srv::Trigger>(
            "wave_left_hand",
            std::bind(&Commander::waveLeftHandCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));

        left_arm_traj_pub_=node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/left_arm_joint_trajectory_position_controller/joint_trajectory", 10);
    }



    // 挥手动作的服务回调
    void waveLeftHandCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try {
            waveLeftHand();
            response->success = true;
            response->message = "Wave motion completed";
        } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
        }
    }


    // 核心：挥手动作的连续轨迹实现（新增轨迹平滑规划，解决运动不顺畅问题）
    void waveLeftHand()
    {
        RCLCPP_INFO(node_->get_logger(), "Starting left hand wave motion...");

        // 1. 获取左手臂的关节名称顺序（保证关节值顺序正确）
        const std::vector<std::string> joint_names = left_arm_->getJointNames();
        
        // 2. 读取你在SRDF中配置的三个预设状态的关节值
        std::map<std::string, double> raise_vals = left_arm_->getNamedTargetValues("raise_left_hand");
        std::map<std::string, double> wave_left_vals = left_arm_->getNamedTargetValues("leftwave_left_hand");
        std::map<std::string, double> wave_right_vals = left_arm_->getNamedTargetValues("waveright_left_hand");

        // 3. 将map格式的关节值转换为按关节顺序排列的vector
        auto raise_vec = toJointVector(raise_vals, joint_names);
        auto wave_left_vec = toJointVector(wave_left_vals, joint_names);
        auto wave_right_vec = toJointVector(wave_right_vals, joint_names);

        // 4. 构造连续关节轨迹（初始轨迹，仅包含目标点，无平滑处理）
        trajectory_msgs::msg::JointTrajectory joint_traj;
        joint_traj.joint_names = joint_names;

        // 添加轨迹点：
        // 点1: 抬手到初始位置，耗时2秒
        trajectory_msgs::msg::JointTrajectoryPoint p1;
        p1.positions = raise_vec;
        p1.time_from_start = rclcpp::Duration::from_seconds(2.0);
        joint_traj.points.push_back(p1);

        // 来回摆动3次，每次摆动耗时1秒（初始时间戳，后续会被平滑算法优化）
        double current_time = 2.0;
        for(int i=0; i<3; i++)
        {
            // 摆到左
            trajectory_msgs::msg::JointTrajectoryPoint p_left;
            p_left.positions = wave_left_vec;
            current_time += 0.5;
            p_left.time_from_start = rclcpp::Duration::from_seconds(current_time);
            joint_traj.points.push_back(p_left);

            // 摆到右
            trajectory_msgs::msg::JointTrajectoryPoint p_right;
            p_right.positions = wave_right_vec;
            current_time += 0.5;
            p_right.time_from_start = rclcpp::Duration::from_seconds(current_time);
            joint_traj.points.push_back(p_right);
        }

        const moveit::core::RobotModelConstPtr& robot_model = left_arm_->getRobotModel();
        robot_trajectory::RobotTrajectory rt(robot_model, "arm_left");
        
        // 修复：直接传RobotState对象，不是getRobotStateMsg()
        rt.setRobotTrajectoryMsg(*(left_arm_->getCurrentState()), joint_traj);

        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        bool smooth_success = time_param.computeTimeStamps(
            rt,  // 传RobotTrajectory而不是JointTrajectory
            left_arm_->getMaxVelocityScalingFactor(),
            left_arm_->getMaxAccelerationScalingFactor()
        );
        if (!smooth_success)
        {
            RCLCPP_WARN(node_->get_logger(), "Trajectory smoothing failed, using original trajectory");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Trajectory smoothed successfully");
        }
        // --------------------------------------------------------------------------

        // 包装成RobotTrajectory（兼容MoveGroup的execute接口）
        moveit_msgs::msg::RobotTrajectory robot_traj;
        robot_traj.joint_trajectory = joint_traj;
        left_arm_traj_pub_->publish(joint_traj);

        // 执行整个连续轨迹（此时轨迹已平滑，运动无突变、更顺畅）
        // left_arm_->execute(robot_traj);
        RCLCPP_INFO(node_->get_logger(), "Wave motion completed!");
    }

private:
    // 将map格式的关节值转换为按关节顺序排列的vector
    std::vector<double> toJointVector(
        const std::map<std::string, double> &joint_map, 
        const std::vector<std::string> &joint_names)
    {
        std::vector<double> res;
        res.reserve(joint_names.size());
        for(const auto &name : joint_names)
        {
            if(joint_map.find(name) == joint_map.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing joint value for: %s", name.c_str());
                throw std::runtime_error("Missing joint value");
            }
            res.push_back(joint_map.at(name));
        }
        return res;
    }


    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_arm_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wave_left_service_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_traj_pub_;
};

// 自定义服务消息定义（如果你没有这个接口，可以创建一个简单的，或者用这个）
// 你需要在你的package里添加这个自定义接口，或者简化为直接调用，比如：
// 如果你不想用服务，也可以在main函数里直接调用commander.waveLeftHand()来测试

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("commander_node");
  auto commander = Commander(node);

  // 如果你想启动节点后自动执行挥手，可以取消下面这行注释
  std::thread([&commander](){ commander.waveLeftHand(); }).detach();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
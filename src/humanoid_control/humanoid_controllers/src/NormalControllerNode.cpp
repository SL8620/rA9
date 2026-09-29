//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"
#include <algorithm>
#include <thread>
#include "rclcpp/rclcpp.hpp"

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

bool pause_flag = false;
// 控制周期超限告警阈值与 dt 上限：单次循环超过 3 倍周期时告警；
// dt 钳位防止一次长卡顿（如断点调试）以巨大 dt 灌入状态估计器
constexpr double kDesiredPeriod = 1.0 / 500;
constexpr double kOverrunWarnFactor = 3.0;
constexpr double kMaxDt = 0.005;

void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg){
    pause_flag = msg->data;
    std::cerr << "pause_flag: " << pause_flag << std::endl;
}

int main(int argc, char** argv){
    rclcpp::Duration elapsedTime_ = rclcpp::Duration::from_seconds(0.002);

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        "humanoid_controller_node",
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));


    
    //create a subscriber to pauseFlag
    auto pause_sub = node->create_subscription<std_msgs::msg::Bool>("pauseFlag", 1, pauseCallback);
    humanoid_controller::humanoidController controller;

    if (!controller.init(node)) {
        RCLCPP_ERROR(node->get_logger(),"Failed to initialize the humanoid controller!");
        return -1;
    }

    auto startTime = Clock::now();
    auto startTimeROS = node->get_clock()->now();
    controller.starting(startTimeROS);
    auto lastTime = startTime;

    //create a thread to spin the node
    std::thread spin_thread([node](){
        rclcpp::spin(node);
    });
    spin_thread.detach();

    while(rclcpp::ok()){
        if (!pause_flag)
        {
            const auto currentTime = Clock::now();
            // Compute desired duration rounded to clock decimation
            const Duration desiredDuration(kDesiredPeriod);

            // Get change in time
            Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime);
            lastTime = currentTime;

            // Check cycle time for excess delay
            if (time_span.count() > kDesiredPeriod * kOverrunWarnFactor) {
                RCLCPP_WARN(node->get_logger(),
                            "Control cycle overrun: cycle time %.1f ms (expected %.1f ms)",
                            time_span.count() * 1e3, kDesiredPeriod * 1e3);
            }
            // dt 钳位：防止一次长卡顿（如断点调试）以巨大 dt 灌入状态估计器
            elapsedTime_ = rclcpp::Duration::from_seconds(std::min(time_span.count(), kMaxDt));

            // Control
            // let the controller compute the new command (via the controller manager)
            controller.update(node->get_clock()->now(), elapsedTime_);

            // Sleep
            const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
            std::this_thread::sleep_until(sleepTill);
        }
    }



    return 0;
}
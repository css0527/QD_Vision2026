#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "armor_solver/armor_solver_node.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_utils/vision_state_machine.hpp"

namespace qd::auto_aim {
namespace {
    using namespace std::chrono_literals;

    TEST(ArmorSolverTargetTimeout, StopsCommandsWithoutNewFramesAndRecovers) {
        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__ns:=/target_timeout_test" });
        options.parameter_overrides({
            { "debug", false },
            { "target_frame", "timeout_test_odom" },
            { "gimble_frame", "timeout_test_gimbal" },
            { "enable_solveYawPnp", false },
            { "tracker.tracking_thres", 0 },
            { "tracker.lost_time_thres", 0.2 },
            { "solver.use_armor_top", true },
        });
        auto solver_node = std::make_shared<ArmorSolverNode>(options);
        auto observer = std::make_shared<rclcpp::Node>("observer", "/target_timeout_test");

        std::optional<rm_interfaces::msg::GimbalCmd> latest_command;
        auto command_sub = observer->create_subscription<rm_interfaces::msg::GimbalCmd>(
            "armor_solver/cmd_gimbal",
            rclcpp::SensorDataQoS(),
            [&latest_command](rm_interfaces::msg::GimbalCmd::ConstSharedPtr command) {
                latest_command = *command;
            }
        );

        tf2_ros::StaticTransformBroadcaster broadcaster(observer);
        geometry_msgs::msg::TransformStamped gimbal_transform;
        gimbal_transform.header.stamp = observer->now();
        gimbal_transform.header.frame_id = "timeout_test_odom";
        gimbal_transform.child_frame_id = "timeout_test_gimbal";
        gimbal_transform.transform.rotation.w = 1.0;
        broadcaster.sendTransform(gimbal_transform);

        auto& state_machine = qd::utils::VisionStateMachine::getInstance();
        state_machine.setVisionMode(VisionMode::AUTO_AIM_RED);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(solver_node);
        executor.add_node(observer);

        // 直接输入 odom 观测，隔离相机和串口设备，同时走真实工作线程与定时发布链路。
        auto publish_frame = [&](bool has_observation = true) {
            std_msgs::msg::Header header;
            header.stamp = observer->now();
            header.frame_id = "timeout_test_odom";
            rm_interfaces::msg::Armor armor;
            armor.number = "4";
            armor.type = "small";
            armor.pose.position.x = 2.0;
            armor.pose.orientation.w = 1.0;
            armor.area = 1000.0;
            std::vector<rm_interfaces::msg::Armor> armors;
            if (has_observation) {
                armors.push_back(armor);
            }
            state_machine.publishArmorFrame(
                header,
                std::move(armors),
                cv::Mat {},
                sensor_msgs::msg::CameraInfo {}
            );
        };

        // 持续提供新帧直到收到有效指令，允许 DDS 发现和静态 TF 建立连接。
        auto acquire_target = [&]() {
            const auto deadline = std::chrono::steady_clock::now() + 3s;
            auto next_frame = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() < deadline) {
                if (std::chrono::steady_clock::now() >= next_frame) {
                    publish_frame();
                    next_frame = std::chrono::steady_clock::now() + 20ms;
                }
                executor.spin_some();
                if (latest_command && latest_command->distance > 0.0) {
                    return true;
                }
                std::this_thread::sleep_for(5ms);
            }
            return false;
        };

        ASSERT_TRUE(acquire_target()) << "新鲜目标应先产生有效角度指令";

        // 相机仍有新帧但单帧没有装甲板时，停止下发预测角，保持内部 TEMP_LOST 状态。
        publish_frame(false);
        const auto empty_frame_deadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < empty_frame_deadline) {
            executor.spin_some();
            if (state_machine.getTrackerState() == TrackerState::TEMP_LOST && latest_command
                && latest_command->distance < 0.0)
            {
                break;
            }
            std::this_thread::sleep_for(5ms);
        }
        EXPECT_EQ(state_machine.getTrackerState(), TrackerState::TEMP_LOST);
        ASSERT_TRUE(latest_command.has_value());
        EXPECT_LT(latest_command->distance, 0.0);
        EXPECT_FALSE(latest_command->fire_advice);
        latest_command.reset();
        ASSERT_TRUE(acquire_target()) << "短时漏检后的新观测应恢复有效指令";

        // 停止所有新帧，模拟图像中断或 TF 连续失败后的旧目标残留。
        const auto expiry_deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < expiry_deadline) {
            executor.spin_some();
            if (latest_command && latest_command->distance < 0.0) {
                break;
            }
            std::this_thread::sleep_for(5ms);
        }
        ASSERT_TRUE(latest_command.has_value());
        EXPECT_LT(latest_command->distance, 0.0) << "超时旧目标不得继续输出有效指令";
        EXPECT_FALSE(latest_command->fire_advice);

        latest_command.reset();
        EXPECT_TRUE(acquire_target()) << "新鲜观测恢复后应重新输出有效指令";
    }
} // namespace
} // namespace qd::auto_aim

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

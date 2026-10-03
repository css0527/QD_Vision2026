#include <gtest/gtest.h>

#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "armor_solver/armor_solver.hpp"
#include "rm_utils/logger/log.hpp"

namespace qd::auto_aim {
namespace {
    class VerticalPredictionTest: public testing::TestWithParam<bool> {
    protected:
        void SetUp() override {
            FYT_REGISTER_LOGGER("armor_solver", "qd2026-log", INFO);
            rclcpp::NodeOptions options;
            options.parameter_overrides({
                { "solver.use_armor_top", GetParam() },
                { "solver.vertical_prediction_gain", 0.0 },
                { "solver.additional_prediction_time", 0.1 },
                { "solver.bullet_speed", 20.0 },
                { "solver.compensator_type", "ideal" },
            });
            node_ = std::make_shared<rclcpp::Node>("vertical_prediction_test", options);
            node_->declare_parameter("gimble_frame", "prediction_test_gimbal");
            solver_ = std::make_unique<Solver>(node_);
            buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

            // 直接写入本地静态 TF，隔离硬件、DDS 发现和运行线程的时序。
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
            transform.header.frame_id = "prediction_test_odom";
            transform.child_frame_id = "prediction_test_gimbal";
            transform.transform.rotation.w = 1.0;
            ASSERT_TRUE(buffer_->setTransform(transform, "vertical_prediction_test", true));
        }

        /**
         * @brief 构造两种运动模型均能瞄准同一块正面装甲板的目标。
         * @param height_m 装甲板在 odom 中的高度，单位米。
         * @param vertical_speed_m_s 两种模型共同的竖直速度，单位米每秒。
         * @return 图像时刻固定、正面装甲板距离为三米的有效目标。
         */
        rm_interfaces::msg::Target make_target(double height_m, double vertical_speed_m_s) {
            rm_interfaces::msg::Target target;
            target.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
            target.header.frame_id = "prediction_test_odom";
            target.tracking = true;
            target.id = "4";
            target.armors_num = 4;
            target.position.x = 3.2;
            target.position.z = height_m;
            target.position_armor.x = 3.0;
            target.position_armor.z = height_m;
            target.velocity.z = vertical_speed_m_s;
            target.velocity_armor.z = vertical_speed_m_s;
            target.dz_list.assign(4, height_m);
            target.radius_list.assign(4, 0.2);
            return target;
        }

        /**
         * @brief 通过公开求解接口计算固定图像延迟下的实际指令。
         * @param target 包含装甲板位置和运动状态的输入目标。
         * @return 使用八十毫秒图像延迟和真实弹道计算的云台指令。
         */
        rm_interfaces::msg::GimbalCmd solve(rm_interfaces::msg::Target target) {
            return solver_->solve(target, rclcpp::Time(1, 80000000, RCL_ROS_TIME), buffer_);
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::unique_ptr<Solver> solver_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
    };

    TEST_P(VerticalPredictionTest, ZeroGainRejectsAlternatingVerticalVelocityNoise) {
        const auto stationary = solve(make_target(0.0, 0.0));
        ASSERT_GT(stationary.distance, 0.0);
        for (const double noise_speed: { -2.0, 2.0, -0.5, 0.5 }) {
            const auto command = solve(make_target(0.0, noise_speed));
            EXPECT_NEAR(command.pitch, stationary.pitch, 1e-9);
            EXPECT_NEAR(command.pitch_vel, 0.0, 1e-9);
            EXPECT_NEAR(command.yaw, stationary.yaw, 1e-9);
        }
    }

    TEST_P(VerticalPredictionTest, ZeroGainStillFollowsMeasuredHeightChanges) {
        const auto below = solve(make_target(-0.2, 1.0));
        const auto level = solve(make_target(0.0, -1.0));
        const auto above = solve(make_target(0.2, 1.0));
        ASSERT_GT(below.distance, 0.0);
        ASSERT_GT(above.distance, 0.0);
        EXPECT_LT(below.pitch, level.pitch - 1.0);
        EXPECT_GT(above.pitch, level.pitch + 1.0);
    }

    TEST_P(VerticalPredictionTest, FullGainRestoresSignedVerticalLead) {
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 1.0 }).successful);
        const auto stationary = solve(make_target(0.0, 0.0));
        const auto rising = solve(make_target(0.0, 1.0));
        const auto falling = solve(make_target(0.0, -1.0));
        EXPECT_GT(rising.pitch, stationary.pitch + 1.0);
        EXPECT_LT(falling.pitch, stationary.pitch - 1.0);
        EXPECT_GT(rising.pitch_vel, 0.0);
        EXPECT_LT(falling.pitch_vel, 0.0);
    }

    TEST_P(VerticalPredictionTest, ParameterUpdateChangesPredictionWithoutRecreatingSolver) {
        const auto without_lead = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 1.0 }).successful);
        const auto full_lead = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 0.5 }).successful);
        const auto partial_lead = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 0.0 }).successful);
        const auto restored = solve(make_target(0.0, 1.0));
        EXPECT_GT(partial_lead.pitch, without_lead.pitch);
        EXPECT_LT(partial_lead.pitch, full_lead.pitch);
        EXPECT_NEAR(restored.pitch, without_lead.pitch, 1e-9);
        EXPECT_NEAR(restored.pitch_vel, 0.0, 1e-9);
    }

    TEST_P(VerticalPredictionTest, ZeroVerticalGainPreservesHorizontalPrediction) {
        const auto stationary = solve(make_target(0.0, 0.0));
        auto moving_target = make_target(0.0, 2.0);
        moving_target.velocity.y = 0.5;
        moving_target.velocity_armor.y = 0.5;
        const auto moving = solve(moving_target);
        EXPECT_GT(moving.yaw, stationary.yaw + 0.1);
        EXPECT_GT(moving.yaw_vel, 0.0);
    }

    TEST_P(VerticalPredictionTest, AdditionalPredictionUsesTheSameScaledVerticalVelocity) {
        ASSERT_TRUE(node_->set_parameter({ "solver.additional_prediction_time", 0.0 }).successful);
        const auto zero_gain_without_extra = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.additional_prediction_time", 0.2 }).successful);
        const auto zero_gain_with_extra = solve(make_target(0.0, 1.0));
        EXPECT_NEAR(zero_gain_with_extra.pitch, zero_gain_without_extra.pitch, 1e-9);
        EXPECT_NEAR(zero_gain_with_extra.pitch_vel, 0.0, 1e-9);

        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 1.0 }).successful);
        ASSERT_TRUE(node_->set_parameter({ "solver.additional_prediction_time", 0.0 }).successful);
        const auto full_gain_without_extra = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.additional_prediction_time", 0.2 }).successful);
        const auto full_gain_with_extra = solve(make_target(0.0, 1.0));
        EXPECT_GT(full_gain_with_extra.pitch, full_gain_without_extra.pitch + 1.0);
        EXPECT_GT(full_gain_with_extra.pitch_vel, 0.0);
        EXPECT_NEAR(full_gain_with_extra.pitch_vel, full_gain_without_extra.pitch_vel, 1e-9);
    }

    TEST_P(VerticalPredictionTest, RuntimeGainIsClampedToTheDocumentedRange) {
        const auto zero_gain = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", -1.0 }).successful);
        const auto negative_gain = solve(make_target(0.0, 1.0));
        EXPECT_NEAR(negative_gain.pitch, zero_gain.pitch, 1e-9);

        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 1.0 }).successful);
        const auto full_gain = solve(make_target(0.0, 1.0));
        ASSERT_TRUE(node_->set_parameter({ "solver.vertical_prediction_gain", 2.0 }).successful);
        const auto oversized_gain = solve(make_target(0.0, 1.0));
        EXPECT_NEAR(oversized_gain.pitch, full_gain.pitch, 1e-9);
        EXPECT_NEAR(oversized_gain.pitch_vel, full_gain.pitch_vel, 1e-9);
    }

    INSTANTIATE_TEST_SUITE_P(
        ArmorAndRobotModels,
        VerticalPredictionTest,
        testing::Values(true, false)
    );
} // namespace
} // namespace qd::auto_aim

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

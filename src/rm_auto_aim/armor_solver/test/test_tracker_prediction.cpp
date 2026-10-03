// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "armor_solver/armor_tracker.hpp"

namespace qd::auto_aim {
namespace {

    constexpr double FRAME_DT = 0.1;
    constexpr double TOLERANCE = 1e-9;

    /**
     * @brief 构造指定位置的单装甲板观测。
     * @param position 装甲板在跟踪坐标系中的位置，单位为米。
     * @param number 装甲板编号。
     * @return 含一个朝向角为零的小装甲板的检测帧。
     */
    Tracker::Armors::SharedPtr
    make_frame(const Eigen::Vector3d& position, const std::string& number = "4") {
        auto frame = std::make_shared<Tracker::Armors>();
        Tracker::Armor armor;
        armor.number = number;
        armor.type = "small";
        armor.area = 100.0;
        armor.pose.position.x = position.x();
        armor.pose.position.y = position.y();
        armor.pose.position.z = position.z();
        armor.pose.orientation.w = 1.0;
        frame->armors.push_back(armor);
        return frame;
    }

    class TrackerPredictionTest: public testing::Test {
    protected:
        void SetUp() override {
            FYT_REGISTER_LOGGER("armor_solver", "qd2026-log", INFO);
            tracker_ = std::make_unique<Tracker>(0.5, 0.7);
            tracker_->tracking_thres = 0;
            tracker_->lost_thres = 10;
            tracker_->ekf = std::make_unique<RobotStateEKF>(
                Predict(FRAME_DT),
                Measure(),
                []() -> RobotStateEKF::MatrixXX { return RobotStateEKF::MatrixXX::Zero(); },
                [](const RobotStateEKF::MatrixZ1&) -> RobotStateEKF::MatrixZZ {
                    return RobotStateEKF::MatrixZZ::Identity();
                },
                RobotStateEKF::MatrixXX::Identity()
            );
            tracker_->ekf_point = std::make_unique<ArmorStateEKF>(
                EkfPredict(FRAME_DT),
                EkfMeasure(),
                []() -> ArmorStateEKF::MatrixXX { return ArmorStateEKF::MatrixXX::Zero(); },
                [](const ArmorStateEKF::MatrixZ1&) -> ArmorStateEKF::MatrixZZ {
                    return ArmorStateEKF::MatrixZZ::Identity();
                },
                ArmorStateEKF::MatrixXX::Identity()
            );
            tracker_->init(make_frame(initial_position_));
            tracker_->update(make_frame(initial_position_));
            ASSERT_EQ(tracker_->tracker_state, TrackerState::TRACKING);
        }

        /**
         * @brief 设置两个模型的相同初速度，用于验证无观测期间的时间推进。
         */
        void set_velocity() {
            for (int axis = 0; axis < 3; ++axis) {
                tracker_->target_state(2 * axis + 1) = velocity_(axis);
                tracker_->armor_state(2 * axis + 1) = velocity_(axis);
            }
            tracker_->ekf->setState(tracker_->target_state);
            tracker_->ekf_point->setState(tracker_->armor_state);
        }

        /**
         * @brief 检查两个模型在丢失观测后均推进到预期时刻。
         * @param frames 自初始状态起经过的帧数。
         */
        void expect_prediction(int frames) {
            const Eigen::Vector3d expected = initial_position_ + frames * FRAME_DT * velocity_;
            for (int axis = 0; axis < 3; ++axis) {
                EXPECT_NEAR(tracker_->armor_state(2 * axis), expected(axis), TOLERANCE);
                EXPECT_NEAR(tracker_->armor_state(2 * axis + 1), velocity_(axis), TOLERANCE);
                const double center_offset = axis == 0 ? 0.26 : 0.0;
                EXPECT_NEAR(
                    tracker_->target_state(2 * axis),
                    expected(axis) + center_offset,
                    TOLERANCE
                );
            }
            ASSERT_EQ(tracker_->dz_list_.size(), 4U);
            for (const double height: tracker_->dz_list_) {
                EXPECT_NEAR(height, expected.z(), TOLERANCE);
            }
        }

        std::unique_ptr<Tracker> tracker_;
        const Eigen::Vector3d initial_position_ { 3.0, 0.5, 0.2 };
        const Eigen::Vector3d velocity_ { 0.4, -0.3, 0.2 };
    };

    TEST_F(TrackerPredictionTest, EmptyFramesAdvanceBothModelsExactlyOnce) {
        set_velocity();
        auto empty_frame = std::make_shared<Tracker::Armors>();
        for (int frame = 1; frame <= 3; ++frame) {
            tracker_->update(empty_frame);
            EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
            expect_prediction(frame);
        }
    }

    TEST_F(TrackerPredictionTest, OtherArmorIdsPreserveCurrentPrediction) {
        set_velocity();
        auto unrelated_frame = make_frame(Eigen::Vector3d(8.0, 2.0, -1.0), "3");
        for (int frame = 1; frame <= 3; ++frame) {
            tracker_->update(unrelated_frame);
            EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
            EXPECT_EQ(tracker_->tracked_id, "4");
            expect_prediction(frame);
        }
    }

    TEST_F(TrackerPredictionTest, ReacquisitionUsesElapsedMissingFrames) {
        set_velocity();
        auto empty_frame = std::make_shared<Tracker::Armors>();
        for (int frame = 0; frame < 3; ++frame) {
            tracker_->update(empty_frame);
        }
        const Eigen::Vector3d current_position = initial_position_ + 4 * FRAME_DT * velocity_;
        tracker_->update(make_frame(current_position));
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TRACKING);
        expect_prediction(4);
    }

    TEST_F(TrackerPredictionTest, ReinitializationClearsDetectionCount) {
        tracker_->tracking_thres = 2;
        auto frame = make_frame(initial_position_);
        tracker_->init(frame);
        tracker_->update(frame);
        tracker_->update(frame);
        ASSERT_EQ(tracker_->tracker_state, TrackerState::DETECTING);
        tracker_->init(frame);
        tracker_->update(frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::DETECTING);
        tracker_->update(frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::DETECTING);
        tracker_->update(frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TRACKING);
    }

    TEST_F(TrackerPredictionTest, ReinitializationClearsLostCount) {
        tracker_->lost_thres = 2;
        auto empty_frame = std::make_shared<Tracker::Armors>();
        tracker_->update(empty_frame);
        tracker_->update(empty_frame);
        ASSERT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
        tracker_->init(make_frame(initial_position_));
        tracker_->update(make_frame(initial_position_));
        ASSERT_EQ(tracker_->tracker_state, TrackerState::TRACKING);
        tracker_->update(empty_frame);
        tracker_->update(empty_frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::TEMP_LOST);
        tracker_->update(empty_frame);
        EXPECT_EQ(tracker_->tracker_state, TrackerState::LOST);
    }

    struct IdentityModel {
        template<typename T>
        void operator()(const T input[1], T output[1]) const {
            output[0] = input[0];
        }
    };

    TEST(TrackerCovarianceTest, MissingObservationsAccumulatePredictionUncertainty) {
        using ScalarEkf = ExtendedKalmanFilter<1, 1, IdentityModel, IdentityModel>;
        ScalarEkf filter(
            IdentityModel {},
            IdentityModel {},
            []() -> ScalarEkf::MatrixXX { return ScalarEkf::MatrixXX::Constant(2.0); },
            [](const ScalarEkf::MatrixZ1&) -> ScalarEkf::MatrixZZ {
                return ScalarEkf::MatrixZZ::Constant(3.0);
            },
            ScalarEkf::MatrixXX::Constant(1.0)
        );
        filter.init(ScalarEkf::MatrixX1::Zero());
        for (int frame = 0; frame < 3; ++frame) {
            filter.predict();
        }
        // 三次无观测预测后 P = 1 + 3 * 2 = 7，观测增益为 7 / (7 + 3)。
        EXPECT_NEAR(filter.update(ScalarEkf::MatrixZ1::Constant(10.0))(0), 7.0, TOLERANCE);
    }

} // namespace
} // namespace qd::auto_aim

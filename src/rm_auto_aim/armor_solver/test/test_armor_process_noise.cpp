// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Eigenvalues>

#include "armor_solver/armor_process_noise.hpp"

namespace qd::auto_aim {
namespace {

    constexpr double FRAME_DT = 0.01;
    constexpr double POSITION_NOISE_DENSITY = 0.01;
    constexpr double ACCELERATION_NOISE_DENSITY = 100.0;
    constexpr double TARGET_DISTANCE = 0.7;

    /**
     * @brief 构造与生产参数相同的观测模型，并选择修复前后不同的过程噪声。
     * @param use_time_scaled_noise 为真时使用按秒积分的过程噪声。
     * @return 已初始化到前方 0.7 米静止装甲板的平移滤波器。
     */
    ArmorStateEKF make_filter(bool use_time_scaled_noise) {
        ArmorStateEKF filter(
            EkfPredict(FRAME_DT),
            EkfMeasure(),
            [use_time_scaled_noise]() -> ArmorStateEKF::MatrixXX {
                if (use_time_scaled_noise) {
                    return make_armor_process_noise(
                        FRAME_DT,
                        POSITION_NOISE_DENSITY,
                        ACCELERATION_NOISE_DENSITY
                    );
                }
                ArmorStateEKF::MatrixXX noise = ArmorStateEKF::MatrixXX::Zero();
                noise.diagonal() << POSITION_NOISE_DENSITY, ACCELERATION_NOISE_DENSITY,
                    POSITION_NOISE_DENSITY, ACCELERATION_NOISE_DENSITY, POSITION_NOISE_DENSITY,
                    ACCELERATION_NOISE_DENSITY;
                return noise;
            },
            [](const ArmorStateEKF::MatrixZ1& measurement) -> ArmorStateEKF::MatrixZZ {
                ArmorStateEKF::MatrixZZ noise = ArmorStateEKF::MatrixZZ::Zero();
                noise.diagonal() << 1.0, 1.5, 50.0 * measurement(2) * measurement(2);
                return noise;
            },
            ArmorStateEKF::MatrixXX::Identity()
        );
        ArmorStateEKF::MatrixX1 initial_state = ArmorStateEKF::MatrixX1::Zero();
        initial_state(0) = TARGET_DISTANCE;
        filter.init(initial_state);
        return filter;
    }

    /**
     * @brief 生成平移状态在指定时间间隔上的线性转移矩阵。
     * @param delta_t 预测时间间隔，单位为秒。
     * @return 保持各轴速度不变的状态转移矩阵。
     */
    ArmorStateEKF::MatrixXX transition(double delta_t) {
        ArmorStateEKF::MatrixXX matrix = ArmorStateEKF::MatrixXX::Identity();
        for (int position_index = 0; position_index < X_N_; position_index += 2) {
            matrix(position_index, position_index + 1) = delta_t;
        }
        return matrix;
    }

    TEST(ArmorProcessNoise, ZeroElapsedTimeDoesNotInjectNoise) {
        EXPECT_TRUE(
            make_armor_process_noise(0.0, POSITION_NOISE_DENSITY, ACCELERATION_NOISE_DENSITY)
                .isZero()
        );
    }

    TEST(ArmorProcessNoise, AllAxesHavePositiveSemidefiniteSymmetricCovariance) {
        for (const double interval: { 0.001, 0.01, 0.1, 0.3 }) {
            const auto noise = make_armor_process_noise(
                interval,
                POSITION_NOISE_DENSITY,
                ACCELERATION_NOISE_DENSITY
            );
            EXPECT_TRUE(noise.isApprox(noise.transpose(), 1e-12));
            Eigen::SelfAdjointEigenSolver<ArmorStateEKF::MatrixXX> eigen_solver(noise);
            ASSERT_EQ(eigen_solver.info(), Eigen::Success);
            EXPECT_GE(eigen_solver.eigenvalues().minCoeff(), -1e-12);
            EXPECT_TRUE((noise.block<2, 2>(0, 0).isApprox(noise.block<2, 2>(2, 2), 1e-12)));
            EXPECT_TRUE((noise.block<2, 2>(0, 0).isApprox(noise.block<2, 2>(4, 4), 1e-12)));
            EXPECT_GT(noise(0, 1), 0.0);
        }
    }

    TEST(ArmorProcessNoise, SplittingElapsedTimePreservesPredictedCovariance) {
        const std::vector<double> intervals { 0.003, 0.017, 0.04, 0.08 };
        ArmorStateEKF::MatrixXX covariance = ArmorStateEKF::MatrixXX::Identity();
        double total_time = 0.0;
        for (const double interval: intervals) {
            const auto transform = transition(interval);
            covariance = transform * covariance * transform.transpose()
                + make_armor_process_noise(
                             interval,
                             POSITION_NOISE_DENSITY,
                             ACCELERATION_NOISE_DENSITY
                );
            total_time += interval;
        }
        const auto total_transform = transition(total_time);
        const ArmorStateEKF::MatrixXX one_prediction = total_transform * total_transform.transpose()
            + make_armor_process_noise(total_time,
                                       POSITION_NOISE_DENSITY,
                                       ACCELERATION_NOISE_DENSITY);
        EXPECT_TRUE(covariance.isApprox(one_prediction, 1e-12));
    }

    TEST(ArmorProcessNoise, StaticObservationNoiseProducesLessVelocityAndPitchLeadJitter) {
        auto time_scaled_filter = make_filter(true);
        auto legacy_filter = make_filter(false);
        double scaled_velocity_energy = 0.0;
        double legacy_velocity_energy = 0.0;
        double scaled_pitch_energy = 0.0;
        double legacy_pitch_energy = 0.0;
        for (int frame = 0; frame < 1200; ++frame) {
            const double time = frame * FRAME_DT;
            // 静止目标叠加 0.5 度、3 Hz 的俯仰观测扰动，模拟灯条定位抖动。
            const double observed_pitch = 0.5 * M_PI / 180.0 * std::sin(2.0 * M_PI * 3.0 * time);
            const ArmorStateEKF::MatrixZ1 measurement { 0.0, observed_pitch, TARGET_DISTANCE };
            time_scaled_filter.predict();
            legacy_filter.predict();
            const auto scaled_state = time_scaled_filter.update(measurement);
            const auto legacy_state = legacy_filter.update(measurement);
            if (time < 2.0) {
                continue;
            }
            // 同时检查速度以及 50 ms 图像/弹道提前后的俯仰扰动。
            const double scaled_pitch = std::atan2(
                scaled_state(4) + 0.05 * scaled_state(5),
                std::hypot(scaled_state(0) + 0.05 * scaled_state(1), scaled_state(2))
            );
            const double legacy_pitch = std::atan2(
                legacy_state(4) + 0.05 * legacy_state(5),
                std::hypot(legacy_state(0) + 0.05 * legacy_state(1), legacy_state(2))
            );
            scaled_velocity_energy += scaled_state(5) * scaled_state(5);
            legacy_velocity_energy += legacy_state(5) * legacy_state(5);
            scaled_pitch_energy += scaled_pitch * scaled_pitch;
            legacy_pitch_energy += legacy_pitch * legacy_pitch;
        }
        // 能量比小于 0.5，对应 RMS 抖动至少下降约 29%。
        EXPECT_LT(scaled_velocity_energy, legacy_velocity_energy * 0.5);
        EXPECT_LT(scaled_pitch_energy, legacy_pitch_energy * 0.5);
    }

    TEST(ArmorProcessNoise, TimeScaledNoiseStillTracksConstantVerticalMotion) {
        auto filter = make_filter(true);
        constexpr double TARGET_VELOCITY = 0.08;
        for (int frame = 1; frame <= 600; ++frame) {
            const double height = TARGET_VELOCITY * frame * FRAME_DT;
            const ArmorStateEKF::MatrixZ1 measurement { 0.0,
                                                        std::atan2(height, TARGET_DISTANCE),
                                                        std::hypot(TARGET_DISTANCE, height) };
            filter.predict();
            const auto state = filter.update(measurement);
            if (frame >= 300) {
                EXPECT_NEAR(state(4), height, 0.01);
                EXPECT_NEAR(state(5), TARGET_VELOCITY, 0.01);
            }
        }
    }

} // namespace
} // namespace qd::auto_aim

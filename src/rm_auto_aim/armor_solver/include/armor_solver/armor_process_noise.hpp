// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef ARMOR_SOLVER_ARMOR_PROCESS_NOISE_HPP_
#define ARMOR_SOLVER_ARMOR_PROCESS_NOISE_HPP_

#include "armor_solver/motion_model.hpp"

namespace qd::auto_aim {

/**
 * @brief 按实际帧间隔离散化平移模型的位置随机游走和连续白加速度噪声。
 * @param delta_t 预测时间间隔，单位为秒。
 * @param position_noise_density 位置随机游走的谱密度，单位为平方米每秒。
 * @param acceleration_noise_density 白加速度的谱密度，单位为平方米每立方秒。
 * @return 对应 [x, vx, y, vy, z, vz] 状态的过程噪声协方差。
 * @note 参数须有限且非负；同一总时长拆成不同数量的帧会得到相同累计噪声。
 */
inline ArmorStateEKF::MatrixXX make_armor_process_noise(
    double delta_t,
    double position_noise_density,
    double acceleration_noise_density
) {
    ArmorStateEKF::MatrixXX noise = ArmorStateEKF::MatrixXX::Zero();
    const double time_squared = delta_t * delta_t;
    const double position_variance = position_noise_density * delta_t
        + acceleration_noise_density * time_squared * delta_t / 3.0;
    const double position_velocity_covariance = acceleration_noise_density * time_squared / 2.0;
    const double velocity_variance = acceleration_noise_density * delta_t;
    for (int position_index = 0; position_index < X_N_; position_index += 2) {
        noise(position_index, position_index) = position_variance;
        noise(position_index, position_index + 1) = position_velocity_covariance;
        noise(position_index + 1, position_index) = position_velocity_covariance;
        noise(position_index + 1, position_index + 1) = velocity_variance;
    }
    return noise;
}

} // namespace qd::auto_aim

#endif // ARMOR_SOLVER_ARMOR_PROCESS_NOISE_HPP_

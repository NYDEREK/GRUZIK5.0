// =============================================================================
//  odometry.c - differential-drive dead reckoning
// =============================================================================
//
//  Distance comes from the average of both wheels. The heading change comes
//  from the gyro (96 %) blended with the wheel difference (4 %): the gyro
//  does not care about wheel slip, the small encoder share pulls the result
//  back if the gyro drifts. Without a gyro the encoders alone are used.
//
//  The position is integrated at the midpoint heading of each step, which is
//  noticeably more accurate in curves than using the old heading.
// =============================================================================

#include "odometry.h"

#include <math.h>
#include "config.h"

static odometry_t s_odom;

float normalize_angle(float angle_rad)
{
    while (angle_rad > (float)M_PI) angle_rad -= 2.0f * (float)M_PI;
    while (angle_rad <= -(float)M_PI) angle_rad += 2.0f * (float)M_PI;
    return angle_rad;
}

void odometry_reset(void)
{
    s_odom = (odometry_t){0};
}

void odometry_update(float left_delta_m, float right_delta_m,
                     float gyro_delta_yaw_rad, bool gyro_valid)
{
    const float distance = 0.5f * (left_delta_m + right_delta_m);
    const float encoder_yaw = (right_delta_m - left_delta_m) / WHEEL_BASE_M;
    const float gyro_yaw = gyro_valid ? gyro_delta_yaw_rad : encoder_yaw;
    const float fused = gyro_valid
                            ? ODOMETRY_GYRO_WEIGHT * gyro_yaw +
                                  (1.0f - ODOMETRY_GYRO_WEIGHT) * encoder_yaw
                            : encoder_yaw;

    const float midpoint_yaw = s_odom.yaw_rad + 0.5f * fused;
    s_odom.x_m += distance * cosf(midpoint_yaw);
    s_odom.y_m += distance * sinf(midpoint_yaw);
    s_odom.yaw_rad = normalize_angle(s_odom.yaw_rad + fused);
    s_odom.gyro_yaw_rad = normalize_angle(s_odom.gyro_yaw_rad + gyro_yaw);
    s_odom.encoder_yaw_rad = normalize_angle(s_odom.encoder_yaw_rad + encoder_yaw);
    s_odom.total_distance_m += fabsf(distance);
}

const odometry_t *odometry_get(void)
{
    return &s_odom;
}

#pragma once

// =============================================================================
//  config.h - tuning constants
// =============================================================================
//
//  Physical constants of the robot and the timing of every loop. Values the
//  user changes at runtime (PID gains, speeds, threshold) are not here; they
//  come from the app.
// =============================================================================

// --- Control loop ----------------------------------------------------------------
// Fixed 1 kHz tick: encoders, odometry, map recording/playback and motors.
#define CONTROL_PERIOD_MS               1
#define CONTROL_DT_S                    0.001f

// --- Line sensors ------------------------------------------------------------------
#define LINE_SENSOR_COUNT               16
// 74HC4067 output settling time after switching the channel.
#define LINE_MUX_SETTLE_US              6
// One ADC conversion takes ~53 us, so a full 16-channel sweep (~0.95 ms) does
// not fit one tick. Each tick reads half of the sensors; the line PID runs on
// every complete sweep, i.e. at 500 Hz.
#define LINE_CHANNELS_PER_TICK          8

// --- Line follower -----------------------------------------------------------------
// Kd is defined for an error difference taken every 0.5 ms. The PID updates
// every 2 ms, so the difference is scaled by 0.5 / 2 to keep Kd's meaning.
#define LINE_DERIVATIVE_SCALE           0.25f
// How long the last motor command is held when the line disappears inside a
// straight section (a gap in the line) before the edge search starts.
#define LINE_GAP_BRIDGE_MS              4000

// --- Motors --------------------------------------------------------------------------
#define MOTOR_PWM_FREQUENCY_HZ          20000
#define MOTOR_PWM_RESOLUTION_BITS       10
// Motor commands are in "PWM units" where 286 means 100 % duty. All speed
// settings in the app use this scale.
#define MOTOR_PWM_MAX                   286.0f

// --- Battery: 2S LiPo behind the 100k / 22k divider -------------------------------
#define BATTERY_DIVIDER                 (122.0f / 22.0f)
#define BATTERY_EMPTY_V                 6.6f
#define BATTERY_FULL_V                  8.4f
#define BATTERY_PRESENT_V               3.0f   // below this the robot runs on USB only
#define BATTERY_PERIOD_MS               250
// Start behaviour: warn below this voltage...
#define BATTERY_LOW_WARNING_V           7.0f
// ...and scale PWM up as the pack sags: level = (200 - V/Vref*100)/100 - offset.
#define SPEED_LEVEL_REFERENCE_V         8.48f
#define SPEED_LEVEL_OFFSET              0.014f

// --- Encoders and odometry --------------------------------------------------------
#define ENCODER_COUNTS_PER_WHEEL_REV    2560    // 128 CPR x 4 edges x 40/8 gear
#define ENCODER_LEFT_DIRECTION          (-1)    // makes forward motion positive
#define ENCODER_RIGHT_DIRECTION         1
#define ENCODER_SPEED_FILTER_ALPHA      0.35f   // low-pass on wheel speed
#define WHEEL_CIRCUMFERENCE_M           0.07225663f
#define WHEEL_BASE_M                    0.195f
#define SENSOR_OFFSET_M                 0.220f  // sensor bar ahead of the axle
#define ODOMETRY_GYRO_WEIGHT            0.96f   // yaw: 96 % gyro, 4 % encoders
#define GYRO_STATIONARY_DPS             2.0f    // below this, re-learn gyro bias
#define GYRO_BIAS_LEARN_RATE            0.0005f

// --- Mapping and playback ---------------------------------------------------------
#define MAP_RECORD_PERIOD_MS            40      // one route point every 40 ms
#define MAP_FINISH_PID_MS               1750    // PID run-out after the last point
#define MAP_TARGET_RADIUS_M             0.075f  // target counts as reached within this
#define MAP_ADVANCE_LATERAL_M           0.120f
#define MAP_CLOSE_RADIUS_M              0.030f  // loop closes this close to the start...
#define MAP_CLOSE_MIN_DISTANCE_M        0.450f  // ...after at least this distance...
#define MAP_CLOSE_MIN_POINTS            8       // ...and this many points
#define MAP_DEFAULT_SPEED               80.0f
#define PLAYBACK_LINE_WEIGHT            0.18f   // share of the line correction
#define PLAYBACK_LINE_CORR_LIMIT        35.0f
#define PLAYBACK_ODOM_CORR_LIMIT        115.0f
#define PLAYBACK_CORR_LIMIT             140.0f
#define PLAYBACK_LOOKAHEAD_BASE_M       0.120f
#define PLAYBACK_LOOKAHEAD_SPEED_M      0.140f
#define PLAYBACK_CROSSTRACK_LIMIT_M     0.180f
#define PLAYBACK_D_FILTER_ALPHA         0.28f
#define PLAYBACK_TURN_SPEED_REDUCTION   0.35f   // slow down in turns
#define PLAYBACK_LOST_LINE_SPEED_MUL    0.85f   // slow down without line contact
#define PLAYBACK_MAX_POINTS             4000

// --- Link and telemetry -------------------------------------------------------------
#define TELEMETRY_PERIOD_MS             100
#define TELEMETRY_BATTERY_PERIOD_MS     1000
#define LINK_HELLO_DELAY_MS             300     // let the app subscribe before hello

// --- Status LED -----------------------------------------------------------------------
#define LED_DRIVING_TOGGLE_MS           500     // 1 Hz while driving
#define LED_SEARCH_TOGGLE_MS            167     // 3 Hz without a phone

// =============================================================================
//  route_map.c - route recording, playback and map file transfer
// =============================================================================
//
//  File format (one point per line, tab separated):
//    GRUZIK.txt: x  y  sensor_x  sensor_y  speed_mps  line_error  d_error
//    map.txt:    x  y  speed_pwm
//
//  Writing to flash can take several milliseconds (sector erase), which would
//  break the 1 kHz control loop. Recorded points are therefore pushed into a
//  queue and written by a low-priority task on core 0.
// =============================================================================

#include "route_map.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "link.h"
#include "storage.h"

typedef struct {
    float x, y, speed;
} map_point_t;

typedef struct {
    bool close;             // last message of a recording
    float values[7];        // x, y, sensor x, sensor y, speed, error P, error D
} record_msg_t;

static map_params_t s_params = {.p = 90.0f, .i = 0.0f, .d = 10.0f,
                                .default_speed = MAP_DEFAULT_SPEED};

// --- Recording: state owned by the control task, file owned by the writer ---
static QueueHandle_t s_record_queue;
static FILE *s_record_file;
static struct {
    bool active;
    uint16_t points;
    float distance_m;
    float start_x, start_y, start_sx, start_sy;
    float last_sx, last_sy;
    float last_error;
    uint32_t last_record_ms;
} s_rec;

// --- Playback ----------------------------------------------------------------------
static map_point_t s_route[PLAYBACK_MAX_POINTS];
static int s_route_len;
static int s_target;
static float s_error_sum;
static float s_last_error;
static float s_filtered_derivative;

// --- Upload from the app (command task) -------------------------------------------
static FILE *s_upload_file;
static int s_uploaded;
static int s_upload_expected;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

map_params_t *route_map_params(void)
{
    return &s_params;
}

// Drains the record queue into GRUZIK.txt; flushes every 25 points so a
// power cut loses at most one second of route.
static void record_writer_task(void *arg)
{
    record_msg_t msg;
    int since_flush = 0;
    for (;;) {
        xQueueReceive(s_record_queue, &msg, portMAX_DELAY);
        FILE *file = s_record_file;
        if (file == NULL) {
            continue;
        }
        if (msg.close) {
            fclose(file);
            s_record_file = NULL;
            since_flush = 0;
            continue;
        }
        const float *v = msg.values;
        fprintf(file, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
        if (++since_flush >= 25) {
            fflush(file);
            since_flush = 0;
        }
    }
}

void route_map_init(void)
{
    s_record_queue = xQueueCreate(128, sizeof(record_msg_t));
    xTaskCreatePinnedToCore(record_writer_task, "map_writer", 4096, NULL, 3, NULL, 0);
}

static void queue_record(float x, float y, float sx, float sy, float speed,
                         float error_p, float error_d)
{
    const record_msg_t msg = {.values = {x, y, sx, sy, speed, error_p, error_d}};
    xQueueSend(s_record_queue, &msg, 0);  // never block the control loop; a full queue drops one point
    ++s_rec.points;
}

// Opens GRUZIK.txt and records the start pose. The sensor point starts one
// SENSOR_OFFSET_M ahead of the axle, since the pose is reset to yaw 0.
bool route_map_begin_recording(const odometry_t *odom)
{
    if (s_record_file != NULL) {
        return false;  // the writer is still finishing the previous recording
    }
    s_record_file = fopen(STORAGE_RECORDED_MAP_PATH, "w");
    if (s_record_file == NULL) {
        return false;
    }
    s_rec = (typeof(s_rec)){0};
    s_rec.active = true;
    s_rec.start_x = odom->x_m;
    s_rec.start_y = odom->y_m;
    s_rec.start_sx = s_rec.start_x + SENSOR_OFFSET_M;
    s_rec.start_sy = s_rec.start_y;
    s_rec.last_sx = s_rec.start_sx;
    s_rec.last_sy = s_rec.start_sy;
    queue_record(s_rec.start_x, s_rec.start_y, s_rec.start_sx, s_rec.start_sy, 0, 0, 0);
    return true;
}

bool route_map_record_tick(const odometry_t *odom, float speed_mps,
                           float line_error, uint32_t now_ms)
{
    if (!s_rec.active || now_ms - s_rec.last_record_ms < MAP_RECORD_PERIOD_MS) {
        return false;
    }
    s_rec.last_record_ms = now_ms;

    // The line under the sensor bar, not the axle, is what the route describes.
    const float sx = odom->x_m + SENSOR_OFFSET_M * cosf(odom->yaw_rad);
    const float sy = odom->y_m + SENSOR_OFFSET_M * sinf(odom->yaw_rad);
    s_rec.distance_m += hypotf(sx - s_rec.last_sx, sy - s_rec.last_sy);
    s_rec.last_sx = sx;
    s_rec.last_sy = sy;

    const float derivative = line_error - s_rec.last_error;
    s_rec.last_error = line_error;
    queue_record(odom->x_m, odom->y_m, sx, sy, speed_mps, line_error, derivative);

    // Close the loop once the robot is back at the start after a real lap.
    const float close = hypotf(odom->x_m - s_rec.start_x, odom->y_m - s_rec.start_y);
    if (s_rec.distance_m >= MAP_CLOSE_MIN_DISTANCE_M &&
        s_rec.points >= MAP_CLOSE_MIN_POINTS && close <= MAP_CLOSE_RADIUS_M) {
        queue_record(s_rec.start_x, s_rec.start_y, s_rec.start_sx, s_rec.start_sy,
                     speed_mps, line_error, derivative);
        route_map_end_recording();
        return true;
    }
    return false;
}

void route_map_end_recording(void)
{
    if (!s_rec.active) {
        return;
    }
    s_rec.active = false;
    const record_msg_t msg = {.close = true};
    xQueueSend(s_record_queue, &msg, portMAX_DELAY);
}

uint16_t route_map_recorded_points(void)
{
    return s_rec.points;
}

// Loads map.txt into RAM. Points without a speed use the default speed.
bool route_map_begin_playback(void)
{
    FILE *file = fopen(STORAGE_PLAYBACK_MAP_PATH, "r");
    if (file == NULL) {
        return false;
    }
    s_route_len = 0;
    char line[96];
    while (s_route_len < PLAYBACK_MAX_POINTS && fgets(line, sizeof(line), file)) {
        map_point_t p = {.speed = s_params.default_speed};
        const int parsed = sscanf(line, "%f%f%f", &p.x, &p.y, &p.speed);
        if (parsed >= 2) {
            p.speed = clampf(p.speed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            s_route[s_route_len++] = p;
        }
    }
    fclose(file);
    s_target = 0;
    s_error_sum = 0.0f;
    s_last_error = 0.0f;
    s_filtered_derivative = 0.0f;
    return s_route_len > 0;
}

// A target is done when the robot is close to it, or has clearly passed it
// along the current segment (projection beyond the end of the segment).
static bool should_advance(const odometry_t *odom, float distance_to_target)
{
    if (distance_to_target < MAP_TARGET_RADIUS_M) {
        return true;
    }
    if (s_target == 0 || s_target >= s_route_len) {
        return false;
    }
    const map_point_t *prev = &s_route[s_target - 1];
    const map_point_t *target = &s_route[s_target];
    const float seg_x = target->x - prev->x;
    const float seg_y = target->y - prev->y;
    const float seg_len_sq = seg_x * seg_x + seg_y * seg_y;
    if (seg_len_sq < 0.000001f) {
        return true;
    }
    const float pose_x = odom->x_m - prev->x;
    const float pose_y = odom->y_m - prev->y;
    const float projection = (pose_x * seg_x + pose_y * seg_y) / seg_len_sq;
    const float lateral = fabsf(pose_x * seg_y - pose_y * seg_x) / sqrtf(seg_len_sq);
    return (projection > 1.02f && lateral < MAP_ADVANCE_LATERAL_M) || projection > 1.30f;
}

bool route_map_playback_step(const odometry_t *odom, const line_output_t *line,
                             const line_status_t *line_status,
                             float *left_pwm, float *right_pwm)
{
    while (s_target < s_route_len) {
        const map_point_t *t = &s_route[s_target];
        if (!should_advance(odom, hypotf(t->x - odom->x_m, t->y - odom->y_m))) {
            break;
        }
        ++s_target;
    }
    if (s_target >= s_route_len) {
        return false;
    }

    // Heading error towards the current segment: the segment direction plus a
    // pull back onto the segment that grows with the cross-track error and
    // shrinks with speed (longer lookahead when faster).
    const map_point_t *target = &s_route[s_target];
    const float prev_x = s_target > 0 ? s_route[s_target - 1].x : 0.0f;
    const float prev_y = s_target > 0 ? s_route[s_target - 1].y : 0.0f;
    const float seg_x = target->x - prev_x;
    const float seg_y = target->y - prev_y;
    const float seg_len = sqrtf(seg_x * seg_x + seg_y * seg_y);
    float error;
    if (seg_len > 0.000001f) {
        const float pose_x = odom->x_m - prev_x;
        const float pose_y = odom->y_m - prev_y;
        const float cross_track = clampf((pose_x * seg_y - pose_y * seg_x) / seg_len,
                                         -PLAYBACK_CROSSTRACK_LIMIT_M,
                                         PLAYBACK_CROSSTRACK_LIMIT_M);
        const float speed_ratio = clampf(fabsf(target->speed) / MOTOR_PWM_MAX, 0.0f, 1.0f);
        const float lookahead = PLAYBACK_LOOKAHEAD_BASE_M + PLAYBACK_LOOKAHEAD_SPEED_M * speed_ratio;
        error = normalize_angle(atan2f(seg_y, seg_x) - odom->yaw_rad +
                                atan2f(cross_track, lookahead));
    } else {
        error = normalize_angle(atan2f(target->y - odom->y_m, target->x - odom->x_m) -
                                odom->yaw_rad);
    }

    // PID on the heading error, with a filtered derivative.
    s_error_sum = clampf(s_error_sum + error, -3.0f, 3.0f);
    const float derivative = normalize_angle(error - s_last_error);
    s_last_error = error;
    s_filtered_derivative += PLAYBACK_D_FILTER_ALPHA * (derivative - s_filtered_derivative);
    const float odom_limit = clampf(fabsf(target->speed) * 0.85f, 45.0f, PLAYBACK_ODOM_CORR_LIMIT);
    const float odom_correction = clampf(s_params.p * error + s_params.i * s_error_sum +
                                             s_params.d * s_filtered_derivative,
                                         -odom_limit, odom_limit);

    // Small line-sensor correction, trusted only when the line is near the
    // centre and narrow (a wide reading means a crossing or a marker).
    float line_correction = 0.0f;
    if (line->valid) {
        line_correction = clampf(0.5f * (line->left - line->right),
                                 -PLAYBACK_LINE_CORR_LIMIT, PLAYBACK_LINE_CORR_LIMIT);
    }
    float confidence = 0.0f;
    if (line_status->active_count > 0) {
        confidence = 1.0f - clampf((fabsf(line_status->error) - 1800.0f) / 3600.0f, 0.0f, 1.0f);
        if (line_status->active_count > 7) {
            confidence *= 0.45f;
        }
    }

    const float correction = clampf(-odom_correction +
                                        line_correction * PLAYBACK_LINE_WEIGHT * confidence,
                                    -PLAYBACK_CORR_LIMIT, PLAYBACK_CORR_LIMIT);
    // Slow down in turns and when the line is not visible.
    float base = target->speed * (1.0f - clampf(fabsf(error), 0.0f, 1.0f) *
                                             PLAYBACK_TURN_SPEED_REDUCTION);
    if (line_status->active_count == 0) {
        base *= PLAYBACK_LOST_LINE_SPEED_MUL;
    }
    *left_pwm = clampf(base + correction, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    *right_pwm = clampf(base - correction, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    return true;
}

// Sends GRUZIK.txt or map.txt to the app: MAP_BEGIN, one MAP line per point,
// MAP_END with the count.
void route_map_dump(bool optimized)
{
    const char *path = optimized ? STORAGE_PLAYBACK_MAP_PATH : STORAGE_RECORDED_MAP_PATH;
    const char *name = optimized ? "map.txt" : "GRUZIK.txt";
    FILE *file = fopen(path, "r");
    char line[128];
    if (file == NULL) {
        snprintf(line, sizeof(line), "MAP_ERROR,open_read,%s,4\r\n", name);
        link_send(line);
        return;
    }
    link_send_blocking(optimized ? "MAP_BEGIN,optimized\r\n" : "MAP_BEGIN,recorded\r\n");
    int count = 0;
    char out[140];
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] == '\0') {
            continue;
        }
        snprintf(out, sizeof(out), "MAP,%s\r\n", line);
        link_send_blocking(out);
        ++count;
    }
    fclose(file);
    snprintf(out, sizeof(out), "MAP_END,%d\r\n", count);
    link_send_blocking(out);
}

// map.txt upload: MapUploadBegin=N, then N x MapPoint=x,y,speed, MapUploadEnd.
void route_map_upload_begin(int expected_points)
{
    char msg[48];
    if (s_upload_file != NULL) {
        fclose(s_upload_file);
    }
    s_upload_file = fopen(STORAGE_PLAYBACK_MAP_PATH, "w");
    if (s_upload_file == NULL) {
        link_send("UPLOAD_ERROR,open_write,map.txt,1\r\n");
        return;
    }
    s_uploaded = 0;
    s_upload_expected = expected_points;
    snprintf(msg, sizeof(msg), "UPLOAD_READY,%d\r\n", expected_points);
    link_send(msg);
}

void route_map_upload_point(const char *value)
{
    char msg[48];
    if (s_upload_file == NULL) {
        link_send("UPLOAD_ERROR,no_file\r\n");
        return;
    }
    map_point_t p = {.speed = s_params.default_speed};
    if (sscanf(value, "%f,%f,%f", &p.x, &p.y, &p.speed) < 2) {
        link_send("UPLOAD_ERROR,bad_point\r\n");
        return;
    }
    fprintf(s_upload_file, "%.3f\t%.3f\t%.3f\n", p.x, p.y, p.speed);
    if (++s_uploaded % 50 == 0) {
        snprintf(msg, sizeof(msg), "UPLOAD_PROGRESS,%d,%d\r\n", s_uploaded, s_upload_expected);
        link_send(msg);
    }
}

void route_map_upload_end(void)
{
    char msg[48];
    if (s_upload_file != NULL) {
        fclose(s_upload_file);
        s_upload_file = NULL;
    }
    snprintf(msg, sizeof(msg), "UPLOAD_DONE,%d,%d\r\n", s_uploaded, s_upload_expected);
    link_send(msg);
}

// Appends the current position as a map point (building a map by hand).
void route_map_append_point(float x, float y)
{
    FILE *file = fopen(STORAGE_PLAYBACK_MAP_PATH, "a");
    if (file != NULL) {
        fprintf(file, "%.3f\t%.3f\t%.3f\n", x, y, s_params.default_speed);
        fclose(file);
    }
}

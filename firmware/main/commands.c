// =============================================================================
//  commands.c - text command interpreter
// =============================================================================
//
//  Command groups:
//    parameters   Kp, Kd, Base_speed, ..., MapP, CleanSpeed, Treshold
//    runs         StartNormal/Mapping/Playback, Mode=N|Y|P|M|U, State=...
//    manual       Manual=L,R  (joystick), Clean=0|1 (tyre cleaning)
//    telemetry    Telemetry=off|odom|debug
//    maps         MapDump, MapUploadBegin/MapPoint/MapUploadEnd,
//                 Add_Simple_Map_Point
//    status       ImuStatus
// =============================================================================

#include "commands.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "imu.h"
#include "line_follower.h"
#include "link.h"
#include "odometry.h"
#include "robot.h"
#include "route_map.h"

// The app still sends turbine settings; this robot has no turbine, the
// values are only stored.
static float s_turbine_speed;
static float s_turbine_prep_time;

static void trim(char *s)
{
    char *start = s;
    while (isspace((unsigned char)*start)) ++start;
    memmove(s, start, strlen(start) + 1);
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) s[--len] = '\0';
}

static int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Commands that only store a value. The table maps a key to a float field of
// the line follower or map parameters. Returns false if the key is not a
// parameter.
static bool set_parameter(const char *key, const char *value)
{
    line_params_t *lf = line_follower_params();
    map_params_t *map = route_map_params();
    const float f = strtof(value, NULL);
    static const struct {
        const char *key;
        size_t offset;
        bool line;
    } fields[] = {
        {"Kp", offsetof(line_params_t, kp), true},
        {"Kd", offsetof(line_params_t, kd), true},
        {"Base_speed", offsetof(line_params_t, base_speed), true},
        {"Max_speed", offsetof(line_params_t, max_speed), true},
        {"Sharp_bend_speed_right", offsetof(line_params_t, sharp_right), true},
        {"Sharp_bend_speed_left", offsetof(line_params_t, sharp_left), true},
        {"Bend_speed_right", offsetof(line_params_t, bend_right), true},
        {"Bend_speed_left", offsetof(line_params_t, bend_left), true},
        {"MapP", offsetof(map_params_t, p), false},
        {"MapI", offsetof(map_params_t, i), false},
        {"MapD", offsetof(map_params_t, d), false},
        {"MapSpeed", offsetof(map_params_t, default_speed), false},
    };
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (strcmp(key, fields[i].key) == 0) {
            char *base = fields[i].line ? (char *)lf : (char *)map;
            *(float *)(base + fields[i].offset) = f;
            return true;
        }
    }
    if (strcmp(key, "Treshold") == 0) {
        lf->threshold = (uint16_t)clamp_int(atoi(value), 0, 4095);
    } else if (strcmp(key, "Turbine_Speed") == 0) {
        s_turbine_speed = f;
    } else if (strcmp(key, "Turbine_Prep_Time") == 0) {
        s_turbine_prep_time = f;
    } else if (strcmp(key, "CleanSpeed") == 0 || strcmp(key, "TireCleanSpeed") == 0) {
        robot_set_clean_speed(f);
    } else {
        return false;
    }
    return true;
}

static void command_mode(const char *value)
{
    switch (value[0] ? value[0] : 'N') {
    case 'N': robot_stop(); break;
    case 'Y':
    case 'C': robot_start_selected(); break;
    case 'P': robot_select(ROBOT_SELECT_NORMAL); break;
    case 'M': robot_select(ROBOT_SELECT_MAPPING); break;
    case 'U': robot_select(ROBOT_SELECT_PLAYBACK); break;
    default: break;
    }
}

static void command_state(const char *value)
{
    if (!strcmp(value, "PID") || !strcmp(value, "Pid") || !strcmp(value, "P")) {
        robot_select(ROBOT_SELECT_NORMAL);
    } else if (!strcmp(value, "Mapping") || !strcmp(value, "M")) {
        robot_select(ROBOT_SELECT_MAPPING);
    } else if (!strcmp(value, "UnMapping") || !strcmp(value, "U")) {
        robot_select(ROBOT_SELECT_PLAYBACK);
    }
}

static void command_telemetry(const char *value)
{
    char imu[96];
    imu_format_status(imu, sizeof(imu));
    char msg[128];
    if (!strcasecmp(value, "odom") || !strcmp(value, "1")) {
        robot_set_telemetry(TELEMETRY_ODOM);
        snprintf(msg, sizeof(msg), "TELEMETRY,odom\r\n%s", imu);
    } else if (!strcasecmp(value, "debug") || !strcmp(value, "2")) {
        robot_set_telemetry(TELEMETRY_DEBUG);
        snprintf(msg, sizeof(msg), "TELEMETRY,debug\r\n%s", imu);
    } else {
        robot_set_telemetry(TELEMETRY_OFF);
        snprintf(msg, sizeof(msg), "TELEMETRY,off\r\n");
    }
    link_send(msg);
}

// Map transfers do file I/O and many sends; they run without the robot lock
// and only while the robot is stopped.
static bool run_map_transfer(const char *key, const char *value)
{
    const bool is_transfer = !strcmp(key, "MapDump") || !strcmp(key, "MapUploadBegin") ||
                             !strcmp(key, "MapPoint") || !strcmp(key, "MapUploadEnd");
    if (!is_transfer) {
        return false;
    }
    robot_lock();
    const bool stopped = robot_mode() == ROBOT_STOPPED;
    robot_unlock();
    if (!stopped && strcmp(key, "MapPoint") != 0 && strcmp(key, "MapUploadEnd") != 0) {
        link_send(!strcmp(key, "MapDump") ? "MAP_ERROR,stop_robot_first\r\n"
                                          : "UPLOAD_ERROR,stop_robot_first\r\n");
        return true;
    }
    if (!strcmp(key, "MapDump")) {
        route_map_dump(!strcasecmp(value, "optimized"));
    } else if (!strcmp(key, "MapUploadBegin")) {
        route_map_upload_begin(clamp_int(atoi(value), 0, 65535));
    } else if (!strcmp(key, "MapPoint")) {
        route_map_upload_point(value);
    } else {
        route_map_upload_end();
    }
    return true;
}

// Splits "Key=Value" and dispatches it. Everything except map transfers runs
// with the robot lock held.
void commands_execute(const char *line)
{
    char command[256];
    strlcpy(command, line, sizeof(command));
    trim(command);
    if (command[0] == '\0') {
        return;
    }
    char *value = strchr(command, '=');
    if (value != NULL) {
        *value++ = '\0';
    } else {
        value = command + strlen(command);
    }
    const char *key = command;

    if (run_map_transfer(key, value)) {
        return;
    }

    robot_lock();
    if (set_parameter(key, value)) {
        // value stored
    } else if (!strcmp(key, "StartNormal")) {
        robot_start_normal();
    } else if (!strcmp(key, "StartMapping")) {
        robot_start_mapping();
    } else if (!strcmp(key, "StartPlayback")) {
        robot_start_playback();
    } else if (!strcmp(key, "Mode")) {
        command_mode(value);
    } else if (!strcmp(key, "State")) {
        command_state(value);
    } else if (!strcmp(key, "Manual") || !strcmp(key, "Joystick")) {
        float left = 0.0f, right = 0.0f;
        if (sscanf(value, "%f,%f", &left, &right) == 2) {
            robot_manual(left, right);
        }
    } else if (!strcmp(key, "Clean") || !strcmp(key, "TireClean")) {
        robot_clean(atoi(value) > 0);
    } else if (!strcmp(key, "Telemetry") || !strcmp(key, "Debug")) {
        command_telemetry(value);
    } else if (!strcmp(key, "Add_Simple_Map_Point")) {
        const odometry_t *odom = odometry_get();
        route_map_append_point(odom->x_m, odom->y_m);
    } else if (!strncmp(key, "BtName", 6) || !strncmp(key, "BTName", 6) ||
               !strncmp(key, "BluetoothName", 13)) {
        link_send("BT_NAME_OK,gruzik 5.0,0\r\n");  // the name is fixed; acknowledge only
    } else if (!strcmp(key, "ImuStatus")) {
        char imu[96];
        imu_format_status(imu, sizeof(imu));
        link_send(imu);
    }
    robot_unlock();
}

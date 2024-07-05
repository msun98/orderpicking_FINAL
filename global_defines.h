#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

//#define USE_SIM
//#define USE_EX_TEMP

//#define USE_SINGLE_S1
//#define USE_DUAL_S1
//#define USE_DUAL_S3
//#define USE_DUAL_SICK
#define USE_DUAL_LAKI

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se2.hpp>
#include <sophus/interpolate.hpp>

// stl
#include <vector>
#include <stack>
#include <mutex>
#include <thread>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/ximgproc.hpp>
#include "cv_to_qt.h"

// nanoflann
#include "nanoflann.hpp"

// logger
#include "Logger.h"
extern Logger logger;

// qt
#include <QString>

// std::pair
#include <utility>


// defines
#define ACC_G 9.80665
#define N2S (1.0e-9) // nanosec to sec
#define S2N (1.0e9) // sec to nanosec
#define U2S (1.0e-6) // microsec to sec
#define S2U (1.0e6) // sec to microsec
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define toWrap(rad) (std::atan2(std::sin(rad), std::cos(rad)))
#define deltaRad(ed,st) (std::atan2(std::sin(ed - st), std::cos(ed - st)))

// for algorithm
#define P_hit 0.6
#define P_miss 0.495
#define P_min 0.1
#define P_max 0.95
#define WALL_THRESHOLD 0.8
#define PATH_STEP 0.01

enum UI_MOTOR_STATE
{
    UI_MOTOR_NOT_READY = 0,
    UI_MOTOR_READY,
};

enum UI_LOC_STATE
{
    UI_LOC_NOT_READY = 0,
    UI_LOC_BUSY,
    UI_LOC_GOOD,
    UI_LOC_FAIL,
    UI_LOC_MANUAL,
    UI_LOC_GOOD_BUT_FAR_WAY,
};

enum UI_AUTO_STATE
{
    UI_AUTO_NOT_READY = 0,
    UI_AUTO_READY,
    UI_AUTO_MOVING,
    UI_AUTO_WAIT,
    UI_AUTO_PAUSE
};

enum UI_AUTO_FAIL_STATE
{
    UI_AUTO_NO_FAILED = 0,
    UI_AUTO_FAILED,
};

enum UI_MULTI_STATE
{
    UI_MULTI_READY = 0,
    UI_MULTI_MOVING,
    UI_MULTI_WAIT,
};

enum OBS_STATE
{
    OBS_NONE = 0,
    OBS_DETECTED,
};

enum UI_FACE_STATE
{
    UI_FACE_NORMAL = 0,
    UI_FACE_SURPRISE,
    UI_FACE_CRYING,
};

enum UI_MOTOR_LOCK_STATE
{
    UI_MOTOR_LOCK_OFF = 0,
    UI_MOTOR_LOCK_ON
};

enum UI_DRAW_STATE
{
    UI_DRAW_STOP = 0,
    UI_DRAW_START,
};

enum LED_STATE
{
    LED_OFF = 0,
    LED_RED,
    LED_WHITE,
    LED_CYAN,
    LED_CYAN_DIM,
    LED_MAGENTA
};

enum COMMAND
{
    CMD_MOVE_TARGET0 = 1,
    CMD_MOVE_TARGET,
    CMD_MOVE_JOYSTICK,
    CMD_MOVE_MANUAL,
    CMD_MOVE_STOP, //5
    CMD_PAUSE,
    CMD_RESUME,
    CMD_SET_VEL,
    CMD_RESTART,
    CMD_SET_INIT, //10
    CMD_LOC_RUN,
    CMD_LOC_STOP,
    CMD_LOC_AUTO,
    CMD_LOC_AUTO_FULL,
    CMD_MAPPING_START, //15
    CMD_MAPPING_STOP,
    CMD_REQ_CAMERA,
    CMD_MAP_SAVE,
    CMD_OBS_START,
    CMD_OBS_STOP, //20
    CMD_OBS_SAVE,
    CMD_NONE,
    CMD_MOTOR_LOCK_ON,
    CMD_DRAW_START,
    CMD_DRAW_STOP_SAVE, // 25
    CMD_ROBOT_MAP_HARD_RELOAD,
    CMD_ROBOT_CONFIG_RELOAD,
    CMD_FMS_SEND_MAP,
    CMD_CHECK_TRAVEL,
    CMD_ROBOT_MAP_SOFT_RELOAD, // 30
    CMD_MANUAL_LC,
    CMD_LOC_AUTO_RESTING,
    CMD_MOTOR_LOCK_OFF,
};

enum AUTO_FSM_STATE
{
    STATE_AUTO_PATH_FINDING = 0,
    STATE_AUTO_FIRST_ALIGN,
    STATE_AUTO_PURE_PURSUIT,
    STATE_AUTO_FINAL_ALIGN,
    STATE_AUTO_GOAL_REACHED,
    STATE_AUTO_OBSTACLE,
    STATE_AUTO_PAUSE,
    STATE_AUTO_FAILED,
};

enum PRESET_SPEED
{
    PRESET_SPEED_SLOWEST = 1,
    PRESET_SPEED_SLOW,
    PRESET_SPEED_NORMAL,
    PRESET_SPEED_FAST,
    PRESET_SPEED_FASTEST
};

enum PRESET_SPEED_ZONE
{
    SLOWEST_ZONE = 200,
    SLOW_ZONE = 100,
    NORMAL_ZONE = 0,
};

struct DEBUG_VAL
{
    bool DEBUG_VIRTUAL_WALL_IN_ROBOT_RADIUS=false;
    bool DEBUG_STOP_BY_VIRTURE_WALL=false;
    bool DEBUG_DYNAMIC_OBS_BY_CAM=false;
    bool DEBUG_DYNAMIC_OBS_BY_LIDAR=false;
};

struct UPDATE_CONFIG
{
    // motor
    int motor_left_id = 1;
    int motor_right_id = 0;
    double motor_wheel_dir = -1.0;
    double motor_gear_ratio = 1.0;
    double motor_k_p = 100.0;
    double motor_k_i = 0.0;
    double motor_k_d = 4400.0;
    double motor_limit_v = 2.0;
    double motor_limit_v_acc = 1.5;
    double motor_limit_w = 180.0*D2R;
    double motor_limit_w_acc = 180*D2R;

    // driving
    double robot_min_look_ahead_dist = 0.15;
    double robot_look_ahead_dist = 1.0;
    double robot_goal_near_dist = 0.3;
    double robot_goal_near_th = 15.0*D2R;
    double robot_goal_dist = 0.1;
    double robot_goal_th = 3.0*D2R;
    double robot_goal_v = 0.05;
    double robot_st_v = 0.05;
    double robot_path_out_dist = 1.0;
    double robot_path_delta_v_acc_gain = 1.0;
    double robot_path_delta_v_dec_gain = 0.5;
    double robot_path_ref_v_gain = 1.0;
    double robot_k_v = 1.5;
    double robot_k_w = 1.0;
    double robot_manual_limit_v = 0.3;
    double robot_manual_limit_w = 30.0*D2R;

    // localization
    double robot_icp_dist = 0.2;
    double robot_icp_error = 0.1;
    double robot_icp_ratio = 0.5;
    double robot_icp_near = 1.0;
    double robot_icp_repeat_dist = 0.1;
    double robot_icp_repeat_time = 0.3;
    double robot_icp_odometry_weight = 0.8;

    // slam
    double robot_slam_lc_dist = 5.0;
    double robot_slam_lc_icp_dist = 0.5;
    int robot_slam_submap_cnt = 100;
    int robot_map_size = 1000;
    double robot_grid_size = 0.05;
};
extern UPDATE_CONFIG update_config;

struct SETTING_CONFIG
{
    // read file
    QString map_name = "";
    QString map_path = "";

    // robot type
    QString robot_model = "RB_SRV";
    QString robot_sn = "1";
    QString robot_type = "SERVING";

    // server
    QString fms_ip = "";
    QString fms_id = "";
    QString fms_pw = "";

    // use_slam flag
    bool robot_use_obs_preview = false;
    bool robot_use_obs_near = false;
    bool robot_use_early_stop_resting = false;
    bool robot_use_early_stop_serving = false;
    bool robot_use_multi = false;
    bool robot_use_ignore_safetyzone = false;
    bool robot_use_ccma = true;

    // preset speed
    double robot_preset_limit_v[6] = {0.6,};
    double robot_preset_limit_w[6] = {75.0*D2R,};
    double robot_preset_limit_v_acc[6] = {0.3,};
    double robot_preset_limit_w_acc[6] = {90.0*D2R,};
    double robot_preset_limit_pivot[6] = {30.0*D2R,};
    double robot_preset_limit_pivot_acc[6] = {60.0*D2R,};

    // OBSTACLE
    double robot_obs_deadzone = 0.4;
    double robot_static_obs_margin = 0.05;
    double robot_dynamic_obs_margin = 0.05;
    double robot_obs_check_range = 2.5;
    double robot_obs_early_stop_dist = 1.0;
    double robot_obs_cam_height_min = 0.1;
    double robot_obs_cam_height_max = 1.0;
    double robot_obs_wait_time = 1.0;
    double robot_obs_near_dist = 1.0;
    double robot_obs_preview_time = 3.0;
    double robot_obs_decel_gain = 0.8;
    int robot_obs_detect_area = 2;
    int robot_obs_detect_sensitivity = 2;

    // sensor
    double robot_lidar_max_range = 40.0;
    double robot_lidar_mask = 0.0;

    // initialization
    double robot_icp_ratio0 = 0.7;
    double robot_icp_error0 = 0.2;
};
extern SETTING_CONFIG setting_config;

struct STATIC_CONFIG
{
    // ROBOT_HW
    double robot_radius = 0.3;
    double robot_length = 0.52;
    double robot_length_x = 0.52;
    double robot_length_y = 0.52;
    double robot_wheel_base = 0.3542;
    double robot_wheel_radius = 0.0635;

    double robot_lidar_offset_x = 0.0;
    double robot_lidar_offset_y = 0.0;
    double robot_lidar_offset_th = 0.0;

    // SENSOR
    QString robot_lidar_tf = "";
    QString robot_cam_left_sn = "";
    QString robot_cam_left_tf = "";
    QString robot_cam_right_sn = "";
    QString robot_cam_right_tf = "";
};
extern STATIC_CONFIG static_config;

struct MOBILE_POSE
{
    double t;
    cv::Vec3d pose; // global (x, y, th)
    cv::Vec3d vel;  // global (x_dot, y_dot, th_dot)
    cv::Vec2d vw;   // local (v, w)

    MOBILE_POSE()
    {
        t = 0;
        pose = cv::Vec3d(0,0,0);
        vel = cv::Vec3d(0,0,0);
        vw = cv::Vec2d(0,0);
    }
    MOBILE_POSE(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
        vw = p.vw;
    }
};

/*struct MOBILE_STATUS
{
    bool is_ok = false;

    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;

    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;

    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;

    uint8_t temp_ex_m0 = 0;
    uint8_t temp_ex_m1 = 0;

    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;

    uint8_t charge_state = 0;
    uint8_t power_state = 0;
    uint8_t emo_state = 0;
    uint8_t remote_state = 0;

    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float power = 0;
    float total_power = 0;
};*/

struct MOBILE_STATUS
{
    bool is_ok = false;

    double t = 0;

    // motor status
    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;

    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;

    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;

    uint8_t temp_ex_m0 = 0;
    uint8_t temp_ex_m1 = 0;

    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;

    uint8_t charge_state = 0;
    uint8_t power_state = 0;
    uint8_t emo_state = 0;
    uint8_t remote_state = 0;

    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float power = 0;
    float total_power = 0;

    // for timesync
    uint32_t recv_tick = 0;
    float return_time = 0;

    // roller status
    uint8_t roller_controller_state = 0;
    uint8_t roller_sensor0 = 0;
    uint8_t roller_sensor1 = 0;
    uint8_t roller_sensor2 = 0;
    uint8_t roller_sensor3 = 0;
    uint8_t roller_manual_sw0 = 0;
    uint8_t roller_manual_sw1 = 0;
    uint8_t roller_blocking_state0 = 0;
    uint8_t roller_blocking_state1 = 0;
    uint8_t roller_blocking_manual_sw0 = 0;
    uint8_t roller_blocking_manual_sw1 = 0;
    uint8_t roller_blocking_manual_sw2 = 0;
    uint8_t roller_blocking_manual_sw3 = 0;
    uint8_t orgo_on_init = 0;
    uint8_t orgo_on_run = 0;
    uint8_t orgo_pos_state0 = 0;
    uint8_t orgo_pos_state1 = 0;
    uint8_t orgo_manual_sw0 = 0;
    uint8_t orgo_manual_sw1 = 0;
    uint32_t orgo_pos0 = 0;
    uint32_t orgo_pos1 = 0;

    // imu status
    float imu_gyr_x =0;
    float imu_gyr_y =0;
    float imu_gyr_z =0;
    float imu_acc_x =0;
    float imu_acc_y =0;
    float imu_acc_z =0;
};

struct LIDAR_FRM
{
    double t = 0;
    double t0 = 0;
    double t1 = 0;
    std::vector<double> ts;    
    std::vector<cv::Vec2d> pts;
    cv::Vec3d mobile_pose;
    cv::Vec3d pre_mobile_pose;

    LIDAR_FRM()
    {
        mobile_pose = cv::Vec3d(0,0,0);
        pre_mobile_pose = cv::Vec3d(0,0,0);
    }
    LIDAR_FRM(const LIDAR_FRM& p)
    {
        t = p.t;
        t0 = p.t0;
        t1 = p.t1;
        ts = p.ts;        
        pts = p.pts;
        mobile_pose = p.mobile_pose;
        pre_mobile_pose = p.pre_mobile_pose;
    }
};

struct LOC
{
    cv::Vec3d pose;
    cv::Vec3d mobile_pose;
    LOC()
    {
        pose = cv::Vec3d(0,0,0);
        mobile_pose = cv::Vec3d(0,0,0);
    }
    LOC(const LOC& p)
    {
        pose = p.pose;
        mobile_pose = p.mobile_pose;
    }
};

struct LC_INFO
{
    int id0 = 0;
    int id1 = 0;
    cv::Vec3d dxi;

    LC_INFO()
    {
        dxi = cv::Vec3d(0,0,0);
    }
    LC_INFO(const LC_INFO& p)
    {
        id0 = p.id0;
        id1 = p.id1;
        dxi = p.dxi;
    }
};

struct PATH_POINT
{
    cv::Vec2d pt;
    double od = 0;
    double th = 0;
    double v = 0;
    double obs_v = 0;
    int obs = 0;
    int preset_idx = PRESET_SPEED_NORMAL;

    PATH_POINT()
    {
        pt = cv::Vec2d(0, 0);
        od = 0;
        th = 0;
        v = 0;
        obs = 0;
        obs_v = 0;
        preset_idx = PRESET_SPEED_NORMAL;
    }
    PATH_POINT(const PATH_POINT& p)
    {
        pt = p.pt;
        od = p.od;
        th = p.th;
        v = p.v;
        obs = p.obs;
        obs_v = p.obs_v;
        preset_idx = p.preset_idx;
    }
};

struct NODE
{
    QString id;
    QString attrib; // Route, Charging, Resting, Cleaning, Serving, Temp
    cv::Vec3d pose;
    std::vector<QString> linked;

    NODE()
    {
        id = "";
        attrib = "";
        pose = cv::Vec3d(0,0,0);
        linked.clear();
    }

    NODE(const NODE& p)
    {
        id = p.id;
        attrib = p.attrib;
        pose = p.pose;
        linked = p.linked;
    }

    NODE& operator=(const NODE& p)
    {
        id = p.id;
        attrib = p.attrib;
        pose = p.pose;
        linked = p.linked;
        return *this;
    }
};

struct ASTAR_NODE
{
    ASTAR_NODE* parent = NULL;
    NODE* node = NULL;
    cv::Vec2i pos;
    double g = 0;
    double h = 0;
    double f = 0;

    ASTAR_NODE()
    {
        parent = NULL;
        node = NULL;
        pos = cv::Vec2i(0,0);
        g = 0;
        h = 0;
        f = 0;
    }

    ASTAR_NODE(const ASTAR_NODE& p)
    {
        parent = p.parent;
        node = p.node;
        pos = p.pos;
        g = p.g;
        h = p.h;
        f = p.f;
    }

    ASTAR_NODE& operator=(const ASTAR_NODE& p)
    {
        parent = p.parent;
        node = p.node;
        pos = p.pos;
        g = p.g;
        h = p.h;
        f = p.f;
        return *this;
    }
};

struct TIME_POSE
{
    double t = 0;
    cv::Vec3d pose;

    TIME_POSE()
    {
        pose = cv::Vec3d(0,0,0);
    }
    TIME_POSE(const TIME_POSE& p)
    {
        t = p.t;
        pose = p.pose;
    }
};

struct CTS_STATUS
{
    QString id;
    QString state;
    cv::Vec3d pose;
    cv::Vec3d goal;
    std::vector<cv::Vec2d> path;
    std::vector<cv::Vec2d> obs;
    int tick;

    CTS_STATUS()
    {
        id = "";
        state = "path_complete"; // path_req, path_driving, path_complete
        pose = cv::Vec3d(0,0,0);
        goal = cv::Vec3d(0,0,0);
        tick = 0;
    }
    CTS_STATUS(const CTS_STATUS& p)
    {
        id = p.id;
        state = p.state;
        pose = p.pose;
        goal = p.goal;
        path = p.path;
        obs = p.obs;
        tick = p.tick;
    }
    CTS_STATUS& operator=(const CTS_STATUS& p)
    {
        id = p.id;
        state = p.state;
        pose = p.pose;
        goal = p.goal;
        path = p.path;
        obs = p.obs;
        tick = p.tick;
        return *this;
    }
};

struct CMD
{
    uint32_t tick = 0;
    int32_t cmd = 0;
    uint8_t params[255] = {0,};

    CMD()
    {
    }
    CMD(const CMD& p)
    {
        tick = p.tick;
        cmd = p.cmd;
        memcpy(params, p.params, 255);
    }
};

// integrate_ui (orderPicking)
struct ipInfo
{
    QString ip = "";
    int port = -1;
};

extern double st_time_for_get_time;
double get_time();
void update_robot_config();

//universal function
std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r);
std::vector<cv::Vec2i> circle_iterator(cv::Vec2i pt, int r);
std::vector<cv::Vec2i> spiral_iterator(cv::Vec2i pt, int r);
std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1);

#endif // GLOBAL_DEFINES_H




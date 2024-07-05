#include "mainwindow.h"

#include <QApplication>

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(cv::Vec3d)
Q_DECLARE_METATYPE(std::vector<cv::Vec3d>)
Q_DECLARE_METATYPE(MOBILE_POSE)
Q_DECLARE_METATYPE(MOBILE_STATUS)
Q_DECLARE_METATYPE(CMD)

UPDATE_CONFIG update_config;
SETTING_CONFIG setting_config;
STATIC_CONFIG static_config;

Logger logger;

void update_robot_config();

double st_time_for_get_time = get_time();
double get_time()
{
    std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    return (timestamp*1.0e-9) - st_time_for_get_time;
}

void update_robot_config()
{
    // read config
    QString update_path = QDir::homePath()+"/RB_MOBILE/config/update_config.ini";
    QString setting_path = QDir::homePath()+"/RB_MOBILE/config/setting_config.ini";
    QString static_path = QDir::homePath()+"/RB_MOBILE/config/static_config.ini";

    QFileInfo update_info(update_path);
    QFileInfo setting_info(setting_path);
    QFileInfo static_info(static_path);

    bool is_all_exist = true;
    if((update_info.exists() && update_info.isFile()) && (setting_info.exists() && setting_info.isFile()) && (static_info.exists() && static_info.isFile()))
    {
        QSettings up_settings(update_path, QSettings::IniFormat);
        QSettings se_settings(setting_path, QSettings::IniFormat);
        QSettings st_settings(static_path, QSettings::IniFormat);

        // UPDATE CONFIG
        // driving
        update_config.robot_manual_limit_v = up_settings.value("DRIVING/limit_manual_v", -99.0).toDouble();
        if(update_config.robot_manual_limit_v == -99.0)
        {
            logger.write("[UPDATE] robot_manual_limit_v doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_manual_limit_w = up_settings.value("DRIVING/limit_manual_w", -99.0).toDouble()*D2R;
        if(update_config.robot_manual_limit_w == -99.0*D2R)
        {
            logger.write("[UPDATE] robot_manual_limit_w doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_k_v = up_settings.value("DRIVING/k_v", -99.0).toDouble();
        if(update_config.robot_k_v == -99.0)
        {
            logger.write("[UPDATE] robot_k_v doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_k_w = up_settings.value("DRIVING/k_w", -99.0).toDouble();
        if(update_config.robot_k_w == -99.0)
        {
            logger.write("[UPDATE] robot_k_w doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_min_look_ahead_dist = up_settings.value("DRIVING/min_look_ahead_dist", -99.0).toDouble();
        if(update_config.robot_min_look_ahead_dist == -99.0)
        {
            logger.write("[UPDATE] robot_min_look_ahead_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_look_ahead_dist = up_settings.value("DRIVING/look_ahead_dist", -99.0).toDouble();
        if(update_config.robot_look_ahead_dist == -99.0)
        {
            logger.write("[UPDATE] robot_look_ahead_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_goal_near_dist = up_settings.value("DRIVING/goal_near_dist", -99.0).toDouble();
        if(update_config.robot_goal_near_dist == -99.0)
        {
            logger.write("[UPDATE] robot_goal_near_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_goal_near_th = up_settings.value("DRIVING/goal_near_th", -99.0).toDouble()*D2R;
        if(update_config.robot_goal_near_th == -99.0*D2R*D2R)
        {
            logger.write("[UPDATE] robot_goal_near_th doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_goal_dist = up_settings.value("DRIVING/goal_dist", -99.0).toDouble();
        if(update_config.robot_goal_dist == -99.0)
        {
            logger.write("[UPDATE] robot_goal_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_goal_th = up_settings.value("DRIVING/goal_th", -99.0).toDouble()*D2R;
        if(update_config.robot_goal_th == -99.0*D2R)
        {
            logger.write("[UPDATE] robot_goal_th doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_goal_v = up_settings.value("DRIVING/goal_v", -99.0).toDouble();
        if(update_config.robot_goal_v == -99.0)
        {
            logger.write("[UPDATE] robot_goal_v doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_st_v = up_settings.value("DRIVING/st_v", -99.0).toDouble();
        if(update_config.robot_st_v == -99.0)
        {
            logger.write("[UPDATE] robot_st_v doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_path_out_dist = up_settings.value("DRIVING/path_out_dist", -99.0).toDouble();
        if(update_config.robot_path_out_dist == -99.0)
        {
            logger.write("[UPDATE] robot_path_out_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_path_delta_v_acc_gain = up_settings.value("DRIVING/path_delta_v_acc_gain", -99.0).toDouble();
        if(update_config.robot_path_delta_v_acc_gain == -99.0)
        {
            logger.write("[UPDATE] robot_path_delta_v_acc_gain doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_path_delta_v_dec_gain = up_settings.value("DRIVING/path_delta_v_dec_gain", -99.0).toDouble();
        if(update_config.robot_path_delta_v_dec_gain == -99.0)
        {
            logger.write("[UPDATE] robot_path_delta_v_dec_gain doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_path_ref_v_gain = up_settings.value("DRIVING/path_ref_v_gain", -99.0).toDouble();
        if(update_config.robot_path_ref_v_gain == -99.0)
        {
            logger.write("[UPDATE] robot_path_ref_v_gain doesn't exist", true);
            is_all_exist = false;
        }

        // slam
        update_config.robot_slam_submap_cnt = up_settings.value("SLAM/slam_submap_cnt").toInt();
        if(update_config.robot_slam_submap_cnt == -99)
        {
            logger.write("[UPDATE] robot_path_shifting_val doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_slam_lc_dist = up_settings.value("SLAM/slam_lc_dist", -99.0).toDouble();
        if(update_config.robot_slam_lc_dist == -99.0)
        {
            logger.write("[UPDATE] robot_slam_lc_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_slam_lc_icp_dist = up_settings.value("SLAM/slam_lc_icp_dist", -99.0).toDouble();
        if(update_config.robot_slam_lc_icp_dist == -99.0)
        {
            logger.write("[UPDATE] robot_slam_lc_icp_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_map_size = up_settings.value("SLAM/map_size", -99).toInt();
        if(update_config.robot_map_size == -99)
        {
            logger.write("[UPDATE] robot_map_size doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_grid_size = up_settings.value("SLAM/grid_size", -99.0).toDouble();
        if(update_config.robot_grid_size == -99.0)
        {
            logger.write("[UPDATE] robot_grid_size doesn't exist", true);
            is_all_exist = false;
        }

        // localization
        update_config.robot_icp_dist = up_settings.value("LOCALIZATION/icp_dist", -99.0).toDouble();
        if(update_config.robot_icp_dist == -99.0)
        {
            logger.write("[UPDATE] robot_icp_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_near = up_settings.value("LOCALIZATION/icp_near", -99.0).toDouble();
        if(update_config.robot_icp_near == -99.0)
        {
            logger.write("[UPDATE] robot_icp_near doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_error = up_settings.value("LOCALIZATION/icp_error", -99.0).toDouble();
        if(update_config.robot_icp_error == -99.0)
        {
            logger.write("[UPDATE] robot_icp_error doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_ratio = up_settings.value("LOCALIZATION/icp_ratio", -99.0).toDouble();
        if(update_config.robot_icp_ratio == -99.0)
        {
            logger.write("[UPDATE] robot_icp_ratio doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_repeat_dist = up_settings.value("LOCALIZATION/icp_repeat_dist", -99.0).toDouble();
        if(update_config.robot_icp_repeat_dist == -99.0)
        {
            logger.write("[UPDATE] robot_icp_repeat_dist doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_repeat_time = up_settings.value("LOCALIZATION/icp_repeat_time", -99.0).toDouble();
        if(update_config.robot_icp_repeat_time == -99.0)
        {
            logger.write("[UPDATE] robot_icp_repeat_time doesn't exist", true);
            is_all_exist = false;
        }

        update_config.robot_icp_odometry_weight = up_settings.value("LOCALIZATION/icp_odometry_weight", -99.0).toDouble();
        if(update_config.robot_icp_odometry_weight == -99.0)
        {
            logger.write("[UPDATE] robot_icp_odometry_weight doesn't exist", true);
            is_all_exist = false;
        }

        // motor
        update_config.motor_gear_ratio = up_settings.value("MOTOR/gear_ratio", -99.0).toDouble();
        if(update_config.motor_gear_ratio == -99.0)
        {
            logger.write("[UPDATE] motor_gear_ratio doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_wheel_dir = up_settings.value("MOTOR/wheel_dir", -99.0).toDouble();
        if(update_config.motor_wheel_dir == -99.0)
        {
            logger.write("[UPDATE] motor_wheel_dir doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_left_id = up_settings.value("MOTOR/left_id", -99).toInt();
        if(update_config.motor_left_id == -99)
        {
            logger.write("[UPDATE] motor_left_id doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_right_id = up_settings.value("MOTOR/right_id", -99).toInt();
        if(update_config.motor_right_id == -99)
        {
            logger.write("[UPDATE] motor_right_id doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_k_p = up_settings.value("MOTOR/k_p", -99.0).toDouble();
        if(update_config.motor_k_p == -99.0)
        {
            logger.write("[UPDATE] motor_k_p doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_k_i = up_settings.value("MOTOR/k_i", -99.0).toDouble();
        if(update_config.motor_k_i == -99.0)
        {
            logger.write("[UPDATE] motor_k_i doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_k_d = up_settings.value("MOTOR/k_d", -99.0).toDouble();
        if(update_config.motor_k_d == -99.0)
        {
            logger.write("[UPDATE] motor_k_d doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_limit_v = up_settings.value("MOTOR/limit_v", -99.0).toDouble();
        if(update_config.motor_limit_v == -99.0)
        {
            logger.write("[UPDATE] motor_limit_v doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_limit_w = up_settings.value("MOTOR/limit_w", -99.0).toDouble()*D2R;
        if(update_config.motor_limit_w == -99.0*D2R)
        {
            logger.write("[UPDATE] motor_limit_w doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_limit_v_acc = up_settings.value("MOTOR/limit_v_acc", -99.0).toDouble();
        if(update_config.motor_limit_v_acc == -99.0)
        {
            logger.write("[UPDATE] motor_limit_v_acc doesn't exist", true);
            is_all_exist = false;
        }

        update_config.motor_limit_w_acc = up_settings.value("MOTOR/limit_w_acc", -99.0).toDouble()*D2R;
        if(update_config.motor_limit_w_acc == -99.0*D2R)
        {
            logger.write("[UPDATE] motor_limit_w_acc doesn't exist", true);
            is_all_exist = false;
        }

        // STATIC CONFIG
        // robot
        static_config.robot_radius = st_settings.value("ROBOT_HW/robot_radius", -99.0).toDouble();
        if(static_config.robot_radius == -99.0)
        {
            logger.write("[STATIC] robot_radius doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_length = st_settings.value("ROBOT_HW/robot_length", -99.0).toDouble();
        if(static_config.robot_length == -99.0)
        {
            logger.write("[STATIC] robot_length doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_length_x = st_settings.value("ROBOT_HW/robot_length_x", -99.0).toDouble();
        if(static_config.robot_length_x == -99.0)
        {
            logger.write("[STATIC] robot_length_x doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_length_y = st_settings.value("ROBOT_HW/robot_length_y", -99.0).toDouble();
        if(static_config.robot_length_y == -99.0)
        {
            logger.write("[STATIC] robot_length_y doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_wheel_base = st_settings.value("ROBOT_HW/wheel_base", -99.0).toDouble();
        if(static_config.robot_wheel_base == -99.0)
        {
            logger.write("[STATIC] robot_wheel_base doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_wheel_radius = st_settings.value("ROBOT_HW/wheel_radius", -99.0).toDouble();
        if(static_config.robot_wheel_radius == -99.0)
        {
            logger.write("[STATIC] robot_wheel_radius doesn't exist", true);
            is_all_exist = false;
        }

        // sensor
        static_config.robot_lidar_tf = st_settings.value("SENSOR/lidar_offset_tf", "-99.0").toString();
        if(static_config.robot_lidar_tf == "-99.0")
        {
            logger.write("[STATIC] robot_lidar_tf doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_cam_left_sn =st_settings.value("SENSOR/left_camera_serial", "-99.0").toString();
        if(static_config.robot_cam_left_sn == "-99.0")
        {
            logger.write("[STATIC] robot_cam_left_sn doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_cam_right_sn = st_settings.value("SENSOR/right_camera_serial", "-99.0").toString();
        if(static_config.robot_cam_right_sn == "-99.0")
        {
            logger.write("[STATIC] robot_cam_right_sn doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_cam_left_tf = st_settings.value("SENSOR/left_camera_tf", "-99.0").toString();
        if(static_config.robot_cam_left_tf == "-99.0")
        {
            logger.write("[STATIC] robot_cam_left_tf doesn't exist", true);
            is_all_exist = false;
        }

        static_config.robot_cam_right_tf = st_settings.value("SENSOR/right_camera_tf", "-99.0").toString();
        if(static_config.robot_cam_right_tf == "-99.0")
        {
            logger.write("[STATIC] robot_cam_right_tf doesn't exist", true);
            is_all_exist = false;
        }

        // SETTING CONFIG
        // map
        setting_config.map_name = se_settings.value("MAP/map_name", "-99.0").toString();
        if(setting_config.map_name == "-99.0")
        {
            logger.write("[SETTING] robot_cam_right_tf doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.map_path = se_settings.value("MAP/map_path", "-99.0").toString();
        if(setting_config.map_path == "-99.0")
        {
            logger.write("[SETTING] map_path doesn't exist", true);
            is_all_exist = false;
        }

        // robot_type
        setting_config.robot_model = se_settings.value("ROBOT_TYPE/model").toString();
        if(setting_config.robot_model == "-99.0")
        {
            logger.write("[SETTING] robot_model doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_sn = se_settings.value("ROBOT_TYPE/serial_num").toString();
        if(setting_config.robot_sn == "-99.0")
        {
            logger.write("[SETTING] robot_sn doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_type = se_settings.value("ROBOT_TYPE/type").toString();
        if(setting_config.robot_type == "-99.0")
        {
            logger.write("[SETTING] robot_type doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.fms_ip = se_settings.value("SERVER/fms_ip").toString();
        if(setting_config.fms_ip == "-99.0")
        {
            logger.write("[SETTING] fms_ip doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.fms_id = se_settings.value("SERVER/fms_id").toString();
        if(setting_config.fms_id == "-99.0")
        {
            logger.write("[SETTING] fms_id doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.fms_pw = se_settings.value("SERVER/fms_pw").toString();
        if(setting_config.fms_pw == "-99.0")
        {
            logger.write("[SETTING] fms_pw doesn't exist", true);
            is_all_exist = false;
        }

        // obstacle
        setting_config.robot_obs_detect_area = se_settings.value("OBSTACLE/obs_detect_area", -99).toInt();
        if(setting_config.robot_obs_detect_area == -99)
        {
            logger.write("[SETTING] robot_obs_detect_area doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_detect_sensitivity = se_settings.value("OBSTACLE/obs_detect_sensitivity", -99).toInt();
        if(setting_config.robot_obs_detect_sensitivity == -99)
        {
            logger.write("[SETTING] robot_obs_detect_sensitivity doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_deadzone = se_settings.value("OBSTACLE/obs_deadzone", -99.0).toDouble();
        if(setting_config.robot_obs_deadzone == -99.0)
        {
            logger.write("[SETTING] robot_obs_deadzone doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_early_stop_dist = se_settings.value("OBSTACLE/obs_early_stop_dist", -99.0).toDouble();
        if(setting_config.robot_obs_early_stop_dist == -99.0)
        {
            logger.write("[SETTING] robot_obs_early_stop_dist doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_static_obs_margin = se_settings.value("OBSTACLE/obs_margin0", -99.0).toDouble();
        if(setting_config.robot_static_obs_margin == -99.0)
        {
            logger.write("[SETTING] robot_obs_margin0 doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_dynamic_obs_margin = se_settings.value("OBSTACLE/obs_margin1", -99.0).toDouble();
        if(setting_config.robot_dynamic_obs_margin == -99.0)
        {
            logger.write("[SETTING] robot_obs_margin1 doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_check_range = se_settings.value("OBSTACLE/obs_check_range", -99.0).toDouble();
        if(setting_config.robot_obs_check_range == -99.0)
        {
            logger.write("[SETTING] robot_obs_check_range doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_cam_height_min = se_settings.value("OBSTACLE/obs_height_min", -99.0).toDouble();
        if(setting_config.robot_obs_cam_height_min == -99.0)
        {
            logger.write("[SETTING] robot_obs_height_min doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_cam_height_max = se_settings.value("OBSTACLE/obs_height_max", -99.0).toDouble();
        if(setting_config.robot_obs_cam_height_max == -99.0)
        {
            logger.write("[SETTING] robot_obs_height_max doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_wait_time = se_settings.value("OBSTACLE/obs_wait_time", -99.0).toDouble();
        if(setting_config.robot_obs_wait_time == -99.0)
        {
            logger.write("[SETTING] robot_obs_wait_time doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_near_dist = se_settings.value("OBSTACLE/obs_near", -99.0).toDouble();
        if(setting_config.robot_obs_near_dist == -99.0)
        {
            logger.write("[SETTING] robot_obs_near doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_preview_time = se_settings.value("OBSTACLE/obs_preview_time", -99.0).toDouble();
        if(setting_config.robot_obs_preview_time == -99.0)
        {
            logger.write("[SETTING] robot_obs_preview_time doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_obs_decel_gain = se_settings.value("OBSTACLE/obs_decel_gain", -99.0).toDouble();
        if(setting_config.robot_obs_decel_gain == -99.0)
        {
            logger.write("[SETTING] robot_obs_decel_gain doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_use_obs_preview = se_settings.value("USE_SLAM/use_obs_preview").toBool();
        setting_config.robot_use_obs_near = se_settings.value("USE_SLAM/use_obs_near").toBool();
        setting_config.robot_use_early_stop_resting = se_settings.value("USE_SLAM/use_earlystop_resting").toBool();
        setting_config.robot_use_early_stop_serving = se_settings.value("USE_SLAM/use_earlystop_serving").toBool();
        setting_config.robot_use_multi = se_settings.value("USE_SLAM/use_multirobot").toBool();
        setting_config.robot_use_ignore_safetyzone = se_settings.value("USE_SLAM/use_ignore_safetyzone_return").toBool();
        setting_config.robot_use_ccma = se_settings.value("USE_SLAM/use_ccma").toBool();

        // sensor
        setting_config.robot_lidar_max_range = se_settings.value("SENSOR/max_range", -99.0).toDouble();
        if(setting_config.robot_lidar_max_range == -99.0)
        {
            logger.write("[SETTING] robot_lidar_max_range doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_lidar_mask = se_settings.value("SENSOR/mask", -99.0).toDouble();
        if(setting_config.robot_lidar_mask == -99.0)
        {
            logger.write("[SETTING] robot_lidar_mask doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_icp_ratio0 = se_settings.value("INITIALIZATION/icp_init_ratio", "-99.0").toDouble();
        if(setting_config.robot_icp_ratio0 == -99.0)
        {
            logger.write("[SETTING] robot_icp_ratio0 doesn't exist", true);
            is_all_exist = false;
        }

        setting_config.robot_icp_error0 = se_settings.value("INITIALIZATION/icp_init_error", "-99.0").toDouble();
        if(setting_config.robot_icp_error0 == -99.0)
        {
            logger.write("[SETTING] robot_icp_error0 doesn't exist", true);
            is_all_exist = false;
        }

        for(int i = PRESET_SPEED_SLOWEST; i <= PRESET_SPEED_FASTEST; ++i)
        {
            QString str_v;
            QString str_w;
            QString str_v_acc;
            QString str_w_acc;
            QString str_pivot;
            QString str_pivot_acc;

            str_v.sprintf("PRESET%d/limit_v", i);
            str_w.sprintf("PRESET%d/limit_w", i);
            str_v_acc.sprintf("PRESET%d/limit_v_acc", i);
            str_w_acc.sprintf("PRESET%d/limit_w_acc", i);
            str_pivot.sprintf("PRESET%d/limit_pivot", i);
            str_pivot_acc.sprintf("PRESET%d/limit_pivot_acc", i);

            setting_config.robot_preset_limit_v[i] = se_settings.value(str_v, -99.0).toDouble();
            if(setting_config.robot_preset_limit_v[i] == -99.0)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_v %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            setting_config.robot_preset_limit_w[i] = se_settings.value(str_w, -99.0).toDouble()*D2R;
            if(setting_config.robot_preset_limit_w[i] == -99.0*D2R)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_w %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            setting_config.robot_preset_limit_v_acc[i] = se_settings.value(str_v_acc, -99.0).toDouble();
            if(setting_config.robot_preset_limit_v_acc[i] == -99.0)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_v_acc %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            setting_config.robot_preset_limit_w_acc[i] = se_settings.value(str_w_acc, -99.0).toDouble()*D2R;
            if(setting_config.robot_preset_limit_w_acc[i] == -99.0*D2R)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_w_acc %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            setting_config.robot_preset_limit_pivot[i] = se_settings.value(str_pivot, -99.0).toDouble()*D2R;
            if(setting_config.robot_preset_limit_pivot[i] == -99.0*D2R)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_pivot %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            setting_config.robot_preset_limit_pivot_acc[i] = se_settings.value(str_pivot_acc, -99.0).toDouble()*D2R;
            if(setting_config.robot_preset_limit_pivot_acc[i] == -99.0*D2R)
            {
                QString str;
                str.sprintf("[SETTING] robot_preset_limit_pivot_acc %d doesn't exist", i);
                logger.write(str, true);
                is_all_exist = false;
            }

            //printf("%f %f %f %f %f %f\n", setting_config.robot_preset_limit_v[i], setting_config.robot_preset_limit_w[i]*R2D, setting_config.robot_preset_limit_v_acc[i],
            //       setting_config.robot_preset_limit_w_acc[i]*R2D, setting_config.robot_preset_limit_pivot[i]*R2D, setting_config.robot_preset_limit_pivot_acc[i]*R2D);
        }

        logger.write("robot config loaded", true);
    }
    else
    {
        logger.write("robot config not found", true);
    }

    if(is_all_exist == false)
    {
        return;
    }
}

std::vector<cv::Vec2i> filled_circle_iterator(cv::Vec2i pt, int r)
{
    std::vector<cv::Vec2i> res;
    for(int i = -r; i <= r; i++)
    {
        for(int j = -r; j <= r; j++)
        {
            double d = i*i + j*j;
            if(d <= r*r)
            {
                res.push_back(cv::Vec2i(i,j) + pt);
            }
        }
    }
    return res;
}

std::vector<cv::Vec2i> spiral_iterator(cv::Vec2i pt, int r)
{
    int x, y, dy = 0;
    int dx = 1;
    int _r = 1;

    std::vector<cv::Vec2i> res;
    res.push_back(pt);

    int all_pts = (2*r +1)*(2*r +1)-1;
    for(int i = 0; i < all_pts; ++i)
    {
        if(x == _r && y != std::abs(_r))
        {
            dx = 0;
            dy = -1;
        }
        if((x == _r) && (y == -_r))
        {
            dx = -1;
            dy = 0;
        }
        if(x == -_r && y != std::abs(_r))
        {
            dx = 0;
            dy = 1;
        }
        if((x == -_r) && (y == _r))
        {
            dx = 1;
            dy = 0;
            _r += 1;
        }

        x += dx;
        y += dy;
        res.push_back(cv::Vec2i(x, y) +pt);
    }

    return res;
}

std::vector<cv::Vec2i> line_iterator(cv::Vec2i pt0, cv::Vec2i pt1)
{
    std::vector<cv::Vec2i> res;

    int x1 = pt0[0];
    int y1 = pt0[1];
    int x2 = pt1[0];
    int y2 = pt1[1];

    int add_x = 0;
    int add_y = 0;
    int count = 0;

    int dx = x2-x1;
    int dy = y2-y1;

    if(dx < 0)
    {
        add_x = -1;
        dx = -dx;
    }
    else
    {
        add_x = 1;
    }

    if(dy < 0)
    {
        add_y = -1;
        dy = -dy;
    }
    else
    {
        add_y = 1;
    }

    int x = x1;
    int y = y1;

    if(dx >= dy)
    {
        for(int i = 0; i < dx; i++)
        {
            x += add_x;
            count += dy;

            if(count >= dx)
            {
                y += add_y;
                count -= dx;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }
    else
    {
        for(int i = 0; i < dy; i++)
        {
            y += add_y;
            count += dx;

            if(count >= dy)
            {
                x += add_x;
                count -= dy;
            }

            res.push_back(cv::Vec2i(x,y));
        }
    }

    return res;
}

std::vector<cv::Vec2i> circle_iterator(cv::Vec2i pt, int r)
{
    int x = r;
    int y = 0;
    int error = 3 - 2*r;

    std::vector<cv::Vec2i> res;
    while (x >= y)
    {
        res.push_back(cv::Vec2i(x, y) + pt);
        res.push_back(cv::Vec2i(x, -y) + pt);
        res.push_back(cv::Vec2i(-x, y) + pt);
        res.push_back(cv::Vec2i(-x, -y) + pt);

        res.push_back(cv::Vec2i(y, x) + pt);
        res.push_back(cv::Vec2i(y, -x) + pt);
        res.push_back(cv::Vec2i(-y, x) + pt);
        res.push_back(cv::Vec2i(-y, -x) + pt);

        if (error > 0)
        {
            error -= 4 * (--x);
        }
        error += 4 * (++y) + 2;
    }

    return res;
}

int main(int argc, char *argv[])
{
    logger.write("[MAIN] program start", true);

    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<cv::Vec3d>();
    qRegisterMetaType<std::vector<cv::Vec3d>>();
    qRegisterMetaType<MOBILE_POSE>();
    qRegisterMetaType<MOBILE_STATUS>();
    qRegisterMetaType<CMD>();

    // read config
    update_robot_config();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

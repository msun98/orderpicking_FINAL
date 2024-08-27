//#pragma once

#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H

// QT
#include <QString>
#include <QDebug>
#include <QTimer>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtNetwork>
#include <QScrollBar>
#include <QLineEdit>
#include <QFileDialog>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QObject>
#include <QDir>
#include <filesystem>
#include <QProcess>

#include <unistd.h>
#include <iostream>

// opencv
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#define MAX_MC              6
#define RT_TIMER_PERIOD_MS  2
#define COMMAND_CANID           0x01

#define MAX_SHARED_DATA 116
#define MAX_CONFIG_DATA 24

enum FSM_State
{
    STANBY = 0,
    OMRON_MOVE_GOAL_1,
    OMRON_MOVE_GOAL_2,
    OMRON_MOVE_START,

    RB5_MOVE_INIT,
    RB5_MOVE_VISION,
    RB5_MOVE_GRIPPER_READY,
    RB5_MOVE_GRIPPER_GRAPS,

    LIFT_MOVE_TOP,
    LIFT_MOVE_MID,
    LIFT_MOVE_BOT,

    REQ_VISION_DETECT,
    REQ_GRIPPER_GRAPS

};

enum RB5_connect_info
{
    NO_CONNECTION = -1,
    DATA_CONNECTED,
    CMD_CONNECTED,
    CMD_DATA_CONNECTED,
    ALL_CONNECT_AND_INIT_COMPLETE
};

enum socket_connect_info
{
    DISCONNECTED = 0,
    CONNECTED
};

enum RB5_move_info
{
    RB5_idle = 1,
    RB5_paused = 2,
    RB5_moving = 3
};


enum liftCMD
{
    NONE = 0,
    RELEASE_LIMIT_SW = 1,
    MOVE_POS = 2,
    MOVE_POS_INC = 3
};

enum colorInfo
{
    white = -1,
    red,
    green,
    yellow
};

struct jointInfo
{
    float joint_1 = -1;
    float joint_2 = -1;
    float joint_3 = -1;
    float joint_4 = -1;
    float joint_5 = -1;
    float joint_6 = -1;

    float acc = -1;
    float spd = -1;

};

struct tcpInfo
{
    float x = -1;
    float y = -1;
    float z = -1;
    float rx = -1;
    float ry = -1;
    float rz = -1;

    float acc = -1;
    float spd = -1;

};

struct tcp_joint_Info
{
    jointInfo joints;
    tcpInfo tcps;
};

struct xyzInfo
{
    float x = -1;
    float y = -1;
    float z = -1;
};

struct liftInfo
{
    int rpm      = -1;
    int ampere   = -1;
    int crtlType = -1;
    int srpm     = -1;
    int ctrlOut  = -1;
    int alaram   = -1;
    int location = -1;
    int brake    = -1;
    int temp     = -1;

    bool is_lift_move = false;

    float height_top = -1;
    float height_mid = -1;
    float height_bot = -1;
};

struct ipInfo
{
    QString ip = "";
    int port = -1;
};

class Integrated_infomation
{
public:

    QString Log_info;

    int  init_omron = -1;        // telnet connected.
    int  init_lift = -1;         // Can check, homing completed, ref on.
    int  init_rb5 = -1;          // CMD, DATA socket connected, initialization completed.
    bool init_vision = false;   // socket connected.
    bool init_gripper = false;  // socket connected.

    int socket_state_RB5_CMD  = 0; // 0 ~ 6
    int socket_state_RB5_DATA = 0;
    int socket_state_VISION   = 0;
    int socket_state_GRIPPER  = 0;

    // RB5
    bool RB5_check_connection_CMD = false;
    bool RB5_check_connection_DATA = false;
    bool RB5_check_connection_CMD_DATA = false;
    bool RB5_mode = false; // false : simul, true : real
    jointInfo joints;
    tcpInfo tcps;

    // LIFT
    liftInfo lifts;
//    omronInfo  omrons;

    int Lift_status_color = -1;
    int Lift_ref_color = -1;

    float lift_ref = -1;
    float lift_enc = -1;
    float lift_goal = -1;
    float lift_top = -1;
    float lift_mid = -1;
    float lift_bot = -1;

    // Slide bar
    int speed_omron = -1;
    int speed_rb5 = -1;

    cv::Mat Keti_Frame = cv::Mat::zeros(100, 100, CV_8UC1);
    QString keti_res   = "";
    xyzInfo keti_grapsCoord;
    bool is_keti_est_point_get = false;

    QString keti_req   = "";
    QString kitech_res = "";
    QString kitech_req = "";


    // Calibration parameter
    tcpInfo  CALIB_v2n;
    tcpInfo  CALIB_n2e;
    tcpInfo  GrapsPoint;

    Integrated_infomation& operator=(const Integrated_infomation& p)
    {
        if(this == &p)
            return *this;

//        init_omron = p.init_omron;
        init_lift = p.init_lift;
        init_rb5 = p.init_rb5;
        init_vision = p.init_vision;
        init_gripper = p.init_gripper;

        socket_state_RB5_CMD  = p.socket_state_RB5_CMD;
        socket_state_RB5_DATA = p.socket_state_RB5_DATA;
        socket_state_VISION   = p.socket_state_VISION;
        socket_state_GRIPPER  = p.socket_state_GRIPPER;

        // Slide
        speed_omron = p.speed_omron;
        speed_rb5 = p.speed_rb5;

        lift_ref = p.lift_ref;
        lift_enc = p.lift_enc;
        lift_goal = p.lift_goal;

        lift_top = p.lift_top;
        lift_mid = p.lift_mid;
        lift_bot = p.lift_bot;

//        omrons.omron_battery = p.omrons.omron_battery;
//        omrons.omron_temp = p.omrons.omron_temp;
//        omrons.omron_location = p.omrons.omron_location;

//        omrons.omron_map = p.omrons.omron_map.clone();
        Keti_Frame = p.Keti_Frame.clone();
    }
};


typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    FRC:1;		// Friction Compensation //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;   // Multi-Turn Error //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    CAN:1;
        unsigned    rsvd2:7;
    }b;
    unsigned char B[4];
}mSTAT;


enum RCR_INIT_ERR_FLAG{
    INIT_STAT_ERR_CLEAR = 0,
    INIT_STAT_ERR_NO_SMPS,
    INIT_STAT_ERR_NO_EMG_SW,
    INIT_STAT_ERR_GPIF_PSW,
    INIT_STAT_ERR_GPIF_FET,
    INIT_STAT_ERR_MC_CAN_CHECK,
    INIT_STAT_ERR_MC_FIND_HOME,
    INIT_STAT_ERR_COL_OFFSET,
};

enum RCR_INIT_INFO{
    INIT_STAT_INFO_NOACT = 0,
    INIT_STAT_INFO_VOLTAGE_CHECK,
    INIT_STAT_INFO_DEVICE_CHECK,
    INIT_STAT_INFO_FIND_HOME,
    INIT_STAT_INFO_VARIABLE_CHECK,
    INIT_STAT_INFO_COLLISION_ON,
    INIT_STAT_INFO_INIT_DONE
};



typedef struct{
    char    shared_data_name[64];
    int     shared_data_type;
}shared_data_info;
//extern shared_data_info sd_info_stat[MAX_SHARED_DATA];
//extern shared_data_info sd_info_config[MAX_CONFIG_DATA];

/*
typedef union{
    struct{
        char    header[4];
        // 0
        float   time;                   // time [sec]
        float   jnt_ref[6];             // joint reference [deg]
        float   jnt_ang[6];             // joint encoder value [deg]
        float   cur[6];                 // joint current value [mA]
        // 19
        float   tcp_ref[6];             // calculated tool center point from reference [mm, deg]
        float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg]
        // 31
        float   analog_in[4];           // analog input value of control box [V]
        float   analog_out[4];          // analog output value of control box [V]
        int     digital_in[16];         // digital input value of control box [0 or 1]
        int     digital_out[16];        // digital input value of control box [0 or 1]
        // 71
        float   temperature_mc[6];      // board temperature of each joint [celcius]
        // 77
        int     task_pc;                // (ignore)
        int     task_repeat;            // (ignore)
        int     task_run_id;            // (ignore)
        int     task_run_num;           // (ignore)
        float   task_run_time;          // (ignore)
        int     task_state;             // (ignore)
        // 83
        float   default_speed;          // overriding speed [0~1]
        int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving]
        int     power_state;            // power state
        // 86
        float   tcp_target[6];          // (ignore)
        int     jnt_info[6];            // joint information (look mSTAT)
        // 98
        int     collision_detect_onoff; // collision detect onoff [0:off  1:on]
        int     is_freedrive_mode;      // current freedrive status [0:off  1:on]
        int     program_mode;           // current program mode [0:real mode  1:simulation mode]
        // 101
        int     init_state_info;        // status information of robot initialization process
        int     init_error;             // error code of robot initialization process
        // 103
        float   tfb_analog_in[2];       // analog input value of tool flange board [V]
        int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1]
        int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1]
        float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24]
        // 110
        int     op_stat_collision_occur;
        int     op_stat_sos_flag;
        int     op_stat_self_collision;
        int     op_stat_soft_estop_occur;
        int     op_stat_ems_flag;
        // 115
    }sdata;
    float fdata[MAX_SHARED_DATA];
    int idata[MAX_SHARED_DATA];
}systemSTAT;
*/

typedef union{
    struct{
        char    header[4];
        // 0
        float   time;                   // time [sec]
        float   jnt_ref[6];             // joint reference [deg]
        float   jnt_ang[6];             // joint encoder value [deg]
        float   cur[6];                 // joint current value [mA]
        // 19
        float   tcp_ref[6];             // calculated tool center point from reference [mm, deg]
        float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg]
        // 31
        float   analog_in[4];           // analog input value of control box [V]
        float   analog_out[4];          // analog output value of control box [V]
        int     digital_in[16];         // digital input value of control box [0 or 1]
        int     digital_out[16];        // digital input value of control box [0 or 1]
        // 71
        float   temperature_mc[6];      // board temperature of each joint [celcius]
        // 77
        int     task_pc;                // (ignore)
        int     task_repeat;            // (ignore)
        int     task_run_id;            // (ignore)
        int     task_run_num;           // (ignore)
        float   task_run_time;          // (ignore)
        int     task_state;             // (ignore)
        // 83
        float   default_speed;          // overriding speed [0~1]
        int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving]
        int     power_state;            // power state
        //        int information_chunk_1;//information bit combination

        // 86
        float   tcp_target[6];          // (ignore)
        int     jnt_info[6];            // joint information (look mSTAT)
        // 98
        int     collision_detect_onoff; // collision detect onoff [0:off  1:on]
        int     is_freedrive_mode;      // current freedrive status [0:off  1:on]
        int     program_mode;           // current program mode [0:real mode  1:simulation mode]
        // 101
        int     init_state_info;        // status information of robot initialization process
        int     init_error;             // error code of robot initialization process
        // 103
        float   tfb_analog_in[2];       // analog input value of tool flange board [V]
        int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1]
        int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1]
        float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24]
        // 110
        int     op_stat_collision_occur;
        int     op_stat_sos_flag;
        int     op_stat_self_collision;
        int     op_stat_soft_estop_occur;
        int     op_stat_ems_flag;
        ///////////////////////////////////////////////////////////
        // 115
        // Byte count ~: 464
        int information_chunk_2;
        int information_chunk_3;

        // Byte count ~: 472
        int inbox_trap_flag[2];
        int inbox_check_mode[2];

        // Byte count ~: 488
        float eft_fx;
        float eft_fy;
        float eft_fz;
        float eft_mx;
        float eft_my;
        float eft_mz;

        // Byte count ~: 512
        int information_chunk_4;

        // Byte count ~: 516
        float extend_io1_analog_in[4];
        float extend_io1_analog_out[4];
        unsigned int extend_io1_digital_info;

        // Byte count ~: 552
        float aa_joint_ref[6];

        // Byte count ~: 576
        unsigned int safety_board_stat_info;

        // Byte count ~: 580
    }sdata;
    float fdata[MAX_SHARED_DATA];
    int idata[MAX_SHARED_DATA];
}systemSTAT;

typedef union{
    struct{
        char    header[4];
        // 0
        float   sensitivity;            // collision threshold [0~1]
        float   work_x_min;             // (ignore)
        float   work_x_max;             // (ignore)
        float   work_y_min;             // (ignore)
        float   work_y_max;             // (ignore)
        float   work_z_min;             // (ignore)
        float   work_z_max;             // (ignore)
        float   mount_rotate[3];        // direction of gravity [normalized vector]
        // 10
        float   toolbox_size[3];        // virtual collision box size of tool [mm]
        float   toolbox_center_pos[3];  // virtual collision box position of tool [mm]
        float   tool_mass;              // tool mass [kg]
        float   tool_mass_center_pos[3];// center of mass position of tool [mm]
        float   tool_ee_pos[3];         // tool position [mm]
        // 23
    }sdata;
    float fdata[MAX_CONFIG_DATA];
}systemCONFIG;

typedef struct{
    char    header[4];
    char    type;
    char    msg[1000];
    int     len;
}systemPOPUP;

extern int      __IS_PODO_WORKING__;

extern int      _NO_OF_MC;
extern systemSTAT   *sys_status;
extern systemCONFIG *sys_config;
extern systemPOPUP  *sys_popup;
extern systemPOPUP  *sys_custom_alarm;

extern class Integrated_infomation Integrated_info;

// Utility ----------------------------
#define LOG_FAIL    "\033[1;31m"
#define LOG_GOOD    "\033[1;32m"
#define LOG_WARN    "\033[1;33m"
#define LOG_NORMAL  "\033[0m"

#endif // COMMON_HEADER_H

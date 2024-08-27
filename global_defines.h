//#pragma once

#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

#include <QStringList>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/ximgproc.hpp>
#include "opencv2/flann/miniflann.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se2.hpp>
#include <sophus/interpolate.hpp>

#include <time.h> //몇 초 후에 값을 주기 위함.

#define PI 3.14
#define D2R (M_PI/180.0)
//////////////////////////////////////////


namespace Eigen
{
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
}

Eigen::Matrix4d string_to_TF(QString str);
Eigen::Vector6d TF_to_ZYX(Eigen::Matrix4d TF);

enum UI_AUTO_STATE
{
    UI_AUTO_NOT_READY = 0,
    UI_AUTO_READY,
    UI_AUTO_MOVING,
    UI_AUTO_WAIT,
    UI_AUTO_PAUSE
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



enum DIR
{
    CW = 0,
    CCW = 1
};

enum PID
{
    PID_MAIN_DATA = 193

};

struct robot_status{
    int robot_state = 0;
    int gripper_state = 0;
    int lift_state = 0;
};

//for vision capture
enum{
    vison_cap_gripper_cmd=0,
    vison_cap_gripper_wait,
    vison_cap_vision_cmd,
    vison_cap_done
};

//for auto homming
enum{
    auto_homing_start=0,
    auto_homing_lift_wait,
    auto_homing_lift_working,
    auto_homing_working,
    auto_homing_wait,
    auto_homing_done
};

//for lift
enum{
    lift_start=0,
    lift_wait,
    lift_working,
    lift_done
};
enum{
    ROBOT_STATE_NOT_READY=0,
    ROBOT_STATE_START,
    ROBOT_STATE_CHECK_SCENE,

    ROBOT_STATE_AMR_START,
    ROBOT_STATE_AMR_MOVE_CHECK,
    ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE,
    ROBOT_STATE_AMR_MOVE_DONE,

    ROBOT_STATE_LIFT_START,
    ROBOT_STATE_LIFT_MOVE_CHECK,
    ROBOT_STATE_LIFT_WAIT,
    ROBOT_STATE_LIFT_DONE,

    ROBOT_STATE_ROBOT_START,
    ROBOT_STATE_ROBOT_MOVE_CHECK,
    ROBOT_STATE_ROBOT_WAIT,
    ROBOT_STATE_ROBOT_DONE,

    ROBOT_STATE_GRIPPER_START,
    ROBOT_STATE_GRIPPER_MOVE_CHECK,
    ROBOT_STATE_GRIPPER_WAIT,
    ROBOT_STATE_GRIPPER_DONE,

    ROBOT_STATE_VISION_START,
    ROBOT_STATE_VISION_MOVE_CHECK,
    ROBOT_STATE_VISION_WAIT,
    ROBOT_STATE_VISION_DONE,

    ROBOT_STATE_WAIT,
    ROBOT_STATE_WAIT_OUT,

    ROBOT_STATE_SUCCESS,
    ROBOT_STATE_DONE
    //    ROBOT_STATE_END
};


enum{
    FMS_ROBOT_STATE_NOT_READY=0,
    FMS_ROBOT_STATE_START,
    FMS_ROBOT_STATE_CHECK_SCENE,

    FMS_ROBOT_STATE_AMR_START,
    FMS_ROBOT_STATE_AMR_MOVE_CHECK,
    FMS_ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE,
    FMS_ROBOT_STATE_AMR_MOVE_DONE,

    FMS_ROBOT_STATE_LIFT_START,
    FMS_ROBOT_STATE_LIFT_MOVE_CHECK,
    FMS_ROBOT_STATE_LIFT_WAIT,
    FMS_ROBOT_STATE_LIFT_DONE,

    FMS_ROBOT_STATE_ROBOT_START,
    FMS_ROBOT_STATE_ROBOT_MOVE_CHECK,
    FMS_ROBOT_STATE_ROBOT_WAIT,
    FMS_ROBOT_STATE_ROBOT_DONE,

    FMS_ROBOT_STATE_GRIPPER_START,
    FMS_ROBOT_STATE_GRIPPER_MOVE_CHECK,
    FMS_ROBOT_STATE_GRIPPER_WAIT,
    FMS_ROBOT_STATE_GRIPPER_DONE,

    FMS_ROBOT_STATE_VISION_START,
    FMS_ROBOT_STATE_VISION_MOVE_CHECK,
    FMS_ROBOT_STATE_VISION_WAIT,
    FMS_ROBOT_STATE_VISION_DONE,

    FMS_ROBOT_STATE_WAIT,
    FMS_ROBOT_STATE_WAIT_OUT,

    FMS_ROBOT_STATE_SUCCESS
    //    ROBOT_STATE_END
};


// global_defines.h
struct MOTOR_MAIN_DATA
{
    short motor_rpm = 0;
    short motor_armp = 0;
    char  motor_controlType = char(0);
    short motor_ref_rpm = 0;
    short motor_controlOutput = 0;
    char  motor_status = char(0);
    int   motor_position = 0;
    char  motor_brake_duty = char(0);
    char  motor_temperature = 0;

    MOTOR_MAIN_DATA()
    {}

    MOTOR_MAIN_DATA(const MOTOR_MAIN_DATA &p)
    {
        motor_rpm = p.motor_rpm;
        motor_armp = p.motor_armp;
        motor_controlType = p.motor_controlType;
        motor_ref_rpm = p.motor_ref_rpm;
        motor_controlOutput = p.motor_controlOutput;
        motor_status = p.motor_status;
        motor_position = p.motor_position;
        motor_brake_duty = p.motor_brake_duty;
        motor_temperature = p.motor_temperature;
    }

};

struct YUJIN_PATH{
    std::vector<cv::Vec2d> path;

    YUJIN_PATH()
    {

    }
    YUJIN_PATH(const YUJIN_PATH& p)
    {
        path=p.path;
    }
};


// robot docking position, lift, obj name
struct SHELF_INFO
{
    QString shelf_id;
    cv::Vec3d AMR_pose;

    QString obj_direction;
    //    cv::Vec3d obj_pose;
    cv::Vec6d RB_5_pose;

    float lift_pose;

    //    QString linked;

    SHELF_INFO()
    {
        shelf_id = "";
        AMR_pose = cv::Vec3d(0,0,0);

        //        obj_pose = cv::Vec3d(0,0,0);
        //        obj_direction = "";

        RB_5_pose = cv::Vec6d(0,0,0,0,0,0);

        lift_pose = 0;

    }

    SHELF_INFO(const SHELF_INFO& p)
    {
        shelf_id = p.shelf_id;
        AMR_pose = p.AMR_pose;

        //        obj_pose = p.obj_pose;
        //        obj_direction = p.obj_direction;

        RB_5_pose = p.RB_5_pose;

        lift_pose = p.lift_pose;

        //        linked = p.linked;
    }

    //    ANNOTATION& operator=(const ANNOTATION& p)
    //    {
    //        group_id = p.group_id;
    //        group_name = p.group_name;

    //        loc_id = p.loc_id;
    //        loc_name = p.loc_name;

    //        attrib = p.attrib;
    //        tmp = p.tmp;
    //        pose = p.pose;
    //        return *this;
    //    }
};


// robot docking position, lift, obj name
struct OBJECT_INFO
{
    QString obj_id;
    QString obj_grap_pose;
    QString obj_ready_grap_pose;


    //    QString linked;

    OBJECT_INFO()
    {
        obj_id = "";
        obj_grap_pose = "";
        obj_ready_grap_pose = "";
    }

    OBJECT_INFO(const OBJECT_INFO& p)
    {
        obj_id = p.obj_id;
        obj_grap_pose = p.obj_grap_pose;
        obj_ready_grap_pose = p.obj_ready_grap_pose;
    }

    //    ANNOTATION& operator=(const ANNOTATION& p)
    //    {
    //        group_id = p.group_id;
    //        group_name = p.group_name;

    //        loc_id = p.loc_id;
    //        loc_name = p.loc_name;

    //        attrib = p.attrib;
    //        tmp = p.tmp;
    //        pose = p.pose;
    //        return *this;
    //    }
};

struct ROBOT_CONFIG
{
    // read file
    QString shelves_name = "";
    QString object = "";
    QStringList object_list= {};
    QStringList shelves_list= {};

};
extern ROBOT_CONFIG robot_config;

struct motor_cmd
{
    // init motor
    QByteArray REQ_MAIN_DATA;       // PID 193

    QByteArray RELEASE_LIMIT_SW;    // PID 17

    // set motor (homing)
    QByteArray INIT_SET;            // PID 183
    QByteArray INIT_SET_CCW;        // PID 35
    QByteArray INIT_SET_CW;         // PID 35

    // set motor
    QByteArray POSI_MAX_VEL; // PID176

    // move motor
    QByteArray MOVE_TOP; // PID243
    QByteArray MOVE_MID; // PID243
    QByteArray MOVE_BOT; // PID243

    motor_cmd()
    {
        // PID_REQ_PID_DATA
        REQ_MAIN_DATA.resize(7);
        REQ_MAIN_DATA[0] = 0xb7;
        REQ_MAIN_DATA[1] = 0xac;
        REQ_MAIN_DATA[2] = 0x01;
        REQ_MAIN_DATA[3] = 0x04; // PID 4
        REQ_MAIN_DATA[4] = 0x01;
        REQ_MAIN_DATA[5] = 0xc1;
        REQ_MAIN_DATA[6] = 0xd6;

        // PID_FUNC_CMD_TYPE (for homing)
        INIT_SET.resize(8);
        INIT_SET[0] = 0xb7;
        INIT_SET[1] = 0xac;
        INIT_SET[2] = 0x01;
        INIT_SET[3] = 0xb7; // PID 183
        INIT_SET[4] = 0x02;
        INIT_SET[5] = 0x03;
        INIT_SET[6] = 0x00;
        INIT_SET[7] = 0xe0;

        // PID_INIT_SET
        INIT_SET_CW.resize(7);
        INIT_SET_CW[0] = 0xb7;
        INIT_SET_CW[1] = 0xac;
        INIT_SET_CW[2] = 0x01;
        INIT_SET_CW[3] = 0x23; // PID 35
        INIT_SET_CW[4] = 0x01;
        INIT_SET_CW[5] = 0x02; // 0x01 : CCW(+), 0x02 : CW(-)
        INIT_SET_CW[6] = 0x76;

        // PID_INIT_SET
        INIT_SET_CCW.resize(7);
        INIT_SET_CCW[0] = 0xb7;
        INIT_SET_CCW[1] = 0xac;
        INIT_SET_CCW[2] = 0x01;
        INIT_SET_CCW[3] = 0x23; // PID 35
        INIT_SET_CCW[4] = 0x01;
        INIT_SET_CCW[5] = 0x01; // 0x01 : CCW(+), 0x02 : CW(-)
        INIT_SET_CCW[6] = 0x77;

        // PID_USE_LIMIT_SW
        RELEASE_LIMIT_SW.resize(7);
        RELEASE_LIMIT_SW[0] = 0xb7;
        RELEASE_LIMIT_SW[1] = 0xac;
        RELEASE_LIMIT_SW[2] = 0x01;
        RELEASE_LIMIT_SW[3] = 0x11; // PID 17
        RELEASE_LIMIT_SW[4] = 0x01;
        RELEASE_LIMIT_SW[5] = 0x00; // limit clear
        RELEASE_LIMIT_SW[6] = 0x8a;

        // POSI_CMD (not encoder, hall sensor)
        MOVE_TOP.resize(10);
        MOVE_TOP[0] = 0xb7;
        MOVE_TOP[1] = 0xac;
        MOVE_TOP[2] = 0x01;
        MOVE_TOP[3] = 0xf3; // PID 243
        MOVE_TOP[4] = 0x04;
        MOVE_TOP[5] = 0xa0; // DATA0 (little endian)
        MOVE_TOP[6] = 0x0f; //
        MOVE_TOP[7] = 0x00; //
        MOVE_TOP[8] = 0x00; // DATA3 -> 4000
        MOVE_TOP[9] = 0xf6;

        MOVE_MID.resize(10);
        MOVE_MID[0] = 0xb7;
        MOVE_MID[1] = 0xac;
        MOVE_MID[2] = 0x01;
        MOVE_MID[3] = 0xf3; // PID 243
        MOVE_MID[4] = 0x04;
        MOVE_MID[5] = 0xd0; // DATA0 (little endian)
        MOVE_MID[6] = 0x07; //
        MOVE_MID[7] = 0x00; //
        MOVE_MID[8] = 0x00; // DATA3 -> 2000
        MOVE_MID[9] = 0xce;

        MOVE_BOT.resize(10);
        MOVE_BOT[0] = 0xb7;
        MOVE_BOT[1] = 0xac;
        MOVE_BOT[2] = 0x01;
        MOVE_BOT[3] = 0xf3; // PID 243
        MOVE_BOT[4] = 0x04;
        MOVE_BOT[5] = 0x00; // DATA0 (little endian)
        MOVE_BOT[6] = 0x00; //
        MOVE_BOT[7] = 0x00; //
        MOVE_BOT[8] = 0x00; // DATA3 -> 0
        MOVE_BOT[9] = 0xa5;
    }


};
extern motor_cmd MD_CMD;




struct CAM_CONFIG
{
public:
    QString sn;

    int w = 0;
    int h = 0;
    int exp = 0;

    Eigen::Matrix4d RB_TF_SENSOR;
    Eigen::Matrix4d KITECH_TF_SENSOR;

    CAM_CONFIG()
    {
        sn = "";
        RB_TF_SENSOR.setIdentity();
        KITECH_TF_SENSOR.setIdentity();
        exp = 0;
    }
    CAM_CONFIG(const CAM_CONFIG& p)
    {
        w = p.w;
        h = p.h;
        sn = p.sn;
        RB_TF_SENSOR = p.RB_TF_SENSOR;
        KITECH_TF_SENSOR = p.KITECH_TF_SENSOR;
        exp = p.exp;
    }

    CAM_CONFIG& operator=(const CAM_CONFIG& p)
    {
        w = p.w;
        h = p.h;
        sn = p.sn;
        RB_TF_SENSOR = p.RB_TF_SENSOR;
        KITECH_TF_SENSOR = p.KITECH_TF_SENSOR;
        exp = p.exp;
        return *this;
    }
};
extern CAM_CONFIG cam_config;


struct AMR_CONFIG
{
    // server
    QString AMR_ip = "";
    QString AMR_id = "";
    QString AMR_pw = "";
};
extern AMR_CONFIG amr_config;


#endif // GLOBAL_DEFINES_H



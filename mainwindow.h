#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// QT
#include <QMainWindow>
//#include <QDebug>
//#include <QTimer>
//#include <QUdpSocket>
//#include <QTcpServer>
//#include <QTcpSocket>
//#include <QtNetwork>
//#include <QScrollBar>
//#include <QLineEdit>
//#include <QFileDialog>
//#include <iostream>

//// my
#include "common_header.h"
#include "global_defines.h"
#include "option_writer.h"
#include "md_motor.h"
#include "keti_vision.h"
#include "kitech_gripper.h"
#include "barcode.h"
#include "cobot.h"
#include "ipc.h"
#include "websocket.h"
#include "mobile_robot.h"
#include "cv_to_qt.h"

////for using linux socket
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>

//#include <time.h> //몇 초 후에 값을 주기 위함.

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    //    systemSTAT  *stat;
    QLineEdit   *le_digital_in_on[16];
    QLineEdit   *le_digital_in_off[16];
    QLineEdit   *le_digital_out_on[16];
    QLineEdit   *le_digital_out_off[16];
    //    void SetStatus(systemSTAT *input_stat);
    /////////////////////////////////////////////////////////////////////////////

    clock_t start, end;
    double result;

    clock_t start_old, end_old;
    double result_old;

    //    systemSTAT systemStat;

    Option_Writer settings;
    robot_status status;
    float Joint_info_now[6];

    //        QTcpSocket cmdSocket;
    //        QTcpSocket dataSocket;
    systemCONFIG systemConfig;
    systemPOPUP  systemPopup;

    //    int cmdConfirmFlag;
    //    int moveCmdFlag;
    //    int moveCmdCnt;

    //    posInfo initPos;
    //    posInfo visionPos;
    //    posInfo readyGripperPos;
    //    posInfo grapsGripperPos;

    //    posInfo putdown1Pos;
    //    posInfo putdown2Pos;
    //    posInfo readyGripper2Pos;
    //    posInfo grapsGripper2Pos;

    QByteArray recvBuf;
    QTimer timer;

    QTimer logicTimer;
    QTimer systemTimer;

    QTimer liftTimer;
    QTimer CobotStatusTimer;

    //    void print();

    int initFlag = 1;
    bool speed_changed = false;

    void BTN_GRIPPER_HOLD();
    void map_size_load();

    int lift_hight = 0;
    int lift_rpm = 0;
    int lift_pos = 0;
    //    QString lift_state;
    int old_lift_pos = 0;
    int old_lift_rpm = 0;
    int armp = 0;

    int first_time=1;

    bool keti_connect_flag = false;

    // -------------------------------------

    ///////////////////////////////////////////////////////////////////////////
    MD_MOTOR md_mot;

    QTimer map_timer;
    QTimer jog_timer;
    QByteArray buf;

    //    QTcpServer tcp_server;
    QTcpSocket* tcp_socket;


    Keti_vision vision;
    Kitech_gripper gripper;
    Barcode barcode;
    Cobot cobot;

    // ipc
    IPC ipc;

    // websocket
    websocket web;

    //mobile robot;
    mobile_robot mb;
    QTimer mobile_timer;

    IPC::GRIPPER gripper_cmd;

    QTimer timer_shared_memory;
    QTimer *time_seq;

    QStringList scene;
    QStringList yujin_scene;

    QStringList yujin_list;

    uint32_t last_map_tick = 0;
    int cur_step = ROBOT_STATE_NOT_READY;
    int cur_step_yj = ROBOT_STATE_NOT_READY;

    float keti_x,keti_y,keti_z;
    float keti_rx,keti_ry,keti_rz;
    QString cent_res_x,cent_res_y,cent_res_z;

    Eigen::Matrix4d T_sensor;

    bool capture_flag = false;
    bool flag_circle = false;
    QString lift_state;

    bool detecting_flag = false;
    bool mobile_moving_flag = false;

    void init();

    int robot_pop_retry = 0;
    int lift_retry = 0;

    int lift_yujin_retry = 0;
    int yujin_robot_pop_retry = 0;
    int yujin_lift_retry = 0;

    float old_ipc_vision_x;

    QJsonArray pose_to_array(cv::Vec3d pose)
    {
        QJsonArray res;
        res.append(pose[0]);
        res.append(pose[1]);
        res.append(pose[2]);
        return res;
    }

    QJsonArray pose_to_array(std::vector<float> pose)
    {
        QJsonArray res;
        res.append(pose[0]);
        res.append(pose[1]);
        res.append(pose[2]);
        res.append(pose[3]);
        res.append(pose[4]);
        res.append(pose[5]);
        return res;
    }

    cv::Vec3d array_to_pose(QJsonArray array)
    {
        cv::Vec3d res;
        res[0] = array[0].toDouble();
        res[1] = array[1].toDouble();
        res[2] = array[2].toDouble();
        return res;
    }

    cv::Vec6d array_to_pose6(QJsonArray array1)
    {
        cv::Vec6d res;
        res[0] = array1[0].toDouble();
        res[1] = array1[1].toDouble();
        res[2] = array1[2].toDouble();
        res[3] = array1[3].toDouble();
        res[4] = array1[4].toDouble();
        res[5] = array1[5].toDouble();
        return res;
    }

    // storage
    //     std::vector<ANNOTATION> annotations;

    std::map<std::string, SHELF_INFO*> shelf_infos;
    std::map<std::string, OBJECT_INFO*> object_infos;
    void bt_order_check();

    // for yj msg
    std::mutex mtx;
    std::queue<QString> order_json_msg;
    std::queue<QString> order_yj_json_msg;

    // for stack qstringlist order msg
    std::queue<QStringList> order_list_msg;
    std::queue<QStringList> order_yj_list_msg;

    void load();

    // for writing information about shelf and object
    QJsonArray arr;
    QJsonArray arr1;

    int old_msg_size = 0;
    int que_time = 0;

    int yujin_old_msg_size = 0;
    int yujin_que_time = 0;

    int vision_num = 1;

    int old_robot_state=0;
    bool keti_box_request=false;
    bool first_boot_lift = false;

    QString mat_zyxzy();
    float app_x,app_y,app_z;

    int shutter_clicked;
    int auto_homing_clicked;

    bool grip = false;
    void save_log(QString log);
//    bool lift_working = false;

    bool old_connect_flag=false;
    QString old_rb5_msg;

    bool caution_flag = false;
    //    void yujin_order();
    //    void yujin_order_check();

    int lift_clicked = 3;

    int old_tick=0;
    int old_keti_x, old_keti_y, old_keti_z;

private:
    Ui::MainWindow *ui;
    QTimer  updateTimer;

    QTimer auto_homming_timer;
    QTimer vision_capture_timer;

    QTimer vision_trigger_timer;


    //    QTimer  robotStatusTimer;
//    QUdpSocket udp_socket;
    //    QTcpSocket *clientSocket;//for robot arrived check

public slots:
    void onSystemCheck();
    void lift_ui();
    void status_loop();

    void print();

    //    void onReadyCmdRead();
    //    void onReadyDataRead();
    //    void requestDataPacket();

    void UI_Connection_Update(QString msg);
    void UI_Connection_Update_Lift(QString msg);
    void onUpdate();

    /////////////////////////

    void keti_showUI_msg(bool connect_flag);
    void kitech_showUI_msg(bool connect_flag);
    void barcode1_showUI_msg(QByteArray msg);
    void barcode2_showUI_msg(QByteArray msg);
    void barcode_ON_showUI(QString msg);

    // kitech
    void BTN_GRIPPER_CMD_WRITE();
    void BTN_GRIPPER_ALLFOLD();
    void BTN_GRIPPER_GRASP();
    void BTN_GRIPPER_VISION();
    void CB_GRIPPER_GE_CMD_WRITE();
    void Pinch12_FOLD();
    //    void kitech_showUI_msg();

    //    void bt_connect();
    void bt_usb1_con();
    void bt_usb2_con();
    void bt_usb1_dis();
    void bt_usb2_dis();

    void UI_Connection_Update_mobile(QString);
    void rb_pose(QString);
    void bt_cobot_move2object();

    void set_ui_item_from_unimap();


    // for location and pose save
    void bt_save_annot();
    void bt_save_all_annot();
    void bt_delete_annot();

    void bt_save_annot_2();
    void bt_save_all_annot_2();
    void bt_delete_annot_2();

    void bt_order();

    void kitech_msg(QString msg);

    void LE_KETI_IMG(cv::Mat map_img);
    void auto_homming_seq();


    void yujin_order(QJsonObject json);
    void yujin_order_check();

    void vision_trigger();

    //    void sendData();

private slots:

    void tf_any_change();

    void bt_tcp_on();
    void bt_connect();
    void bt_lift_status();
    void bt_rpmZero();

    // position control
    void bt_homing();
    void bt_auto_homing();
    void bt_move_pos();
    void bt_lift_pos_move();
    void sb_lift_pos();
    void bt_set_maxVel_pos();
    void bt_lift_maxVel_pos();

    // RPM
    void bt_move_rpm();

    // received data
    //    void rev_data(QByteArray data);
    void write_log(QString str);

    //    void connected_seiral();
    //    void disconnected_seiral();

//    void readyRead_udp();
    //    void connected_udp();
    //    void disconnected_udp();

    //for ipc loop
    void timer_shared_memory_loop();

    void newConnection_tcp();
    //    void disConnection_tcp();
    //    void readyRead_tcp();

    void BTN_CONNECT_COM();
    void BTN_CONNECT_DATA();

    void homeEnd(QString msg);


    //    void on_BTN_CONNECT_COM_clicked();
    //    void on_BTN_CONNECT_DATA_clicked();
    void on_BTN_COBOT_INIT_clicked();
    void on_BTN_MODE_REAL_clicked();
    void on_BTN_MODE_SIMULATION_clicked();
    void on_BTN_CLEAR_clicked();
    void on_BTN_MOTION_PAUSE_clicked();
    void on_BTN_MOTION_RESUME_clicked();
    void on_BTN_MOTION_HALT_clicked();
    void on_BTN_COLLISION_RESUME_clicked();
    void on_BTN_MOVE_JOINT_HIGH_clicked();
    void on_BTN_MOVE_JOINT_READY_GRIPPER_clicked();
    void on_BTN_MOVE_JOINT_GRAPS_GRIPPER_clicked();
    void on_HS_BASE_SPEED_valueChanged(int value);

    void BTN_CONNECT_GRIPPER();
    void on_BTN_MOVE_JOINT_HIGH_LEFT_clicked();
    void on_BTN_MOVE_JOINT_INIT_3_clicked();
    void on_BTN_MOVE_JOINT_INIT_4_clicked();
    void on_BTN_MOVE_JOINT_INIT_5_clicked();

    //for img change
    void bt_png_change();

    //MAKE ZIP
    void bt_zip();

    //show ui websocket on
    void showUI_msg(bool msg);
    void showUI_recive_msg(QString message);
    void showUI_send_msg(QString message);

    void on_bt_rb5_mode_clicked();
    void on_sld_rb5_speed_valueChanged(int value);

    //pump motor on
    void btn_pump();
    void on_sb_rb5_speed_valueChanged(int arg1);

    void bt_get_map_from_mobile();

    void lb_sn_map(cv::Mat map_img);

    void on_BTN_MOVE_JOINT_Rotation_clicked();
    void on_bt_cobot_push_clicked();
    void on_btn_pump_2_clicked();
    void on_bt_TCP_Blend_clicked();
    void on_BTN_MOVE_JOINT_LOW_clicked();
    void on_BTN_MOVE_JOINT_INIT_7_clicked();
    void on_bt_lift_top_clicked();
    void on_bt_lift_MID_clicked();
    void on_bt_lift_Low_clicked();
    void on_bt_low_get_out_clicked();
    void on_bt_vision_water_clicked();
    void on_bt_vision_high_get_out_clicked();
    void on_BTN_MOVE_JOINT_MID_clicked();
    void on_bt_vision_rael_more_move_clicked();

    void on_BTN_MOVE_TOTAL_clicked();
    void on_pb_shutter_clicked();
    void on_bt_vision_cmd_capture_clicked();
    void on_BTN_MOVE_JOINT_MID_LEFT_clicked();
    void on_BTN_MOVE_JOINT_MID_RIGHT_clicked();

    void seqLoop();
    void on_ALL_STOP_clicked();
    void on_BTN_MOVE_TOTAL_2_clicked();
    void on_BTN_MOVE_TOTAL_3_clicked();
    void on_BTN_MOVE_TOTAL_4_clicked();
    void on_pb_file_choose_clicked();
    void on_bt_rb5_pause_clicked();
    void on_bt_rb5_resume_clicked();
    void on_bt_rb5_halt_clicked();

    void yujin_loop();
    void on_bt_sn_cmd_set_clicked();

    void bt_amr_motor_init();
    void on_BTN_show_clicked();

    void vision_img_capture();

    void BTN_GRIPPER_recog_WRITE();
    void on_BTN_MOVE_JOINT_POP_clicked();
    void on_BTN_MOVE_JOINT_BOX_CENTER_clicked();
    void on_BTN_TCP_MOTION_TEST_clicked();
    void on_BTN_JOINT_MOTION_TEST_clicked();
    void on_pb_reload_clicked();
    void on_bt_scen_resume_clicked();

    void lb_keti_point(QString);
    void on_BTN_MOVE_JOINT_BOX_CENTER_KETI_clicked();
    void on_bt_cobot_move2object_approach_clicked();
    void on_BTN_MOVEL_rel_clicked();
    void on_BTN_quick_return_clicked();
    void on_BTN_GRIPPER_SUCTION_clicked();
    void on_BTN_GRIPPER_OPEN_clicked();
    void on_BTN_MOVE_JOINT_LOW_LEFT_clicked();
    void on_BTN_RETURN_MOVE_JOINT_BOX_CENTER_clicked();
};
#endif // MAINWINDOW_H

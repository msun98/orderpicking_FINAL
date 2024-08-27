#ifndef WEBSOCKET_H
#define WEBSOCKET_H
#include <QWebSocketServer>
#include <QWebSocket>
#include <QIODevice>

#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTimer>
#include <QtCore/QDebug>
#include <QDir>
#include <QSettings>
#include <cmath>
#include <filesystem>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

// 통신을 위해 만든 헤더 삽입.
#include "ipc.h"
#include "mobile_robot.h"
#include "common_header.h"
#include <tbb/concurrent_queue.h>

class websocket : public QObject
{
    Q_OBJECT
public:
    ~websocket();
    explicit websocket(QObject *parent = nullptr);

    //little endian
    typedef union{
        int     data_i;
        char    data_c[4];
    }U_INT_CHAR;

    typedef union{
        short   data_s;
        char    data_c[2];
    }U_SHORT_CHAR;

    // ipc
    //    IPC ipc;

    QString mb_status;
    QString json_mb_status;

    QString md_lift_status;

    bool move_status = false;

    bool msg = false;

    bool charging_status =false;

    QString uuid;

    cv::Mat img, src;//디버깅용 이미지 show하기 위함.

    QByteArray fileData_byte;//유진로봇에서 요청한 내용 보내기 위함.

    QString map_id;

    QFile* file;//file read할 때 사용하기 위함.

    QString map_config_path;

    QString config_path;

    uint32_t last_status_tick = 0;

    uint32_t last_sucess_tick = 0;

    bool connected;

    bool move_flag=false;
    bool rb5_move_flag=false;

    float status_charge,status_power;

    float goal_x[512];
    float goal_y[512];

    double x=0,y=0,theta=0;//for yujin

    void init(mobile_robot *_mb, MD_MOTOR *_md);

    //      double x,y,theta;//for yujin

    float mobile_pose_x,mobile_pose_y,mobile_pose_theta;
    void CMD_RESULT(QString result,QString Error="None");

    QString old_mobile_status;

    QString action;

    int seq_num=0;
    int rb_seq_num=0;
    int lift_seq_num=0;

    QString entry;

    QStringList scene_yujin;

    int cur_step = ROBOT_STATE_NOT_READY;

    int item_count;

    QString rb_status;
    QString rb_pause_status;

    QString lift_status;

    bool flag_circle_yj = false;

    bool seqLoop_done_flag = false;

    std::queue<QString> yujin_json_msg;

    std::map<std::string, SHELF_INFO*> shelf_infos;
    std::map<std::string, OBJECT_INFO*> object_infos;

    QString old_mb_status;

    QJsonObject main_json;

    QByteArray map_buf;

    double calc_theta(cv::Vec2d path,cv::Vec2d cur_path);

    float robot_length_x,robot_length_y;
    QString mobile_fsm_status = "STATE_AUTO_GOAL_REACHED";

    QString mobile_status;

    QString robot_name,robot_id;

    QString cobot_status;
    //    bool lift_move_flag;

    int pick_item_success_count=0;
    int pick_item_failure_count=0;

    QString send_map_name;
    QString send_map_filesize;

//    bool pause_flag = false;

    QString old_uuid,pause_uuid,old_acton;
    bool move_finish_flag = false;

signals:

    void msgSignal(bool msg);

    void msgReciveSignal(QString message);

    void msgSendSignal(QString message);

    void check_robot_connected(bool connected);

    void order_pass(QJsonObject);


public slots:

    void open();

    void Feedback();

    void SEND_NOTICE();

    void onNewConnection();

    void onClosed();

    void onTextMessageReceived(QString message);

    void onBinaryMessageReceived(QByteArray message);

    void MissionCheck(QString uuid);

    void onDisconnected();

    void send_img_package(QString map_config_path,int image_file_size,QString signature,QString fileName);

    void moveCheck();

    void cmd_loop(QWebSocket *pClient_address);

private:
    QTimer *timer;
    QTimer *feedbacktimer;
    QTimer *cmd_que_timer;

    QWebSocketServer *server;
    QList<QWebSocket *> clients;

    void sendNotice(QWebSocket *client_socket);

    void sendCommandAck(QWebSocket *client_socket, QString result, QJsonObject error_info, QString uuid);

    //    void sendCommandResult(QWebSocket *client_socket, QString result, QJsonObject error_info, QString uuid);

    QJsonObject data(QString robot_manufacture, QString action,QString robot_type, QString map_id, QString map_name);

    //    void sendCommandResult(QWebSocket *client_socket, QString action, QString result, QJsonObject data, QJsonObject error_info, QString uuid);

    void sendAck(QString uuid);

    mobile_robot *mb=NULL;
    MD_MOTOR *md_mot = NULL;
};



#endif // WEBSOCKET_H

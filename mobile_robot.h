#ifndef MOBILE_ROBOT_H
#define MOBILE_ROBOT_H

#include "global_defines.h"
#include "md_motor.h"
#include "cobot.h"
#include "common_header.h"

//#define IP "10.108.1.68" //자기자신 ip
#define IP "192.168.2.91" //자기자신 ip
//#define IP "10.108.2.93" //자기자신 ip
#define PORT1 7799 //통신할 포트
#define PORT2 7788 //통신 받을 포트
#define MAP_PORT 5555 //통신 받을 포트

class mobile_robot: public QObject
{
    Q_OBJECT

public:
    explicit mobile_robot(QObject *parent = nullptr);

    QTcpServer *mobile_robot_server;//file header 및 cmd listen 할 용도.
    QTcpSocket *clientSocket;//for robot arrived check

    QTcpServer *mobile_rb_status_server; //mobile에 status 전송 받는용
    QTcpSocket *mobile_status_socket;

    QTcpServer *tcpServer_map; //file 전송용
    QTcpSocket *map_Socket;//for robot arrived check

    QByteArray new_buf;

    bool connect_flag = false;
    int tmp_fd = 0;
    QTimer robotStatusTimer;

    QString get_somthing_msg ;

    void mobile_Connect();
    void onReadyCmdRead();
//    void send_command();
//    void send_command(int code,QString str);

    //    void onMobileConnected();
    //    void onMobiledisConnected();

    void moveCheck(bool flag);

    int lift_hight = 0;
    //    int lift_rpm = 0;
    //    int lift_pos = 0;
    //    int old_lift_pos = 0;
    //    int old_lift_rpm = 0;
    //    int armp = 0;
    //       QString lift_state;

    void init(MD_MOTOR *_md_mot, Cobot *_cobot);
    void liftState(QString lift_state);

    void cmdSendData(QByteArray cmd_msg);
    void getData(QString msg);

    cv::Mat map_img;

    QString _lift_State;
    QFile file;
    QString filename;
    qint64 filesize;
    qint64 recvSize;
    qint64 sendSize;

    double pose_x;
    double pose_y;
    double pose_th;
    double battery;
    bool charge_state;
    bool map_changed=false;
    int status = 0;
    int fsm_status = 0;

    bool move_flag = false;

    //     cv::Mat map_img;
    float mobile_pose_x,mobile_pose_y,mobile_pose_theta;

    QJsonObject json;
    void png_change();
    void bt_zip();

    QString AMR_FSM_status = "STATE_AUTO_GOAL_REACHED";
    QString old_AMR_FSM_status = "STATE_AUTO_GOAL_REACHED";

    bool new_start = true;

    QString uuid;

signals:

    void LiftState();
    void mobile_run(QString run);
    void img_signal(cv::Mat img);
    void mobile_status(QString pose_msg);



private slots:

    void newConnection_tcp();
    void disConnection_tcp();

    void sendData();

    void readyRead_tcp();

    void on_read_mobile_status();

    void on_map_read_command();


    //    void readyRead_tcp();    //    void write_log(Integrated_infomation& Ii, const QString& text);



private:

    Cobot *cobot = NULL;
    MD_MOTOR *md_mot=NULL;
};

#endif // MOBILE_ROBOT_H

#ifndef INTEGRATE_UI_H
#define INTEGRATE_UI_H

#include "global_defines.h"

#include <QObject>
#include <QTcpSocket>
#include <QJsonArray>
#include <QByteArray>
#include <QHostAddress>
#include <QSettings>
#include <QDir>
#include <QTimer>
#include <filesystem>

#include "autocontrol.h"
#include "slam_2d.h"
#include "mobile.h"

//#define IP "10.108.1.68" //
#define IP "192.168.2.91" //
//#define IP "10.108.2.93" //
#define PORT1 7799
#define PORT2 7788
#define MAP_PORT 5555

class INTEGRATE_UI: public QObject
{
    Q_OBJECT

public:
    explicit INTEGRATE_UI(QObject *parent =nullptr);
    //    ~INTEGRATE_UI();

    ipInfo IP_UI;

    QTcpSocket *ui_com;
    QTcpSocket *mobile_status_socket;
    QTcpSocket *map_socket;

    void init(MOBILE *_mobile, AUTOCONTROL *_ctrl, SLAM_2D *_slam);
    void onSocketWrite(QString msg);
    void onMobileStatusSocketWrite(QString socketmsg);
    void onMapImageSocketWrite(QByteArray map_msg);

    void set_file(QString filePath);
    void on_read_command(QString head);

    void send_command(int code,QString str);
    void send_file();
    //    void sendUIStatus(std::automic<int> msg);

    int robot_state = 0;
    int lift_rpm = 0;
    int lift_pos = 0;

    QFile file;
    QByteArray fileData_byte;

    QTimer mobile_status_timer;
    QTimer path_timer;
    QTimer reconnectTimer;

    QString lift_state;
    QString filename;
    qint64 filesize;
    qint64 recvSize;
    qint64 sendSize;

    //    enum UI_AUTO_STATE old_status; //이전 mobile status 올려받아서 현재 status 와 비교하기 위함.

    int old_status=0;
    int status_tick = 1;
    int old_status_tick=0;
    int mobile_status;
    int mobile_moving_status=0;

    tbb::concurrent_queue<cv::Vec3d> command_path_que;
    std::queue<QByteArray> send_mobile_status;

    QString                     change_map_name;

    bool change_map_flag = false;
    QByteArray new_AMR_status;
    std::mutex mtx;

signals:
    void change_map(QString);


public slots:

    // 통신을 주고받기 위함.
    void onUIConnected();
    void onUIdisConnected();
    void onReadyCmdRead();
    //mobile robot상태를 전달하기 위함.
    void sendUIStatus();

    // map/ini file 주고받기 위함.
    void onMobileStatusConnected();
    void onMobileStatusDisConnected();
    void onMobileStatusCmdRead();

    // 실시간 map 이미지 주기 위함.
    void onMapIMGConnected();
    void onMapIMGDISConnected();

    void onReconnectToClients();

    void publish_path(); // publish path give to yujin


private:

    AUTOCONTROL *ctrl = NULL;
    SLAM_2D *slam = NULL;
    MOBILE *mobile = NULL;

};



#endif // INTEGRATE_UI_H

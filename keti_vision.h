#ifndef KETI_VISION_H
#define KETI_VISION_H

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QHostAddress>
#include <QTimer>
#include <QImageReader>
#include <QBuffer>
#include <QTime>

//#include <eigen3/Eigen/Core>
//#include <Eigen/Dense>

//#include <opencv2/core/eigen.hpp>

#include "common_header.h"
#include "global_defines.h"
#include "option_writer.h"

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

class Keti_vision : public QObject
{
    Q_OBJECT

public:
    explicit Keti_vision(QObject *parent = nullptr);

    QTimer keti_restart_timer;

    Option_Writer settings;

    bool connect_flag = false;

    ipInfo IP_VISION;
    QTcpSocket *Keti_Client;

    int X, Y, Z;

    void write_log(Integrated_infomation& Ii, const QString& text);

    void saveEstimated_GrapsPoints(const QString& path);
    void loadEstimated_GrapsPoints(const QString& path);

    void c_p2c_tcp(QString str);

    Eigen::Matrix4d TCP2cam;

    QString res_x,res_y,res_z;
    QString res_rx,res_ry,res_rz,res_wrist_rot;
    QByteArray new_buf;

    QTimer connectTimer;

    void Vision_Connect();

    cv::Mat vision_img;

    bool capture_flag = false;
    int i = 1;

    Eigen::Matrix4d TF;

    float keti_x,keti_y,keti_z;
    float keti_rx,keti_ry,keti_rz;
    float keti_app_x,keti_app_y,keti_app_z;

    QString old_timeString;

    bool time_change = 0;

    Eigen::Matrix4d BoxCent(QString str);

    bool box_cent_value=false;



signals:

    void SIGNAL_LOG_WRITE();

    void msgSignal(bool connect_flag);

    void keti_img_signal(cv::Mat img);

    void keti_img_point(QString point);



public slots:

    void onReadyCmdRead();

    void onKetiConnected();

    void onKetidisConnected();

    void onReconnectToServer();

};

#endif // KETI_VISION_H

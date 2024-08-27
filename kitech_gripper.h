#ifndef KITECH_GRIPPER_H
#define KITECH_GRIPPER_H

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QHostAddress>
#include <QTimer>

#include "common_header.h"

//for using linux socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class Kitech_gripper : public QObject
{
    Q_OBJECT

public:
    explicit Kitech_gripper(QObject *parent = nullptr);

    int tmp_fd = 0;

    ipInfo IP_GRIPPER;
    QTcpSocket *Kitech_Client;

    void write_log(Integrated_infomation& Ii, const QString& text);
    void Gripper_Connect();

    bool connect_flag = false;


    QTimer connectTimer;

//    void kitech_cmd(QString msg);

signals:
    void SIGNAL_LOG_WRITE();

//    void Gripper_Connect();
    void kitech_cmd(QString msg);

    void msgSignal(bool connect_flag);


public slots:
    void onReadyCmdRead();

    void onReconnectToServer();

//    void msgSignal(bool connect_flag);


    void onKitechConnected();
    void onKitechdisConnected();

};

#endif // KITECH_GRIPPER_H

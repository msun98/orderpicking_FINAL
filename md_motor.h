#ifndef MD_MOTOR_H
#define MD_MOTOR_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QtDebug>

#include <iostream>
#include "common_header.h"
#include "global_defines.h"
//#include "mobile_robot.h"

class MD_MOTOR : public QObject
{
    Q_OBJECT

public:
    explicit MD_MOTOR(QObject *parent = nullptr);

//    mobile_robot mb;
    QSerialPort motor;

    bool move_poisition_flag=false;
    bool init_flag = true;
    int rev_data_size = 0;
    QByteArray buf;
    MOTOR_MAIN_DATA main_data;

    int pose;

    void open(QString port);
    void req_data();
    void test_pid();

    void set_maxVel_pos(unsigned short val);

    void move_rpm(short val);

    void move_position(int val);

    void homing(int dir);
    void write_data(int pid, int data);

    void checkSum(QByteArray &data);

    MOTOR_MAIN_DATA get_main_data();


signals:
    void rev_DATA(QByteArray data);
    void write_log(QString str);

    // ui에 connect됨을 확인하기 위함.
    void LiftMSG(QString msg);

    void homingEnd(QString msg);

private slots:
    void onReadyCmdRead();
};

#endif // MD_MOTOR_H

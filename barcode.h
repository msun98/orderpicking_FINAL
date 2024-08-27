#ifndef BARCODE_H
#define BARCODE_H

#include <QObject>
#include <QHostAddress>
#include <iostream>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

class Barcode: public QObject
{
    Q_OBJECT

public:
    explicit Barcode(QObject *parent = nullptr);

    void bt_usb1_con(QString bar1);
    void bt_usb2_con(QString bar2);

    void bt_usb1_dis();
    void bt_usb2_dis();

    QSerialPort* m_serialPort1 = nullptr;
    QSerialPort* m_serialPort2 = nullptr;

    QString msg1="usb1 off";
    QString msg2="usb2 off";

    QString total_msg;

    QTimer barcode_Timer;
//     void send_msg_ui(QString msg1,QString msg2);

private slots:
    void readData_1();
    void readData_2();

    void send_msg_ui();

signals:
    void msgSignal(QByteArray msg);
    void msgSignal2(QByteArray msg);

    void barcode_UI(QString msg);

//    void handleError();
};

#endif // BARCODE_H

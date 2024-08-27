#include "kitech_gripper.h"


Kitech_gripper::Kitech_gripper(QObject *parent) : QObject(parent)
{
    Kitech_Client = new QTcpSocket(this);

    IP_GRIPPER.ip = "192.168.2.73";
    IP_GRIPPER.port = 7003;

    /*connect(Kitech_Client, SIGNAL(connected()), this, SLOT(onKitechConnected()));
    connect(Kitech_Client, SIGNAL(disconnected()), this, SLOT(onKitechdisConnected()));

    //    connect(this, SIGNAL(Gripper_Connect()), this, SLOT(msgSignal(bool connect_flag)));

    connectTimer.start(500);
    connect(&connectTimer, SIGNAL(timeout()), this, SLOT(onReconnectToServer()));

    connect_flag = false;
    emit msgSignal(connect_flag);

    Gripper_Connect();*/
}



void Kitech_gripper::Gripper_Connect()
{
    //    qDebug()<<"wait Connecting";
    if(!Integrated_info.init_gripper)
    {
        //                        Keti_Client->abort();
        Kitech_Client->connectToHost(QHostAddress(IP_GRIPPER.ip), IP_GRIPPER.port);
        connect(Kitech_Client, SIGNAL(readyRead()), this, SLOT(onReadyCmdRead()));
        //        qDebug()<<"gripper connect!!!!!!!!!!!!!!!!";
        //        Integrated_info.init_gripper =false;
        connect_flag = false;
        emit msgSignal(connect_flag);
    }

    else
    {
        connectTimer.stop();
        connect_flag = true;
        emit msgSignal(connect_flag);
    }
}

void Kitech_gripper::onReconnectToServer()
{
    //        qDebug()<<"working";

    Integrated_info.init_gripper = false;
    Gripper_Connect();

    if (!Kitech_Client->isOpen()) {

    }
}

void Kitech_gripper::onReadyCmdRead()
{
    QByteArray Read_Data = Kitech_Client->readAll();
    QString buf = QString(Read_Data);
    qDebug()<<"KETECH : "<<buf;

    qDebug()<<"Read_Data size : "<<Read_Data.size();
    if(Read_Data.size()>10)
    {
        // 관절의 개수는 12개임.
        QStringList list = buf.split(",");
        QString joint_buf;
        qDebug()<<"list size : "<<list.size();
        for(int i = 0;i<list.size();i++)
        {
            joint_buf+=QString::number(list[i].toFloat()*0.1)+", ";
            emit kitech_cmd(joint_buf);
        }
    }
    else
    {
        emit kitech_cmd(buf);
    }

    Integrated_info.kitech_res = QString(Read_Data);


}

void Kitech_gripper::onKitechConnected()
{
    qDebug()<<"손가락 통신중.";
    Integrated_info.init_gripper = true;
    connect_flag = true;
    emit msgSignal(connect_flag);
    write_log(Integrated_info, "[Gripper] Server connected.\n");
    Gripper_Connect();
}

void Kitech_gripper::onKitechdisConnected()
{
    qDebug()<<"손가락 통신끊김.";
    Integrated_info.init_gripper = false;
    connect_flag = false;
    emit msgSignal(connect_flag);
    write_log(Integrated_info, "[Gripper] Server disconnected.\n");
    connectTimer.start();
}

void Kitech_gripper::write_log(Integrated_infomation& Ii, const QString& text)
{
    Ii.Log_info.append(text);
    emit SIGNAL_LOG_WRITE();
}


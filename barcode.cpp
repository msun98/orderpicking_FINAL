#include "barcode.h"

Barcode::Barcode(QObject *parent) : QObject(parent)
{
    m_serialPort1 = new QSerialPort(this);
    m_serialPort2 = new QSerialPort(this);

    connect(m_serialPort1, &QSerialPort::readyRead, this, &Barcode::readData_1);
    //    connect(m_serialPort1, &QSerialPort::errorOccurred, this, &Barcode::handleError);

    connect(m_serialPort2, &QSerialPort::readyRead, this, &Barcode::readData_2);
    //    connect(m_serialPort2, &QSerialPort::errorOccurred, this, &Barcode::handleError);

    //    connect(this, SIGNAL(readData_1), this, SLOT(msgSignal(QString msg)));
    //    connect(this, SIGNAL(readData_2), this, SLOT(msgSignal(QString msg)));

    connect(&barcode_Timer, SIGNAL(timeout()), this, SLOT(send_msg_ui()));
    barcode_Timer.start(1000);
    bt_usb1_con("/dev/ttyBAR0");
    bt_usb2_con("/dev/ttyBAR1");

    //    send_msg_ui

}

void Barcode::bt_usb1_con(QString bar1)
{

    //    qDebug()<<"open port : "<<ui->cb_usb1->currentText();
    // 시리얼 설정 코드
    //    m_serialPort->setPortName("/dev/ttyUSB0"); // 포트 이름 지정
    //    m_serialPort1->setPortName(ui->cb_usb1->currentText()); // 포트 이름 지정
    //    bar1
    //    std::cout<<"barcode open : "<<bar1.toStdString()<<std::endl;
    m_serialPort1->setPortName(bar1); // 포트 이름 지정
    m_serialPort1->setBaudRate(QSerialPort::Baud9600); // baud: 초당 신호(siganl) 요소의 수 , 예) 하나의 버드에 2bit 있다면 1Baud 동안 2bit 전송 됨
    m_serialPort1->setDataBits(QSerialPort::Data8); // dataBits

    //    m_serialPort->setBaudRate(QSerialPort::Baud9600); // baud: 초당 신호(siganl) 요소의 수 , 예) 하나의 버드에 2bit 있다면 1Baud 동안 2bit 전송 됨
    //    m_serialPort->setDataBits(QSerialPort::Data104); // dataBits
    m_serialPort1->setParity(QSerialPort::NoParity); // 정보 전달 과정에 오류가 생겼는지 검사하기 위한 것
    m_serialPort1->setStopBits(QSerialPort::OneStop); // 포트를 열기전에 set 또는 success 하면 return true로 반환된다.
    m_serialPort1->setFlowControl(QSerialPort::NoFlowControl); // 흐름제어
    if (m_serialPort1->open(QIODevice::ReadWrite)) { //시리얼 장치 연 후 작업들


        //        ui->bt_usb1_con->setEnabled(false);
//        std::cout<<"LEFT barcode open : "<<bar1.toStdString()<<std::endl;

        //show ui open msg
        msg1 = "usb1 on";
        //        emit barcode_UI(msg);
        //        ui->bt_usb1_dis->setEnabled(true);
    }
    else {
        std::cout<<"barcode ERROR"<<std::endl;
        //        QMessageBox::critical(this, tr("Error"), m_serialPort->errorString());
        //show ui open msg
        msg1 = "usb1 off";
        //        emit barcode_UI(msg);
    }
}


void Barcode::bt_usb2_con(QString bar2)
{

    //    qDebug()<<"open port : "<<ui->cb_usb2->currentText();
    // 시리얼 설정 코드
    //    m_serialPort->setPortName("/dev/ttyUSB0"); // 포트 이름 지정
    m_serialPort2->setPortName(bar2); // 포트 이름 지정
    m_serialPort2->setBaudRate(QSerialPort::Baud9600); // baud: 초당 신호(siganl) 요소의 수 , 예) 하나의 버드에 2bit 있다면 1Baud 동안 2bit 전송 됨
    m_serialPort2->setDataBits(QSerialPort::Data8); // dataBits

    //    m_serialPort->setBaudRate(QSerialPort::Baud9600); // baud: 초당 신호(siganl) 요소의 수 , 예) 하나의 버드에 2bit 있다면 1Baud 동안 2bit 전송 됨
    //    m_serialPort->setDataBits(QSerialPort::Data104); // dataBits
    m_serialPort2->setParity(QSerialPort::NoParity); // 정보 전달 과정에 오류가 생겼는지 검사하기 위한 것
    m_serialPort2->setStopBits(QSerialPort::OneStop); // 포트를 열기전에 set 또는 success 하면 return true로 반환된다.
    m_serialPort2->setFlowControl(QSerialPort::NoFlowControl); // 흐름제어
    if (m_serialPort2->open(QIODevice::ReadWrite)) { //시리얼 장치 연 후 작업들


        //        ui->bt_usb2_con->setEnabled(false);
//        std::cout<<"RIGHT barcode open : "<<bar2.toStdString()<<std::endl;
        //        ui->bt_usb2_dis->setEnabled(true);

        //show ui open msg
        msg2 = "usb2 on";
        //        emit barcode_UI(msg);
    }
    else {
        std::cout<<"ERROR"<<std::endl;
        //        QMessageBox::critical(this, tr("Error"), m_serialPort->errorString())

        //show ui open msg
        msg2 = "usb2 off";
        //        emit barcode_UI(msg);

    }

}

void Barcode::send_msg_ui()
{
    if (msg1=="usb1 on")
    {
        if (msg2=="usb2 on")
        {
            total_msg="all usb is on";
//            qDebug("all usb is on");
        }

        else if (msg2 == "usb2 off")
        {
            total_msg="usb1 on, usb2 off";
            qDebug("usb1 on, usb2 off");
        }
    }

    if (msg1=="usb2 off")
    {
        if (msg2=="usb1 on")
        {
            total_msg="usb1 on, usb2 off";
            qDebug("usb1 on, usb2 off");
        }

        else if (msg2=="usb1 off")
        {
            total_msg="all usb is off";
            qDebug("all usb is off");
        }
    }

    emit barcode_UI(total_msg);

}

void Barcode::bt_usb1_dis()
{
    if (m_serialPort1->isOpen()) // 시리얼 포트 열렸으면
        m_serialPort1->close(); // 시리얼 닫아라
    std::cout<<"close usb1"<<std::endl;
    //    ui->bt_usb1_con->setEnabled(true);
    //    ui->bt_usb1_dis->setEnabled(false);
}

void Barcode::bt_usb2_dis()
{
    if (m_serialPort2->isOpen()) // 시리얼 포트 열렸으면
        m_serialPort2->close(); // 시리얼 닫아라
    std::cout<<"close usb2"<<std::endl;
    //    ui->bt_usb2_con->setEnabled(true);
    //    ui->bt_usb2_dis->setEnabled(false);
}

void Barcode::readData_1()
{

    QByteArray serialData = m_serialPort1->readAll();
    qDebug() << serialData;
    emit msgSignal(serialData);
    //    ui->te_usb1->setText(serialData);

}

void Barcode::readData_2()
{

    QByteArray serialData = m_serialPort2->readAll();
    qDebug() << serialData;
    emit msgSignal2(serialData);
    //    ui->te_usb2->setText(serialData);

}

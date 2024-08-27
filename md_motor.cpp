#include "md_motor.h"

//usb 권힌 에러 발생 시 해결 코드
// sudo adduser $USER dialout

MD_MOTOR::MD_MOTOR(QObject *parent) : QObject(parent)
{
    connect(&motor, SIGNAL(readyRead()), this, SLOT(onReadyCmdRead()));
}

void MD_MOTOR::open(QString port)
{
    if(motor.isOpen())
    {
        motor.close();
    }

    motor.setPortName(port);
    motor.setBaudRate(QSerialPort::Baud19200);
    motor.setDataBits(QSerialPort::Data8);
    motor.setStopBits(QSerialPort::OneStop);
    motor.setParity(QSerialPort::NoParity);

    if(!motor.open(QIODevice::ReadWrite))
    {
        printf("motor connection failed.\n");
        QString msg = "false";
        //        qDebug()<<msg;
        emit LiftMSG(msg);
    }
    else
    {
        printf("motor connection successed.\n");
        write_log(QString("motor connection successed.\n"));
        usleep(100*100);
        motor.write(MD_CMD.RELEASE_LIMIT_SW);

        QString msg = "true";
        //        qDebug()<<msg;
        emit LiftMSG(msg);
    }
}

void MD_MOTOR::req_data()
{
    if(!motor.isOpen())
    {
        return;
    }

    motor.write(MD_CMD.REQ_MAIN_DATA);
}

void MD_MOTOR::set_maxVel_pos(unsigned short val)
{
    // PID 176
    QByteArray POSI_MAX_VEL;
    POSI_MAX_VEL.resize(7);
    POSI_MAX_VEL[0] = 0xb7; //183
    POSI_MAX_VEL[1] = 0xac; //172
    POSI_MAX_VEL[2] = 0x01; //1
    POSI_MAX_VEL[3] = 0xb0; //176
    POSI_MAX_VEL[4] = 0x02; //2

    memcpy(POSI_MAX_VEL.data() + 5, &val, 2); //183, TMID, ID, 176, 2, D1, D2, CHK (2byte)
    checkSum(POSI_MAX_VEL);
    motor.write(POSI_MAX_VEL);
    qDebug() << POSI_MAX_VEL;
}

void MD_MOTOR::move_position(int val)
{
    move_poisition_flag = true;

    if(!motor.isOpen())
    {
        printf("motor not connected.\n");
        return;
    }
    // qDebug()<<"val1 : "<<val;
    if (val>620)
    {
        val = 620;
    }
    else if(val<0)
    {
        val = 0;
    }
    val = val*4700/620;

    if (val > 4700){
        val = 4700;
    }
    else if(val < 0){
        val = 0;
    }

    QByteArray move;
    move.resize(9);
    move[0] = 0xb7;
    move[1] = 0xac;
    move[2] = 0x01;
    move[3] = 0xf3;
    move[4] = 0x04;

    memcpy(move.data() + 5, &val, 4);
    checkSum(move);
    motor.write(move);

    QString str = "motor run, POS : " + QString::number(val);
    //    qDebug()<<str;
    write_log(str);
}

void MD_MOTOR::homing(int dir)
{
    if(!motor.isOpen())
    {
        printf("motor not connected.\n");
        return;
    }

    printf("motor homing.\n");

    if(dir == CCW)
    {
        motor.write(MD_CMD.RELEASE_LIMIT_SW);
        motor.write(MD_CMD.INIT_SET);
        usleep(1000*100);
        motor.write(MD_CMD.INIT_SET_CCW);
    }
    else
    {
        motor.write(MD_CMD.RELEASE_LIMIT_SW);
        motor.write(MD_CMD.INIT_SET);
        usleep(1000*100);
        motor.write(MD_CMD.INIT_SET_CW);
    }
    emit homingEnd("homing end");
}

void MD_MOTOR::write_data(int pid, int data)
{

}

void MD_MOTOR::onReadyCmdRead()
{
    // this fucn act only PID 193
    QByteArray _buf = motor.readAll();
    if(_buf.size() > 0)
    {
        buf.append(_buf);

        if(buf.size() < 5)
        {
            return;
        }

        bool is_header = false;
        for(int p = 0; p < buf.size()-1; p++)
        {
            // header check
            if(buf[p] == (char)0xac && buf[p+1] == (char)0xb7
                    && buf[p+2] == (char)0x01 && buf[p+3] == (char)0xc1 && buf[p+4] == (char)0x11)
            {
                buf.remove(0, p);
                is_header = true;

                // 172,183,1,193,17,D1, ..., D17,CHK -> size:23
                const int packet_size = 23;
                if(is_header && buf.size() >= packet_size)
                {
                    uchar* body = (uchar*)buf.data();

                    MOTOR_MAIN_DATA _main_data;
                    memcpy(&_main_data.motor_rpm, &body[5], 2);
                    memcpy(&_main_data.motor_armp, &body[7], 2);
                    memcpy(&_main_data.motor_controlType, &body[9], 1);
                    memcpy(&_main_data.motor_ref_rpm, &body[10], 2);
                    memcpy(&_main_data.motor_controlOutput, &body[12], 2);
                    memcpy(&_main_data.motor_status, &body[14], 1);
                    memcpy(&_main_data.motor_position, &body[15], 4);
                    memcpy(&_main_data.motor_brake_duty, &body[19], 1);
                    memcpy(&_main_data.motor_temperature, &body[20], 1);

                    buf.remove(0, buf.size());

                    pose = _main_data.motor_position*680/2700;

                    main_data = _main_data;
                    //                break;
                }

            }
            else if(buf[p] == (char)0xac && buf[p+1] == (char)0xb7 && buf[p+2] == (char)0x01 && buf[p+3] == (char)0x22)
            {
                // 172,183,1,34,1,DATA,CHK -> size:6
                buf.remove(0, p);

                // 172,183,1,193,17,D1, ..., D17,CHK -> size:23
                const int packet_size = 6;
                if(buf.size() >= packet_size)
                {
                    uchar* body = (uchar*)buf.data();

//                    MOTOR_MAIN_DATA _main_data;
                    int status;
                    memcpy(&status, &body[5], 1);
                }
            }
        }
    }
}

MOTOR_MAIN_DATA MD_MOTOR::get_main_data()
{
    MOTOR_MAIN_DATA _main_data = main_data;
    return _main_data;
}

void MD_MOTOR::checkSum(QByteArray &data)
{
    uchar cs = 0;
    for(int i =0; i<data.size(); ++i)
    {
        //        qDebug()<<"data.size() :"<<data.size();
        cs += *(data.data() + i);
        //        qDebug()<<"cs :"<<cs;
    }
    cs = ~cs;
    cs = cs + 1;
    //qDebug() << cs;
    data.append(cs);
}

void MD_MOTOR::move_rpm(short val)
{
    move_poisition_flag = true;

    if(!motor.isOpen())
    {
        write_log(QString("motor not connected."));
        return;
    }

    QByteArray move;
    move.resize(7);
    move[0] = 0xb7;
    move[1] = 0xac;
    move[2] = 0x01;
    move[3] = 0x82;
    move[4] = 0x02;

    memcpy(move.data() + 5, &val, 2);
    checkSum(move);

    //qDebug() << move;
    motor.write(move);
    //    QString str = "motor run, RPM : " + QString::number(val);
    write_log(move);
}

void MD_MOTOR::test_pid()
{
    motor.write(MD_CMD.RELEASE_LIMIT_SW);
}

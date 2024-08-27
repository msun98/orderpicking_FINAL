#include "keti_vision.h"
//class Integrated_infomation Integrated_info;//헤더 클래스 사용한다고 선언.
Keti_vision::Keti_vision(QObject *parent) : QObject(parent)
{
    Keti_Client = new QTcpSocket(this);
    IP_VISION.ip = "192.168.2.209";
    //        IP_VISION.ip = "10.108.2.210";
    IP_VISION.port = 7777;

    connect(Keti_Client, SIGNAL(connected()), this, SLOT(onKetiConnected()));
    connect(Keti_Client, SIGNAL(disconnected()), this, SLOT(onKetidisConnected()));

    connectTimer.start(200);
    connect(&connectTimer, SIGNAL(timeout()), this, SLOT(onReconnectToServer()));

    qDebug()<<"start keti tcp";
    Vision_Connect();

    //    TF = cam2TCP_TF();
    //    TCP2cam = cam_config.RB_TF_SENSOR;
    //    std::cout<<TCP2cam<<std::endl;


}

void Keti_vision::Vision_Connect()
{

    //    qDebug()<<"dd :"<<Integrated_info.init_vision;
    // 가장 최초에 사용할 때에는 false로 설정됨. connection 이후에는 true로 변경.

    if(!Integrated_info.init_vision)
    {
        //                        Keti_Client->abort();
        Keti_Client->connectToHost(QHostAddress(IP_VISION.ip), IP_VISION.port);
        connect(Keti_Client, SIGNAL(readyRead()), this, SLOT(onReadyCmdRead()));
        //        qDebug()<<"vision connect!!!!!!!!!!!!!!!!";
        connect_flag = false;
        emit msgSignal(connect_flag);
    }

    else
    {
        //        qDebug()<<"aaaaaaaaaaaaaa";
        connectTimer.stop();
        connect_flag = true;
        emit msgSignal(connect_flag);
    }

}
void Keti_vision::onKetiConnected()
{
    qDebug()<<"통신중.";
    Integrated_info.init_vision = true;
    write_log(Integrated_info, "[Vision] Server connected.\n");
    Vision_Connect();
}

void Keti_vision::onKetidisConnected()
{
    //
    Integrated_info.init_vision = false;
    qDebug()<<"통신끊김.";
    //    Integrated_info.init_vision = false;
    connect_flag = false;
    emit msgSignal(connect_flag);
    write_log(Integrated_info, "[Vision] Server disconnected.\n");
    connectTimer.start();
}

void Keti_vision::onReconnectToServer()
{
    //    qDebug()<<"working";

    Integrated_info.init_vision = false;
    Vision_Connect();

    if (!Keti_Client->isOpen()) {

    }
}

QByteArray new_buf;
//int packet_size;
cv::Mat new_img;

void Keti_vision::onReadyCmdRead()
{
    QByteArray Read_Data = Keti_Client->readAll();
    QString buf = QString(Read_Data);
    QStringList list = buf.split(", ");

    QTime time = QTime::currentTime();
    QString timeString = time.toString();

    if(list[0] == "KETI")
    {
        if(old_timeString != timeString)
        {
            qDebug()<<list.size();
            if(list.size()>4)
            {
                //                            qDebug()<<list;
                qDebug()<<"coordinate : "<<list[1]<<","<<list[2]<<","<<list[3]<<","<<list[4]<<","<<list[5]<<","<<list[6]<<","<<list[7];

                res_x = list[1];
                res_y = list[2];
                res_z = list[3];

                res_rx = list[4];
                res_ry = list[5];
                res_rz = list[6];
                //                res_rz = "1";

                res_wrist_rot = list[7];
                //        new_buf.remove(0, Read_Data.size());

                QString str;
                str.sprintf("%.3f, %.3f, %.3f",res_x.toFloat(),res_y.toFloat(),res_z.toFloat());
                //            c_p2c_tcp(str);

                QString rstr;
                rstr.sprintf("%.3f, %.3f, %.3f",res_rx.toFloat(),res_ry.toFloat(),-1*res_rz.toFloat());
                c_p2c_tcp(str+", "+rstr);
                keti_img_point(str+", "+rstr);
                //        Read_Data.remove(0, Read_Data.size());
                Read_Data.clear();
            }
            else
            {

                //                롤, 피치, 요 안들어오는 부분 수정해야함.
                qDebug()<<list;
                qDebug()<<"coordinate : "<<list[1]<<","<<list[2]<<","<<list[3]<<","<<list[4]<<","<<list[5]<<","<<list[6];

                res_x = list[1];
                res_y = list[2];
                res_z = list[3];

                QString str;
                str.sprintf("%.3f, %.3f, %.3f",res_x.toFloat(),res_y.toFloat(),res_z.toFloat());
                //            c_p2c_tcp(str);

                c_p2c_tcp(str);
                keti_img_point(str);
                //        Read_Data.remove(0, Read_Data.size());
                Read_Data.clear();
            }
        }
    }

    else if(list[0] == "BOX_CENT")
    {
        if(old_timeString != timeString)
        {

            //        qDebug()<<"coordinate : "<<list[1]<<","<<list[2]<<","<<list[3]<<","<<list[4]<<","<<list[5]<<","<<list[6];
            //        qDebug()<<"BOX_CENT coordinate : "<<list[1]<<","<<list[2]<<","<<list[3];
            res_x = list[1];
            res_y = list[2];
            // 로봇의 높이
            res_z = list[3];
            //            float zz = res_z.toFloat()-0.5;
            //            float new_zz = res_z.toFloat()-zz;
            //            res_z = "0.10";

            res_rx = list[4];
            res_ry = list[5];
            res_rz = list[6];
            //        new_buf.remove(0, Read_Data.size());


            QString str;
            str.sprintf("%.3f, %.3f, %.3f",res_x.toFloat(),res_y.toFloat(),res_z.toFloat()-0.50);

            QString rstr;
            rstr.sprintf("%.3f, %.3f, %.3f",res_rx.toFloat(),res_ry.toFloat(),res_rz.toFloat());

            qDebug()<<"BOX_CENT "+str+", "+rstr;
            Eigen::Matrix4d cam2box_center_tf = BoxCent(str+", "+rstr);

            cv::Mat cam2box_center;
            cv::eigen2cv(cam2box_center_tf,cam2box_center);
            //            std::cout<<cam2box_center<<std::endl;

            cv::Mat TCP2cam_cv;
            cv::eigen2cv(TCP2cam,TCP2cam_cv);

            double data3[]={0,0,0,1};
            cv::Mat my_mat(4,1,CV_64FC1,data3);

            cv::Mat box_center_qr2cam = TCP2cam_cv*cam2box_center*TCP2cam_cv.inv()*my_mat;

            keti_x = box_center_qr2cam.ptr<double>(0)[0];
            keti_y = box_center_qr2cam.ptr<double>(0)[1];
            keti_z = box_center_qr2cam.ptr<double>(0)[2];
            //        std::cout<<"box_center_qr2cam : "<<box_center_qr2cam<<std::endl;
            box_cent_value = true;

            qDebug()<<"box_cent_value : "<<box_cent_value;

            keti_img_point(str+", "+rstr);
            Read_Data.clear();
        }
    }

    else if(buf == "OBJ_NONE")
    {
        if(old_timeString != timeString)
        {
            keti_img_point(buf);
        }
    }
    else
    {
        //        new_buf.clear();
        new_buf.append(Read_Data);
        //                qDebug()<<"size : "<<new_buf.size();
        //        qDebug()<<"new_buf[0] : "<<new_buf[0];
        //        qDebug()<<"new_buf[1] : "<<new_buf[1];
        //        qDebug()<<"new_buf[2] : "<<new_buf[2];
        const int packet_size = 300*500*3;
        //        qDebug()<<"packet_size : "<<packet_size;
        if (new_buf!=0)
        {
            QByteArray chop_byte = new_buf.chopped(3);
            QString byte(chop_byte);
            //            qDebug()<<byte;

            if(byte == "IMG")
            {
                if(new_buf.size() >= packet_size)
                {
                    uchar* body = (uchar*)new_buf.data();
                    cv::Mat new_img = cv::Mat::zeros(300,500,CV_8UC3);

                    memcpy(new_img.data,&body[3],300*500*3);
                    //                        imshow("img",new_img);
                    vision_img = new_img.clone();
                    //            qDebug()<<capture_flag;

                    emit keti_img_signal(vision_img);
                    //            new_buf.remove(0, packet_size);
                    new_buf.clear();
                    Read_Data.clear();
                    //            Read_Data.remove(0, packet_size);
                }

            }
            else{
                new_buf.clear();
                Read_Data.clear();
            }
        }
    }
    old_timeString = timeString;
}

void Keti_vision::saveEstimated_GrapsPoints(const QString& path)
{
    if(Integrated_info.keti_grapsCoord.x != -1)
        settings.WriteSettingFile_VISION(path.toStdString(), Integrated_info.keti_grapsCoord);
    else
        write_log(Integrated_info, "[Vision] need est point.\n");
}

void Keti_vision::loadEstimated_GrapsPoints(const QString &path)
{
    settings.ReadSettingFile_VISION(path.toStdString(), Integrated_info.keti_grapsCoord);
}

void Keti_vision::write_log(Integrated_infomation& Ii, const QString& text)
{
    Ii.Log_info.append(text);
    emit SIGNAL_LOG_WRITE();
}

void Keti_vision::c_p2c_tcp(QString str)
{
    qDebug()<<str;
    QStringList list = str.split(",");
    if(list.size() <= 3)
    {
        double tx = list[0].toDouble();
        double ty = list[1].toDouble();
        double tz = list[2].toDouble();

        Eigen::Vector3d P(tx,ty,tz);
        Eigen::Vector3d _P = TCP2cam.block(0,0,3,3)*P+TCP2cam.block(0,3,3,1);

        keti_x = _P[0];
        keti_y = _P[1];
        keti_z = _P[2];

        //////////////////////////////////////////////////////
        //for approach
        double approach_tz = tz - 0.10;
        Eigen::Vector3d app_P(tx,ty,approach_tz);
        Eigen::Vector3d app_trance_P = TCP2cam.block(0,0,3,3)*app_P+TCP2cam.block(0,3,3,1);

        keti_app_x = app_trance_P[0];
        keti_app_y = app_trance_P[1];
        keti_app_z = app_trance_P[2];
        //////////////////////////////////////////////////////

    }
    else
    {

        double tx = list[0].toDouble();
        double ty = list[1].toDouble();
        double tz = list[2].toDouble();

        //for using keti normal vector!!
        double rx = list[3].toDouble();
        double ry = list[4].toDouble();
        double rz = list[5].toDouble();

        //        double rx = 0.0;
        //        double ry = 0.0;
        //        double rz = 1.0;

        Eigen::Vector3d P(tx,ty,tz);
        Eigen::Vector3d _P = TCP2cam.block(0,0,3,3)*P+TCP2cam.block(0,3,3,1);

        keti_x = _P[0];
        keti_y = _P[1];
        keti_z = _P[2];

        //////////////////////////////////////////////////////
        //for approach
        double approach_tz = tz - 0.10;
        Eigen::Vector3d app_P(tx,ty,approach_tz);
        Eigen::Vector3d app_trance_P = TCP2cam.block(0,0,3,3)*app_P+TCP2cam.block(0,3,3,1);

        keti_app_x = app_trance_P[0];
        keti_app_y = app_trance_P[1];
        keti_app_z = app_trance_P[2];
        //////////////////////////////////////////////////////

        Eigen::Vector3d RP(rx,ry,rz);
        Eigen::Vector3d R_P = TCP2cam.block(0,0,3,3)*RP+TCP2cam.block(0,3,3,1);

        keti_rx = R_P[0];
        keti_ry = R_P[1];
        keti_rz = R_P[2];

        QString value;
        value.sprintf("nomal vector is : %f, %f, %f",keti_rx,keti_ry,keti_rz);

        qDebug()<<value;
        //    std::cout<<"TCP2cam : "<<TCP2cam<<std::endl;
    }
}


Eigen::Matrix4d Keti_vision::BoxCent(QString str)
{
    QStringList list = str.split(",");
    double tx = list[0].toDouble();
    double ty = list[1].toDouble();
    double tz = list[2].toDouble();

    //    cv::Mat cam2box_center_off;

    Eigen::Matrix3d T_cam2box_center;
    T_cam2box_center << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = T_cam2box_center.matrix();
    res(0,3) = 1*tx;
    res(1,3) = 1*ty;
    res(2,3) = 1*tz;
    res(3,3) = 1;

    //    std::cout<<res<<std::endl;

    return res;
}

#include "mobile_robot.h"

mobile_robot::mobile_robot(QObject *parent) : QObject(parent)
{
    mobile_Connect();
}


void mobile_robot::mobile_Connect()
{

    //    mobile_robot_server.listen(QHostAddress(QString("192.168.2.90")), 7799);

    // for mobile robot
    clientSocket = new QTcpSocket();
    clientSocket->setSocketOption(QAbstractSocket::LowDelayOption,1);

    mobile_robot_server = new QTcpServer();
    mobile_robot_server->listen(QHostAddress(IP), PORT1);

    mobile_status_socket = new QTcpSocket();
    mobile_status_socket->setSocketOption(QAbstractSocket::LowDelayOption,1);

    mobile_rb_status_server = new QTcpServer();
    mobile_rb_status_server->listen(QHostAddress(IP), PORT2);

    //     std::cout <<"TCP FLAG : " <<connect_flag << std::endl;
    connect(mobile_robot_server, SIGNAL(newConnection()), this, SLOT(newConnection_tcp()));
    connect(mobile_robot_server, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    connect(mobile_rb_status_server,SIGNAL(newConnetion()),this,SLOT(newConnection_tcp()));
    //    connect(mobile_rb_status_server,SIGNAL(disconnected()),this,SLOT(newConnection_tcp()));

    /////////////////////for get maps//////////////////////////////////////////////
    tcpServer_map = new QTcpServer();
    map_Socket = new QTcpSocket();

    map_Socket->setSocketOption(QAbstractSocket::LowDelayOption,1);

    tcpServer_map->listen(QHostAddress(IP), MAP_PORT);

    connect(tcpServer_map,SIGNAL(newConnection()),this,SLOT(newConnection_tcp()));
}

void mobile_robot::newConnection_tcp()
{
    while (mobile_robot_server->hasPendingConnections())
    {
        clientSocket = mobile_robot_server->nextPendingConnection();
        clientSocket->setSocketOption(QAbstractSocket::LowDelayOption,1);
        connect_flag = true;
        connect(clientSocket, SIGNAL(readyRead()), this, SLOT(readyRead_tcp()));
        connect(clientSocket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));
        //        connect(&robotStatusTimer, SIGNAL(timeout()), this, SLOT(sendData()));
        robotStatusTimer.start(200);
    }

    while (mobile_rb_status_server->hasPendingConnections())
    {
        mobile_status_socket = mobile_rb_status_server->nextPendingConnection();
        mobile_status_socket->setSocketOption(QAbstractSocket::LowDelayOption,1);
        //        connect_flag = true;
        //        connect(mobile_status_socket, SIGNAL(readyRead()), this, SLOT(on_read_file()));
        connect(mobile_status_socket, SIGNAL(readyRead()), this, SLOT(on_read_mobile_status()));
        connect(mobile_status_socket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    }

    while (tcpServer_map->hasPendingConnections())
    {
        //        qDebug()<<"tcp server map new connected!!!!!!!!!!!!";
        map_Socket = tcpServer_map->nextPendingConnection();
        map_Socket->setSocketOption(QAbstractSocket::LowDelayOption,1);
        //        connect_flag = true;
        //        connect(mobile_status_socket, SIGNAL(readyRead()), this, SLOT(on_read_file()));
        connect(map_Socket, SIGNAL(readyRead()), this, SLOT(on_map_read_command()));
        connect(map_Socket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    }
}

void mobile_robot::disConnection_tcp()
{
    connect_flag = false;

    clientSocket->deleteLater();
    mobile_status_socket->deleteLater();
    map_Socket->deleteLater();

    //disconnect 하기
    disconnect(clientSocket, SIGNAL(readyRead()), this, SLOT(readyRead_tcp()));
    disconnect(clientSocket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    disconnect(mobile_status_socket, SIGNAL(readyRead()), this, SLOT(on_read_mobile_status()));
    disconnect(mobile_status_socket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    disconnect(map_Socket, SIGNAL(readyRead()), this, SLOT(on_map_read_command()));
    disconnect(map_Socket, SIGNAL(disconnected()), this, SLOT(disConnection_tcp()));

    qDebug()<<"AMR tcp connection is dead";

    old_AMR_FSM_status = "STATE_AUTO_PATH_FINDING";
    move_flag = false;
    new_start = true;

}

void mobile_robot::on_map_read_command(){
    // 이미지 틀어지지 않도록 해더 추가함 "map"
    QByteArray _buf = map_Socket->readAll();
    cv::Mat new_img = cv::Mat::zeros(1000,1000,CV_8UC3);

    if(_buf.size() > 0)
    {
        new_buf.append(_buf);
        //        qDebug()<<new_buf;

        if(new_buf.size() < 2)
        {
            return;
        }
        bool is_header = false;
        for(int p = 0; p < new_buf.size()-1; p++)
        {
            // header check "map"
            if(new_buf[p] == (char)0x6d && new_buf[p+1] == (char)0x61 && new_buf[p+2] == (char)0x70)
            {
                new_buf.remove(0, p);
                is_header = true;
                break;
            }
        }

        const int packet_size = 1000*1000*3;
        if(is_header && new_buf.size() >= packet_size)
        {
            uchar* body = (uchar*)new_buf.data();
            //            qDebug()<<"body[0] : "<<body[0];
            //            qDebug()<<"body[1] : "<<body[1];
            //            qDebug()<<"body[2] : "<<body[2];
            memcpy(new_img.data,&body[3],1000*1000*3);
            map_img = new_img.clone();
            emit img_signal(map_img);

            new_buf.remove(0, packet_size);
        }
    }
}

void mobile_robot::init(MD_MOTOR *_md_mot,Cobot *_cobot)
{
    md_mot =_md_mot;
    cobot = _cobot;
}

void mobile_robot::liftState(QString lift_state)
{
    //    qDebug()<<"lift_state :" <<lift_state;
    _lift_State=lift_state;
}

void mobile_robot::cmdSendData(QByteArray cmd_msg)
{
    if(clientSocket != nullptr){
        if(clientSocket->isWritable()){
            clientSocket->write(cmd_msg);
        }
    }
}

//for robot arrived
void mobile_robot::sendData()
{
    if(clientSocket != nullptr)
    {
        if(clientSocket->isWritable())
        {
            QJsonObject json_output;
            json_output["MSG_TYPE"] = "STATE";
            //            qDebug()<<"cobot->systemStat.sdata.robot_state : "<<cobot->systemStat.sdata.robot_state;

            json_output["robot_state"] = cobot->systemStat.sdata.robot_state; //로봇 동작, 완료 등을 나타냄.

            MOTOR_MAIN_DATA md_status = md_mot->get_main_data();
            //            qDebug()<<"lift_pos:"<<md_status.motor_position;

            json_output["lift_rpm"] = md_status.motor_rpm;
            json_output["lift_state"] = _lift_State;
            json_output["lift_hight"] = md_status.motor_position*680/2700+1;

            QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);

            //            qDebug()<<"json_string"<<str_json;
            clientSocket->write(json_string);
        }
    }
}

void mobile_robot::getData(QString msg) //get file data from odroid
{
    if(mobile_status_socket != nullptr){
        if(mobile_status_socket->isWritable()){
            QJsonObject json_output;
            QByteArray json_string;
            get_somthing_msg = msg;

            if (msg == "map")
            {
                json_output["MSG_TYPE"] = "DOWNLOAD MAP";
                json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                qDebug()<<"json_string"<<json_string;
            }
            else if (msg == "ini")
            {
                json_output["MSG_TYPE"] = "DOWNLOAD INI";
                json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                qDebug()<<"json_string"<<json_string;
            }

            mobile_status_socket->write(json_string);
        }
    }
}

int test_flag = false;
int test_state = 0;
QString cmd_check;

char buf[4*1024]={0};
void mobile_robot::on_read_mobile_status() //get map data
{
    //     file socket
    QByteArray data = mobile_status_socket->readAll();
    QJsonObject json_input;
    QString end;
    bool test = false;

    QString jsonString = QString::fromUtf8(data);
    json_input = QJsonDocument::fromJson(jsonString.toUtf8()).object();

    // 데이터에서 각 JSON 객체를 분리
    QStringList jsonObjects = jsonString.split("}{");

    foreach (const QString& jsonString, jsonObjects)
    {
        QString trimmedJsonString = jsonString;

        // 맨 앞의 '{'가 빠진 경우, 다시 붙이기
        if (!trimmedJsonString.startsWith("{"))
        {
            trimmedJsonString.prepend("{");
        }

        // 맨 끝의 '}'가 빠진 경우, 다시 붙이기
        if (!trimmedJsonString.endsWith("}"))
        {
            trimmedJsonString.append("}");
        }

        // QString을 QByteArray로 변환하여 QJsonDocument로 파싱
        QByteArray jsonBytes = trimmedJsonString.toUtf8();
        QJsonParseError parseError;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonBytes, &parseError);

        if (parseError.error != QJsonParseError::NoError) {
            qDebug() << "Error parsing JSON:" << parseError.errorString();
            continue; // 다음 JSON 객체로 넘어감
        }

        // JSON 객체에서 원하는 데이터에 접근
        QJsonObject jsonObject = jsonDoc.object();
        QString msgType = jsonObject["MSG_TYPE"].toString();
        //        qDebug() << "MSG_TYPE:" << msgType;

        if (msgType == "MOBILE_POSE")
        {
            status = json_input["STATUS"].toInt();
            QString AMR_status;
            //            QString AMR_FSM_status;
            QString charge;

            if (status == 0)
            {
                AMR_status="UI_LOC_NOT_READY";
                //                emit mobile_run("false");
            }
            else if(status == 1)
            {
                AMR_status = "UI_LOC_BUSY";
                //                emit mobile_run("false");
            }
            else if(status == 2)
            {
                AMR_status="UI_LOC_GOOD";
                //                emit mobile_run("true");
            }
            else if(status == 3)
            {
                AMR_status = "UI_LOC_FAIL";
                //                emit mobile_run("true");
            }
            else if(status == 4)
            {
                AMR_status = "UI_LOC_MANUAL";
                //                emit mobile_run("false");
            }
            else if(status == 5)
            {
                AMR_status = "UI_LOC_GOOD_BUT_FAR_WAY";
                //                emit mobile_run("false");
            }
            //배터리 정보도 받아와야 함.
            pose_x = json_input["Pose_x"].toDouble();
            pose_y = json_input["Pose_y"].toDouble();
            pose_th = json_input["Pose_th"].toDouble();
            battery = json_input["battery"].toDouble();

            fsm_status = json_input["FSM STATUS"].toInt();



            if(fsm_status == 0)
            {
                AMR_FSM_status = "STATE_AUTO_PATH_FINDING";
                move_flag = true;
            }

            else if(fsm_status == 1)
            {
                AMR_FSM_status = "STATE_AUTO_FIRST_ALIGN";
                move_flag = true;
                emit mobile_run("true");
            }
            else if(fsm_status == 2)
            {
                AMR_FSM_status = "STATE_AUTO_PURE_PURSUIT";
                emit mobile_run("true");
                move_flag = true;
            }
            else if(fsm_status == 3)
            {
                AMR_FSM_status = "STATE_AUTO_FINAL_ALIGN";
                emit mobile_run("true");
                move_flag = true;
            }
            else if(fsm_status == 4)
            {
                AMR_FSM_status = "STATE_AUTO_GOAL_REACHED";
                emit mobile_run("false");
                //                qDebug()<<old_AMR_FSM_status;
                if(old_AMR_FSM_status == AMR_FSM_status)
                {
                    move_flag = false;
                }
                else
                {

                    uuid = json_input["uuid"].toString();
                    qDebug()<<"uuid : "<<uuid;
                    move_flag = true;
                }
            }
            else if(fsm_status == 5)
            {
                AMR_FSM_status = "STATE_AUTO_OBSTACLE";
                emit mobile_run("false");
                move_flag=true;
            }
            else if(fsm_status == 6)
            {
                AMR_FSM_status = "STATE_AUTO_PAUSE";
                emit mobile_run("true");
                move_flag=true;
            }
            else if(fsm_status == 7)
            {
                AMR_FSM_status = "STATE_AUTO_FAILED";
                emit mobile_run("true");
                move_flag=false;
            }

            //            old_AMR_FSM_status = AMR_FSM_status;
            //        charge_state = json_input["charge_state"].toInt;

            if (json_input["charge_state"].toInt()==0)
            {
                charge_state = false;
                charge = "false";                //                move_flag=false;
            }
            else
            {
                charge_state = true;
                charge = "true";
            }

            QString pose_msg;
            pose_msg.sprintf("x : %.2f, y : %.2f, z : %.1f, bat : %.1f, status : %s, charging status : %s, fsm status : %s", pose_x,pose_y,pose_th,battery,AMR_status.toLocal8Bit().data(),charge.toLocal8Bit().data(),AMR_FSM_status.toLocal8Bit().data());
            emit mobile_status(pose_msg);
        }
        else if (msgType == "MOBILE_MAP_CHANGE")
        {
            qDebug()<<"MOBLE_MAP_CHANGE";
            map_changed = true;
        }
        else if(msgType == "MAP_SEND_DONE")
        {
            png_change();
            bt_zip();
        }
        else
        {
            qDebug() << "Unknown MSG_TYPE:" << msgType;
        }
    }
    old_AMR_FSM_status = AMR_FSM_status;
}

void mobile_robot::png_change()
{
    cv::Mat raw_map;
    cv::Mat src;

    // load map
    QString path = "/home/rainbow/maps/";
    QDir dir(path);

    //file 있는지 없는지 검사
    foreach(QFileInfo item, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries)) //QDir::NoDotAndDotDot | QDir::AllEntries)
    {
        if(item.isDir())
        {
            QString filelist = item.absoluteFilePath();
            //만약 수정 맵이 있으면 수정맵 쓰고, 없으면 raw 맵 쓰기.
            QString read_map_path = filelist+"/map_edited.png";
            //            qDebug()<<read_map_path;

            QFileInfo raw_map_info(read_map_path);
            if(raw_map_info.exists() && raw_map_info.isFile())
            {
                raw_map = cv::imread(read_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
                qDebug()<<"[UNIMAP] map_edited.png loaded";
            }
            else
            {
                QString read_editied_map_path = filelist+"/map_raw.png";
                raw_map = cv::imread(read_editied_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
                qDebug()<<"[UNIMAP] map_raw.png loaded";
            }

            //            // load map meta
            QString map_meta_path = filelist + "/map_meta.ini";
            QFileInfo map_meta_info(map_meta_path);
            int map_w,map_h,map_ou,map_ov;
            if(map_meta_info.exists() && map_meta_info.isFile())
            {
                // load meta data file
                QSettings settings(map_meta_path, QSettings::IniFormat);
                map_w = settings.value("map_metadata/map_w", raw_map.cols).toInt();
                map_h = settings.value("map_metadata/map_h", raw_map.rows).toInt();
                map_ou = settings.value("map_metadata/map_origin_u", raw_map.cols/2).toInt();
                map_ov = settings.value("map_metadata/map_origin_v", raw_map.rows/2).toInt();
                //        int map_grid_width = settings.value("map_metadata/map_grid_width", robot_config.robot_grid_size).toDouble();

                //logger.write("[UNIMAP] map_meta.ini loaded", true);
            }
            else
            {
                //logger.write("[UNIMAP] map_meta.ini not found", true);
            }

            // raw map file saving
            for(int i = 0; i < raw_map.rows; i++)
            {
                for(int j = 0; j < raw_map.cols; j++)
                {
                    if(raw_map.ptr<uchar>(i)[j] == 0)
                    {
                        raw_map.ptr<uchar>(i)[j] = 127;
                    }
                    else if(raw_map.ptr<uchar>(i)[j] == 255)
                    {
                        raw_map.ptr<uchar>(i)[j] = 0;
                    }
                    else
                    {
                        raw_map.ptr<uchar>(i)[j] = 255;
                    }
                }
            }

            //    cv::imshow("map",raw_map);
            //    cv::Mat _map = raw_map.clone();
            QString item_name = filelist+"/changed_map.png";
            //            qDebug()<<item_name;
            //            if()

            //            cv::imwrite(item_name.toStdString(),raw_map);
            cv::flip(raw_map, src, 0); // X축 대칭
            cv::imwrite(item_name.toStdString(),src);
        }
    }
}

void mobile_robot::bt_zip()
//유진로봇 요청 대응하기 위한 자동 압축 프로그램 만들기...
{
    qDebug()<<"압축중입니다....";
    QString filelist;
    QString map_name;
    // load map
    QString path = "/home/rainbow/maps/";
    //    QString path = "/home/rainbow/RB_MOBILE/maps";
    QDir dir(path);

    //file 있는지 없는지 검사
    foreach(QFileInfo item, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries) ) //QDir::NoDotAndDotDot | QDir::AllEntries)
    {
        if(item.isDir())
        {
            filelist = item.absoluteFilePath();
            map_name = item.baseName();

            //zip in build directory
            QProcess::execute(QString("tar cf ")+map_name+QString(".tar.xz -C") +filelist+QString(" ."));
            //zip file move to /home/rainbow/maps
            QString aaa = QString("mv ")+map_name+QString(".tar.xz ")+ QString(" /home/rainbow/maps ");
            QProcess::execute(aaa);
            qDebug()<<aaa;
        }
    }
    qDebug()<<filelist;
}

void mobile_robot::readyRead_tcp()
{
    //    QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
    QByteArray data = clientSocket->readAll();
    //    qDebug()<<data;

    QJsonObject json_input;
    json_input = QJsonDocument::fromJson(data).object();

    // Motion Sequence ===============================================

    if(json_input["MSG_TYPE"] == "CMD")
    {
        QString cmd = json_input["lift_cmd"].toString();
        QString robot_cmd = json_input["robot_cmd"].toString();
        if(cmd == "lift high")
        {
            lift_hight = json_input["lift_hight"].toInt();
            qDebug()<<lift_hight;
            md_mot->move_position(lift_hight);
        }
        else if(cmd == "lift mid")
        {
            lift_hight = json_input["lift_hight"].toInt();
            qDebug()<<lift_hight;
            md_mot->move_position(lift_hight);
        }

        else if(cmd == "lift low")
        {
            printf("lift low");
            lift_hight = json_input["lift_hight"].toInt();
            qDebug()<<lift_hight;
            md_mot->move_position(lift_hight);
        }

        else if(robot_cmd == "robot init pos")
        {
            printf("robot init pos\n");
            //            cobot->MoveJoint(178.86,-15.61, -120.44, 46.04, -90.00, -88.89, 0.7, 0.4);

            cobot->MoveJoint(178.86,6.5,-114, 20, -90.00, -88.89, 2, -1);

            md_mot->move_position(50);
            md_mot->move_position(50);
            md_mot->move_position(50);
        }
        else if(robot_cmd == "robot pos1")//0x05
        {
            //            cobot->MoveJoint(47.93,-55.67, -89.85, -34.44, 137.95, -179.98, 2, -1);
            cobot->MoveJoint(-0.30,-6.29,-59,9.6,-90,-88, 1, -1);
            //        BTN_GRIPPER_ALLFOLD();
            printf("robot pos1\n");
        }
        else if(robot_cmd == "robot putdown1"){//0x06
            //            qDebug()<<"putttttttttttt dobbbbbbbbbb";
            cobot->MoveTCP(-600.0, -500.0, -25.0,-0.1,0,-90.0,2,-1);
        }
        else if(robot_cmd == "robot putdown2"){//0x07
            cobot->MoveTCP(-400.0, -500.0, -25.0,-0.1,0,-90.0,2,-1);
            //        Pinch12_FOLD();
        }
        else if(robot_cmd == "robot readyGripper2Pos"){//0x08
            cobot->MoveJoint(178.86,-15.61, -120.44, 46.04, -90.00, -88.89, 2, -1);
        }
        else if(robot_cmd == "robot grapsGripper2Pos"){//0x09
            cobot->MoveTCP(500.0, 100.0, -58.68,90.0,0,0.0,2,-1);
        }

        else if(robot_cmd == "robot vision"){//0x10
            //            cobot->MoveJoint(11.717,16.514,-125.54,89.9167,-108.462,-88.889,2, -1);

            qDebug()<<"robot vision";
            cobot->MoveJoint(-0.30,-6.29,-59 ,9.6,-90,-88, 1, -1);
            //        cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        }
        else if(robot_cmd == "robot pump"){//0x10
            //            cobot->MoveJoint(6, -27, -81, 109,-108,-88,2, -1);
            cobot->MoveJoint(-645-20, -146,339-20,88.59,-34.29,-177.87, 0.5, -1);
            cobot->ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        }
        else if(robot_cmd == "robot pump off"){
            cobot->ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        }
        else if(robot_cmd == "robot pick"){//0x10
            cobot->MoveTCP(-665.11,-174,339,88.59,-34.29,-177.87, 0.5, -1);
        }
        else if(robot_cmd == "robot out"){//0x11
            //            cobot->MoveJoint(178.86,-15.61, -120.44, 46.04, -90.00, -88.89, 2, -1);

            cobot->MoveTCP(-665-20+200,-174, 339-20+200,88.59,-34.29,-177.87,  0.1, 0.1);

        }
        else if(robot_cmd == "robot origin"){//0x11
            //            cobot->MoveJoint(178.86,-15.61, -120.44, 46.04, -90.00, -88.89, 2, -1);

            cobot->MoveTCP(-665-20+200,-174, 339-20+200,88.59,-34.29,-177.87,  0.1, 0.1);

        }
        else if(robot_cmd == "robot mid finish"){//0x12
            //            cobot->MoveJoint(-27.42,19.57,-91.84,69.98,-62.28,-80.16, 2, -1);
            cobot->MoveJoint(178.86,6.5,-114,20, -90.00, -88.89, 2, -1);
        }
        else if(robot_cmd == "robot 13"){//0x13
            cobot->MoveTCP(-581.79,-116.26,797.34,13.1,-81.0,-102.6, 2, -1);
        }
        else if(robot_cmd == "robot 14"){//0x14
            cobot->MoveTCP(-229.27,6.68,642.14,-7.1,-79.3,-82.7,2, -1);
        }
        else if(robot_cmd == "open gripper"){
            //        Pinch12_FOLD();
        }
        else if(robot_cmd == "close gripper"){
            //        BTN_GRIPPER_ALLFOLD();
        }
    }
    else if(json_input["MSG_TYPE"] == "Global_PATH")
    {
        QJsonArray json_arr;
        QJsonObject json_output;
        QJsonObject json_global;
        YUJIN_PATH global_path;

        QJsonArray arr_path_x = json_input["path_x"].toArray();
        QJsonArray arr_path_y = json_input["path_y"].toArray();

        for(int p=0;p<arr_path_x.size();p++)
        {
            global_path.path.push_back(cv::Vec2d(arr_path_x[p].toDouble(),arr_path_y[p].toDouble()));

            json_output["x"] = arr_path_x[p].toDouble();
            json_output["y"] = arr_path_y[p].toDouble();
            //            json_arr.insert(0,json_output); //append 로 해보기
            json_arr.append(json_output);
        }
        json_global["global"] = json_arr;
        json["path_plan"] = json_global;
        QString path_info = QJsonDocument(json).toJson(QJsonDocument::Compact);//보낸 내용 확인용
        //        qDebug() << path_info;
    }
}



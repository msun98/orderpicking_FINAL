#include "integrate_ui.h"
#include <QJsonDocument>
#include <QJsonObject>
INTEGRATE_UI::INTEGRATE_UI(QObject *parent):QObject(parent)
{
    //클라이언트(서버의 주소와 포트를 알고 있어야 함.)
    ui_com = new QTcpSocket();
    mobile_status_socket = new QTcpSocket();
    map_socket = new QTcpSocket();


    //    mobile_status_socket -> connectToHost(QHostAddress(IP),PORT2);
    connect(ui_com,SIGNAL(connected()),this, SLOT(onUIConnected()));
    connect(ui_com,SIGNAL(disconn1ected()),this, SLOT(onUIdisConnected()));

    connect(mobile_status_socket,SIGNAL(connected()),this, SLOT(onMobileStatusConnected()));
    connect(mobile_status_socket,SIGNAL(disconnected()),this, SLOT(onMobileStatusDisConnected()));
    //    connect(mobile_status_socket,SIGNAL(disconnected()),this, SLOT(onUIdisConnected()));

    connect(map_socket,SIGNAL(connected()),this, SLOT(onMapIMGConnected()));
    connect(map_socket,SIGNAL(disconnected()),this, SLOT(onMapIMGDISConnected()));


    QJsonObject json_output;
    json_output["MSG_TYPE"] = "MOBILE_STATUS";
    json_output["STATUS"] = mobile_status;
    //            //        json_output["buf length"]=len;//vector 로 담아서 보내기
    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
    //    onMobileStatusSocketWrite(json_string);
    //    qDebug() << mobile_status;

    reconnectTimer.start(500);
    connect(&reconnectTimer,SIGNAL(timeout()),this, SLOT(onReconnectToClients()));

    path_timer.start(100);
    connect(&path_timer,SIGNAL(timeout()),this, SLOT(publish_path()));
}

void INTEGRATE_UI::onUIConnected()
{
    qDebug("connecting........");
    ui_com -> connectToHost(QHostAddress(IP),PORT1);
    ui_com -> setSocketOption(QAbstractSocket::LowDelayOption,1);

    connect(ui_com,SIGNAL(readyRead()),this, SLOT(onReadyCmdRead()));
    connect(ui_com,SIGNAL(disconnected()),this, SLOT(onUIdisConnected()));

    if(ui_com ->state() == QAbstractSocket::ConnectedState)
    {
        qDebug("cmd 소켓은 성공이이야");
        //        start_flag =true;
        reconnectTimer.stop();
    }
    connect(&mobile_status_timer,SIGNAL(timeout()),this, SLOT(sendUIStatus()));
    mobile_status_timer.start(100);

    //    onMapConnected();
}

void INTEGRATE_UI::onMobileStatusConnected()
{
    qDebug("map connecting........");
    mobile_status_socket -> connectToHost(QHostAddress(IP),PORT2);
    mobile_status_socket -> setSocketOption(QAbstractSocket::LowDelayOption,1);

    //    connect(ui_com,SIGNAL(readyRead()),this, SLOT(onReadyCmdRead()));
    connect(mobile_status_socket,SIGNAL(readyRead()),this, SLOT(onMobileStatusCmdRead()));

    if(mobile_status_socket ->state() == QAbstractSocket::ConnectedState)
    {
        qDebug("mobile_status_socket 소켓은 성공이이야");
    }
}

void INTEGRATE_UI::onMapIMGConnected()
{
    qDebug("map img connecting........");
    map_socket -> connectToHost(QHostAddress(IP),MAP_PORT);
    map_socket -> setSocketOption(QAbstractSocket::LowDelayOption,1);

    //    connect(ui_com,SIGNAL(readyRead()),this, SLOT(onReadyCmdRead()));
    //    connect(map_socket,SIGNAL(readyRead()),this, SLOT(onMapIMGCmdRead()));

    if(map_socket ->state() == QAbstractSocket::ConnectedState)
    {
        qDebug("map_socket 소켓은 성공이이야");
    }

}

void INTEGRATE_UI::onReconnectToClients()
{
    qDebug()<<"reconnecting to clients";
    onUIConnected();
    onMobileStatusConnected();
    onMapIMGConnected();
}

void INTEGRATE_UI::init(MOBILE *_mobile, AUTOCONTROL *_ctrl, SLAM_2D *_slam)
{
    mobile = _mobile;
    ctrl = _ctrl;
    slam = _slam;

    //    QJsonObject json_output;

    //    json_output["MSG_TYPE"]="MOBILE_STATUS";
    //    json_output["STATUS"]=mobile_status;
    //    //            //        json_output["buf length"]=len;//vector 로 담아서 보내기
    //    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
    //    onMobileStatusSocketWrite(json_string);
    //    qDebug()<<mobile_status;

}

void INTEGRATE_UI::onUIdisConnected()
{
    onSocketWrite("tcp disconnected");
    qDebug()<<"tcp disconnected";
    ui_com ->close();
}

void INTEGRATE_UI::onMobileStatusDisConnected()
{
    mobile_status_socket ->close();

}

void INTEGRATE_UI::onMapIMGDISConnected()
{
    map_socket ->close();
    reconnectTimer.start();
}

void INTEGRATE_UI::sendUIStatus()
{
    if(send_mobile_status.size()!=0)
    {
        QByteArray json_string = send_mobile_status.front();
        onMobileStatusSocketWrite(json_string);
        mtx.lock();
        send_mobile_status.pop();
        mtx.unlock();
    }
    if(send_mobile_status.size()>50)
    {
        send_mobile_status.pop();
    }
    //    old_status = status;
    //    old_status_tick = status_tick;
}


void INTEGRATE_UI::publish_path()
{
    //    qDebug()<<mobile_moving_status;
    if(mobile_moving_status == 2){
        // get path
        std::vector<PATH_POINT> path = ctrl->get_cur_path();
        if(path.size() == 0)
        {
            return;
        }

        cv::Vec3d cur_pose = slam->get_cur_pose();
        cv::Vec2d cur_pos = cv::Vec2d(cur_pose[0], cur_pose[1]);
        int cur_idx = ctrl->get_cur_idx(cur_pos, path);
        //        qDebug()<<"cur_idx :"<<cur_idx;

        QJsonArray arr_path_x;
        QJsonArray arr_path_y;
        //    QJsonArray arr_path_th;

        // path sampling, 0.5m
        std::vector<PATH_POINT> sampled_path;
        sampled_path.push_back(path[cur_idx]);
        for(size_t p = cur_idx; p < path.size()-1; p+=50)
        {
            sampled_path.push_back(path[p]);
        }
        sampled_path.push_back(path.back());

        int num = sampled_path.size();
        for(int p = 0; p < num; p++)
            //        for(int p = num; p > 0; p--)
        {
            arr_path_x.append(sampled_path[p].pt[0]);
            arr_path_y.append(sampled_path[p].pt[1]);
            //            qDebug()<<p;
        }

        QJsonObject json;
        json["MSG_TYPE"] = "Global_PATH";
        json["path_x"] = arr_path_x;
        json["path_y"] = arr_path_y;


        QByteArray json_string = QJsonDocument(json).toJson(QJsonDocument::Compact);
        //        qDebug()<<QString(json_string);
        onSocketWrite(QString(json_string));
    }
}



//void INTEGRATE_UI::onMapIMGCmdRead() //map data 줄 때 사용하는 함수.
//{
//    QByteArray Read_Data = map_socket -> readAll();
//    qDebug()<<Read_Data;
//    QJsonObject json_input;4
//}

void INTEGRATE_UI::onMobileStatusCmdRead() //map data 달라고 할 때 사용하는 함수.
{
    QByteArray Read_Data = mobile_status_socket -> readAll();
    qDebug()<<Read_Data;

    QJsonObject json_input;
    //    QJsonObject json_output;
    json_input = QJsonDocument::fromJson(Read_Data).object();

    QString ip = setting_config.fms_ip;
    QString id = setting_config.fms_id;
    QString pw = setting_config.fms_pw;

    if(json_input["MSG_TYPE"] == "DOWNLOAD INI"){
        QString ini_path = "/home/rainbow/RB_MOBILE/config";

        QString destination_path = "/home/" + setting_config.fms_id + "/RB_MOBILE";
        qDebug()<<"destination_path : "<<destination_path;

        QString cmd = "sshpass -p " + pw + " rsync -avz -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' " + ini_path + " " + id + "@" + ip + ":" + destination_path;

        qDebug()<<cmd;

        int result = system(cmd.toStdString().c_str());

        std::cout<<"file transfer result : "<<result<<std::endl;
        if(result != 0)
        {
            logger.write("[SERVER] rsync command failed.", true);
        }
        else
        {
            QJsonObject json_output;
            json_output["MSG_TYPE"] = "MAP_SEND_DONE";

            QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
            //                        onMobileStatusSocketWrite(json_string);

            mtx.lock();
            send_mobile_status.push(json_string);
            mtx.unlock();

            std::cout<<"file transfer done!"<<std::endl;
            logger.write("[SERVER] successfully transferred using rsync..", true);
        }

        std::system(cmd.toLocal8Bit().data());

        QString str;
        str.sprintf("[FMS] map config send, server: %s, map config path: %s", setting_config.fms_ip.toLocal8Bit().data(), setting_config.map_path.toLocal8Bit().data());
        logger.write(str, true);
    }

    else if(json_input["MSG_TYPE"] == "DOWNLOAD MAP"){
        QString maps_path = "/home/rainbow/RB_MOBILE/maps";
        QString destination_path = "/home/" + setting_config.fms_id + "/RB_MOBILE";

        QString cmd = "sshpass -p " + pw + " rsync -avz -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' " + maps_path + " " + id + "@" + ip + ":" + destination_path;
        qDebug()<<cmd;

        int result = system(cmd.toStdString().c_str());

        std::cout<<"file transfer result : "<<result<<std::endl;
        if(result != 0)
        {
            logger.write("[SERVER] rsync command failed.", true);
        }
        else
        {
            QJsonObject json_output;
            json_output["MSG_TYPE"] = "MAP_SEND_DONE";

            QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
            //            onMobileStatusSocketWrite(json_string);

            std::cout<<"file transfer done!"<<std::endl;
            logger.write("[SERVER] successfully transferred using rsync..", true);
        }

        std::system(cmd.toLocal8Bit().data());

        QString str;
        str.sprintf("[FMS] map send, server: %s, map_path: %s", setting_config.fms_ip.toLocal8Bit().data(), setting_config.map_path.toLocal8Bit().data());
        logger.write(str, true);
    }
}

void INTEGRATE_UI::on_read_command(QString head){

}


void INTEGRATE_UI::send_file(){
}

void INTEGRATE_UI::send_command(int code,QString str){
}

void INTEGRATE_UI::onReadyCmdRead() //nuc 에게 로봇 상태를 시간에 맞추어 알려줌.
{
    QByteArray Read_Data = ui_com -> readAll();
    //    qDebug()<<"readAll :"<<Read_Data;
    QJsonObject json_input;
    //    QJsonObject json_output;
    json_input = QJsonDocument::fromJson(Read_Data).object();

    if(json_input["MSG_TYPE"] == "STATE") //nuc로 부터 리프트, 로봇의 정보를 받음.
    {
        robot_state = json_input["robot_state"].toInt();
        lift_rpm = json_input["lift_rpm"].toInt();
        lift_state = json_input["lift_state"].toString();
        lift_pos = json_input["lift_pose"].toInt();
    }

    //유진로봇과 통신하는 부분 (잘 작동하니 건들지 말것...)
    else if(json_input["MSG_TYPE"] == "MOVE"){
        double move_x = json_input["POSE_x"].toDouble();
        double move_y = json_input["POSE_y"].toDouble();
        double move_th = json_input["POSE_theta"].toDouble();

        //yujin robot get preset_idx

        int preset_idx = PRESET_SPEED_NORMAL;
        ctrl->run_pick(cv::Vec3d(move_x,move_y,move_th), preset_idx);

        qDebug()<<move_x<<move_y<<move_th;
    }

    else if(json_input["MSG_TYPE"] == "MOVE_EXT"){

        QJsonArray move_path = json_input["PATH"].toArray();
        QJsonObject pose;
        cv::Vec3d path;
        std::vector<cv::Vec3d> waypoints;

        for(int p = 0; p < move_path.size(); p++)
        {
            QJsonObject move_path_obj = move_path[p].toObject();
            pose = move_path_obj["pose"].toObject();
            path[0] = pose["x"].toDouble();
            path[1] = pose["y"].toDouble();
            path[2] = pose["theta"].toDouble();

            //            qDebug()<<path[0];
            //            command_path_que.push(path);
            waypoints.push_back(cv::Vec3d(path[0],path[1],path[2]));
        }
        //        zcommand_path_que.tryz_pop(1);z

        //yujin robot get preset_idx

        int preset_idx = PRESET_SPEED_NORMAL;
        ctrl->run_ext(waypoints,preset_idx);

        //        qDebug()<<move_x<<move_y<<move_th;
    }

    else if(json_input["MSG_TYPE"] == "DOCK"){
        QString marker_id = json_input["marker_id"].toString();
        QString direction = json_input["direction"].toString();

    }
    else if(json_input["MSG_TYPE"] == "PAUSE"){
        //        ctrl->is_pause = true;
        if(ctrl->fsm_state != STATE_AUTO_GOAL_REACHED &&
                ctrl->fsm_state != STATE_AUTO_FAILED)
        {
            ctrl->fsm_state = STATE_AUTO_PAUSE;
        }
    }
    else if(json_input["MSG_TYPE"] == "RESUME"){
        //        ctrl->is_pause = false;
        if(ctrl->fsm_state == STATE_AUTO_PAUSE)
        {
            ctrl->fsm_state = STATE_AUTO_PATH_FINDING;
        }
    }
    else if(json_input["MSG_TYPE"] == "STOP"){
        ctrl->stop();
        //        if(ctrl->ofsm_state == STATE_WAITING)
        //        {
        //            ctrl->ofsm_state = STATE_IDLE;
        //        }
    }

    else if(json_input["MSG_TYPE"]=="CHANGE MAP"){
        change_map_name = json_input["map_name"].toString();
        qDebug()<<"changed map : "<< change_map_name;
        emit change_map(change_map_name);
    }

    else if(json_input["MSG_TYPE"] == "INIT")
    {
        mobile->motor_init();

        qDebug()<<"motor_init";
    }

    //    qDebug()<<Read_Data;w
}

void INTEGRATE_UI::set_file(QString filePath){
}

void INTEGRATE_UI::onSocketWrite(QString msg)
{
    ui_com->write(msg.toUtf8());
}

void INTEGRATE_UI::onMobileStatusSocketWrite(QString socketmsg)
{
    mobile_status_socket->write(socketmsg.toUtf8());
}

void INTEGRATE_UI::onMapImageSocketWrite(QByteArray map_msg)
{
    map_socket->write(map_msg);
}

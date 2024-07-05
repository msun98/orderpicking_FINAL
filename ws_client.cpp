#include "ws_client.h"

WS_CLIENT::WS_CLIENT(QObject *parent)
    : QObject(parent)
    , reconnect_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &WS_CLIENT::connected);
    connect(&client, &QWebSocket::disconnected, this, &WS_CLIENT::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

WS_CLIENT::~WS_CLIENT()
{
    client.close();
}

void WS_CLIENT::init()
{
    #ifdef USE_SIM

    client.open(QUrl("ws://127.0.0.1:12334"));
    reconnect_timer.start(1000);

    #else

    QString str;
    str.sprintf("ws://%s:12334", setting_config.fms_ip.toLocal8Bit().data());
    client.open(QUrl(str));
    reconnect_timer.start(1000);

    #endif
}

void WS_CLIENT::connected()
{
    connect(&client, &QWebSocket::textMessageReceived, this, &WS_CLIENT::recv_message);
    is_connected = true;
    is_print = true;

    logger.write("[WS] server connected", true);
}

void WS_CLIENT::disconnected()
{
    is_connected = false;

    // clear parameters
    id = "";
    disconnect(&client, &QWebSocket::textMessageReceived, this, &WS_CLIENT::recv_message);

    if(is_print == true)
    {
        is_print = false;
        logger.write("[WS] server disconnected", true);
    }
}

void WS_CLIENT::reconnect_loop()
{
    if(client.state() == QAbstractSocket::ConnectedState)
    {

    }
    else if(client.state() == QAbstractSocket::ConnectingState)
    {

    }
    else
    {
        if(is_connected == false)
        {
            #ifdef USE_SIM
            client.open(QUrl("ws://127.0.0.1:12334"));
            #else
            QString str;
            str.sprintf("ws://%s:12334", setting_config.fms_ip.toLocal8Bit().data());
            client.open(QUrl(str));
            #endif
        }
    }
}

void WS_CLIENT::recv_message(QString message)
{
    QJsonObject json = QJsonDocument::fromJson(message.toUtf8()).object();

    // server to client, set id
    if(id == "" && json["type"].toString() == "STC_ID")
    {
        id = json["id"].toString();

        emit signal_stc_id(id);
        printf("[WS] server to client, set id:%s\n", id.toLocal8Bit().data());
    }

    // server to client, set path
    if(id == json["id"].toString() && json["type"].toString() == "STC_PATH")
    {
        // path parsing
        QJsonArray arr;
        arr = json["path"].toArray();

        std::vector<cv::Vec2d> path;
        for(int p = 0; p < arr.size(); p++)
        {
            QString str = arr[p].toString();
            QStringList str_split = str.split(",");

            double x = str_split[0].toDouble();
            double y = str_split[1].toDouble();

            path.push_back(cv::Vec2d(x,y));
        }

        emit signal_stc_path(path);
        printf("[WS] server to client, path:%d\n", (int)path.size());
    }

    // server to client, tick
    if(id == json["id"].toString() && json["type"].toString() == "STC_TICK")
    {
        fms_tick = json["tick"].toInt();
    }

    // server to client, stop
    if(id == json["id"].toString() && json["type"].toString() == "STC_STOP")
    {
        emit signal_stc_stop();
        printf("[WS] server to client, stop, %f\n", get_time());
    }

    // server to client, avoid enable
    if(id == json["id"].toString() && json["type"].toString() == "STC_ALLOW")
    {
        int allow = json["allow"].toInt();
        emit signal_stc_allow(allow);
        printf("[WS] server to client, path allow:%d, %f\n", allow, get_time());
    }
}

void WS_CLIENT::cts_status(CTS_STATUS status)
{
    QJsonObject json;
    json["type"] = "CTS_STATUS";
    json["id"] = status.id;
    json["state"] = status.state;

    QString pose_str;
    pose_str.sprintf("%.6f,%.6f,%.6f", status.pose[0], status.pose[1], status.pose[2]);
    json["pose"] = pose_str;

    QString goal_str;
    goal_str.sprintf("%.6f,%.6f,%.6f", status.goal[0], status.goal[1], status.goal[2]);
    json["goal"] = goal_str;

    QJsonArray arr;
    for(size_t p = 0; p < status.path.size(); p++)
    {
        QString str;
        str.sprintf("%.6f,%.6f", status.path[p][0], status.path[p][1]);
        arr.append(str);
    }
    json["path"] = arr;

    QJsonArray arr2;
    for(size_t p = 0; p < status.obs.size(); p++)
    {
        QString str;
        str.sprintf("%.6f,%.6f", status.obs[p][0], status.obs[p][1]);
        arr2.append(str);
    }
    json["obs"] = arr2;

    json["tick"] = status.tick;

    QJsonDocument doc(json);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

void WS_CLIENT::cts_confirm(std::vector<cv::Vec2d> path)
{
    QJsonObject json;
    json["type"] = "CTS_CONFIRM";
    json["id"] = id;

    QJsonArray arr;
    for(size_t p = 0; p < path.size(); p++)
    {
        QString str;
        str.sprintf("%.6f,%.6f", path[p][0], path[p][1]);
        arr.append(str);
    }
    json["path"] = arr;

    QJsonDocument doc(json);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

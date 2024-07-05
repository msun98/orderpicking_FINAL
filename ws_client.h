#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include "global_defines.h"

#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class WS_CLIENT : public QObject
{
    Q_OBJECT
public:
    explicit WS_CLIENT(QObject *parent = nullptr);
    ~WS_CLIENT();

    QString id = "";
    QWebSocket client;
    QTimer reconnect_timer;

    std::atomic<bool> is_print = {false};
    std::atomic<bool> is_connected = {false};
    std::atomic<int> fms_tick = {0};

    void init();

signals:
    void signal_stc_id(QString id);
    void signal_stc_stop();
    void signal_stc_path(std::vector<cv::Vec2d> path);
    void signal_stc_allow(int allow);

private slots:
    void connected();
    void disconnected();
    void reconnect_loop();
    void recv_message(QString message);

    void cts_status(CTS_STATUS status);
    void cts_confirm(std::vector<cv::Vec2d> path);
};

#endif // WS_CLIENT_H

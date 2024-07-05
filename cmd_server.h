#ifndef CMD_SERVER_H
#define CMD_SERVER_H

#include "global_defines.h"

#include <QObject>
#include <QWebSocket>
#include <QWebSocketServer>

class CMD_SERVER : public QObject
{
    Q_OBJECT
public:
    explicit CMD_SERVER(QObject *parent = nullptr);
    void init();

    std::mutex mtx;
    QWebSocketServer server;

signals:    
    void process_command_signal(CMD cur_cmd);

public slots:    
    void client_connected();
    void client_disconnected();
    void recv_message(QByteArray message);
};

#endif // CMD_SERVER_H

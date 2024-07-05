#include "cmd_server.h"

CMD_SERVER::CMD_SERVER(QObject *parent)
    : QObject(parent)
    , server("fms_server", QWebSocketServer::NonSecureMode, this)
{
}

void CMD_SERVER::init()
{
    if(server.listen(QHostAddress::LocalHost, 12335))
    {
        connect(&server, SIGNAL(newConnection()), this, SLOT(client_connected()));        
    }
}

void CMD_SERVER::client_connected()
{
    QWebSocket *socket = server.nextPendingConnection();
    connect(socket, &QWebSocket::binaryMessageReceived, this, &CMD_SERVER::recv_message);
    connect(socket, &QWebSocket::disconnected, this, &CMD_SERVER::client_disconnected);

    logger.write("[CMD_SERVER] client(UI) connected", true);
}

void CMD_SERVER::client_disconnected()
{
    QWebSocket *socket = qobject_cast<QWebSocket*>(sender());
    socket->deleteLater();

    logger.write("[CMD_SERVER] client(UI) disconnected", true);
}

void CMD_SERVER::recv_message(QByteArray message)
{
    CMD _cur_cmd;
    memcpy(&_cur_cmd, message.data(), sizeof(CMD));

    emit process_command_signal(_cur_cmd);
}

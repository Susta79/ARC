#include "mytcpsocket.h"
#include "error_def.h"

#include <Eigen/Dense>
#include <iostream>
#include <string.h>
#include <stdio.h>

using namespace Eigen;
using Eigen::MatrixXd;

MyTcpSocket::MyTcpSocket(QObject *parent) :
    QObject(parent)
{
}

ARCCode_t MyTcpSocket::doConnect(Array<double, 6, 1> joint_send, Array<double, 6, 1>& joint_rec)
{
    char str[64];
    sprintf_s(str, "%0007.3f %0007.3f %0007.3f %0007.3f %0007.3f %0007.3f\n", joint_send(0), joint_send(1), joint_send(2), joint_send(3), joint_send(4), joint_send(5));
    QTextStream(stdout) << str;

    socket = new QTcpSocket(this);

    socket->connectToHost("127.0.0.1", 1755);

    if(socket->waitForConnected(5000))
    {
        qDebug() << "Connected!";

        // send
        socket->write(str);
        socket->waitForBytesWritten(1000);
        socket->waitForReadyRead(3000);
        
        qDebug() << "Reading: " << socket->bytesAvailable();

        // get the data
        QByteArray reply = socket->readAll();
        qDebug() << reply;
        char *s = reply.data();
        
        char seps[] = " ";
        char *token = NULL;
        char *next_token = NULL;

        // Establish string and get the first token:
        token = strtok_s(s, seps, &next_token);

        // While there are tokens in "string"
        int i=0;
        while (token != NULL)
        {
            // Get next token:
            joint_rec(i) = std::stod(token);
            token = strtok_s(NULL, seps, &next_token);
            i++;
        }

        // close the connection
        socket->close();
    }
    else
    {
        qDebug() << "Not connected!";
    }

    return ARC_CODE_OK;
}
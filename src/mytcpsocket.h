#ifndef MYTCPSOCKET_H
#define MYTCPSOCKET_H

#include "error_def.h"

#include <QObject>
#include <QTcpSocket>
#include <QDebug>
#include <Eigen/Dense>

using namespace Eigen;
//using Eigen::MatrixXd;

class MyTcpSocket : public QObject
{
    Q_OBJECT
public:
    explicit MyTcpSocket(QObject *parent = 0);
    
    ARCCode_t MyTcpSocket::doConnect(Array<double, 6, 1> joint_send, Array<double, 6, 1>& joint_rec);

signals:
    
public slots:

private:
    QTcpSocket *socket;
    
};

#endif // MYTCPSOCKET_H
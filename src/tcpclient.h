#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include "error_def.h"
#include "joint.h"

#include <QObject>
#include <QTcpSocket>
#include <QDebug>
#include <QGroupBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <Eigen/Dense>

using namespace Eigen;

class TcpClient : public QObject {
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = 0);
    ~TcpClient();
    QGroupBox *gbGroup;
    Joint* pCmdJoint;
    Joint* pActJoint;

signals:

private slots:
    void socket_stateChanged();
    void pbConnDisc_released();
    void pbSync_released();
    void tmr_timeout();

private:
    QTcpSocket *socket;
    QLineEdit *leIP;
    QSpinBox *sbPort;
    QPushButton *pbConnDisc;
    QPushButton *pbSync;
    QTimer* tmr;
    ARCCode_t Connect(QString IP, int Port);
    ARCCode_t Disconnect();
};

#endif // TCPCLIENT_H
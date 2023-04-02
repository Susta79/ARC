#include "tcpclient.h"
#include "error_def.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QRegExpValidator>
#include <QTimer>
#include <Eigen/Dense>
#include <iostream>
#include <string.h>
#include <stdio.h>

using namespace Eigen;
using Eigen::MatrixXd;

TcpClient::TcpClient(QObject *parent) : QObject(parent) {
    // QTcpSocket
    this->socket = new QTcpSocket(this);
    connect(socket, &QTcpSocket::stateChanged, this, &TcpClient::socket_stateChanged);

    // QPushButton pbConnDisc
    this->pbConnDisc = new QPushButton("Connect");
    connect(pbConnDisc, &QPushButton::released, this, &TcpClient::pbConnDisc_released);

    // QPushButton pbSync
    this->pbSync = new QPushButton("Start");
    this->pbSync->setEnabled(false);
    connect(pbSync, &QPushButton::released, this, &TcpClient::pbSync_released);

    // QLineEdit leIP
    this->leIP = new QLineEdit();
    QString ipRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    QRegExp ipRegex ("^" + ipRange
                    + "\\." + ipRange
                    + "\\." + ipRange
                    + "\\." + ipRange + "$");
    QRegExpValidator *ipValidator = new QRegExpValidator(ipRegex, this);
    this->leIP->setValidator(ipValidator);
    this->leIP->setText("127.0.0.1");

    // QSpinBox sbPort
    this->sbPort = new QSpinBox();
    this->sbPort->setRange(1024, 65535);
    this->sbPort->setValue(1755);

    // QTimer tmr
    tmr = new QTimer(this);
    connect(tmr, &QTimer::timeout, this, &TcpClient::tmr_timeout);

    this->pCmdJoint = new Joint("Cmd");
    this->pActJoint = new Joint("Act");

    // Group Main
    this->gbGroup = new QGroupBox("TcpClient");
    QHBoxLayout *layoutGroup = new QHBoxLayout();
    layoutGroup->addWidget(this->pCmdJoint->gbJoints);
    layoutGroup->addWidget(this->pActJoint->gbJoints);
    QVBoxLayout *vBoxLayout = new QVBoxLayout();
    vBoxLayout->addWidget(this->leIP);
    vBoxLayout->addWidget(this->sbPort);
    vBoxLayout->addWidget(this->pbConnDisc);
    vBoxLayout->addWidget(this->pbSync);
    QWidget *windowMS = new QWidget();
    windowMS->setLayout(vBoxLayout);
    layoutGroup->addWidget(windowMS);
    gbGroup->setLayout(layoutGroup);
}

TcpClient::~TcpClient() {
    this->socket->close();
}

ARCCode_t TcpClient::Connect(QString IP, int Port) {
    this->socket->connectToHost(IP, Port);
    if(socket->waitForConnected(5000))
    {
        qDebug() << "Connected!";
        return ARC_CODE_OK;
    }
    else
    {
        qDebug() << "Not connected!";
        // Todo: adjust the error code
        return ARC_ERR_COMM_TIMEOUT;
    }
}

ARCCode_t TcpClient::Disconnect() {
    if (this->socket->state() != QAbstractSocket::UnconnectedState)
        this->socket->disconnectFromHost();
    return ARC_CODE_OK;
}

void TcpClient::socket_stateChanged() {
    switch (this->socket->state())
    {
    case QAbstractSocket::UnconnectedState:
        this->pbConnDisc->setText("Unconnected");
        break;
    case QAbstractSocket::HostLookupState:
        this->pbConnDisc->setText("HostLookup");
        break;
    case QAbstractSocket::ConnectingState:
        this->pbConnDisc->setText("Connecting");
        break;
    case QAbstractSocket::ConnectedState:
        this->pbConnDisc->setText("Connected");
        break;
    case QAbstractSocket::BoundState:
        this->pbConnDisc->setText("Bound");
        break;
    case QAbstractSocket::ClosingState:
        this->pbConnDisc->setText("Closing");
        break;
    case QAbstractSocket::ListeningState:
        this->pbConnDisc->setText("Listening");
        break;
    default:
        this->pbConnDisc->setText("Unknow");
        break;
    }
}

void TcpClient::pbConnDisc_released() {
    if (this->socket->state() == QAbstractSocket::UnconnectedState) {
        if (this->Connect(this->leIP->text(), this->sbPort->value()) == ARC_CODE_OK) {
            this->pbSync->setEnabled(true);
            this->pbSync->setText("Start");
        } else {
            this->pbSync->setEnabled(false);
            this->pbSync->setText("Start");
        }
    } else {
        this->Disconnect();
        this->pbSync->setEnabled(false);
        this->pbSync->setText("Start");
    }
}

void TcpClient::pbSync_released() {
    if (this->socket->state() == QAbstractSocket::ConnectedState) {
        if (this->tmr->isActive() == false) {
            this->tmr->start(100);
            this->pbSync->setEnabled(true);
            this->pbSync->setText("Stop");
        } else {
            this->tmr->stop();
            this->pbSync->setEnabled(true);
            this->pbSync->setText("Start");
        }
    } else {
            this->tmr->stop();
            this->pbSync->setEnabled(false);
            this->pbSync->setText("Start");
    }
}

void TcpClient::tmr_timeout() {
    Array<double, 6, 1> joint_send = this->pCmdJoint->get_joints_deg();
    Array<double, 6, 1> joint_rec;

    char str[64];
    sprintf_s(str, "%0007.3f %0007.3f %0007.3f %0007.3f %0007.3f %0007.3f\n", joint_send(0), joint_send(1), joint_send(2), joint_send(3), joint_send(4), joint_send(5));
    QTextStream(stdout) << str;

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

    this->pActJoint->set_joints_deg(joint_rec);

    return;
}
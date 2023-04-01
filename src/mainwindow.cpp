#include "mainwindow.h"
#include "joint.h"
#include "mytcpsocket.h"

#include <QMainWindow>
#include <QWidget>
#include <QFormLayout>
#include <QHBoxLayout>

#include <Eigen/Dense>

using namespace Eigen;

#define VERSION "1.1.0"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {

   this->pRobot = new Robot("Robot1");
   this->pTcpClient = new TcpClient();
   this->pbClient = new QPushButton("Client");

   QVBoxLayout *vBoxLayout = new QVBoxLayout();
   vBoxLayout->addWidget(this->pRobot->gbGroup);
   vBoxLayout->addWidget(this->pbClient);
   vBoxLayout->addWidget(this->pTcpClient->gbGroup);

   QWidget *windowMS = new QWidget();
   windowMS->setLayout(vBoxLayout);

   QHBoxLayout *hBoxLayout = new QHBoxLayout();
   hBoxLayout->addWidget(windowMS);

   QWidget *windowGen = new QWidget();
   windowGen->setLayout(hBoxLayout);

   setCentralWidget(windowGen);

   this->setWindowTitle(tr("ARC - V01"));
   this->move(10, 10);

   tmr = new QTimer(this);
   connect(tmr, SIGNAL(timeout()), this, SLOT(Timeout()));
   //tmr->start(100);

   connect(pbClient, &QPushButton::released, this, &MainWindow::pbClient_released);

}

MainWindow::~MainWindow()
{
   if (pRobot)
   {
      delete pRobot;
      pRobot = nullptr;
   }
}

void MainWindow::Timeout()
{
   return;
}

void MainWindow::pbClient_released()
{
   ARCCode_t code = SendRecJoint(this->pRobot);
   return;
}

ARCCode_t MainWindow::SendRecJoint(Robot* robot)
{
   MyTcpSocket s;
   Array<double, 6, 1> j;
   ARCCode_t code = s.doConnect(robot->pJoint->get_joints_deg(), j);
   if(code == ARC_CODE_OK)
      robot->pRealJoint->set_joints_deg(j);
   return code;
}
#include "mainwindow.h"
#include "kinematic.h"

#include <QMainWindow>
#include <QWidget>
#include <QFormLayout>
#include <QHBoxLayout>

#define VERSION "1.1.0"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

   this->pKinematic = new Kinematic(this);
   this->pRobot = new Robot("Robot1");
   this->pTcpClient = new TcpClient();
   this->pbKinematic = new QPushButton("Kinematic");

   QVBoxLayout *vBoxLayout = new QVBoxLayout();
   vBoxLayout->addWidget(this->pRobot->gbGroup);
   vBoxLayout->addWidget(this->pbKinematic);
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

   connect(pbKinematic, &QPushButton::clicked, this, &MainWindow::pbKinematic_clicked);
}

MainWindow::~MainWindow()
{
   if (pRobot) {
      delete pRobot;
      pRobot = nullptr;
   }
   if (pKinematic) {
      delete pKinematic;
      pKinematic = nullptr;
   }
}

void MainWindow::pbKinematic_clicked()
{
   pKinematic->exec();
}

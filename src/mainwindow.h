#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include <map>

#include "joint.h"
#include "robot.h"

#include <QBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QWidget>
#include <QGroupBox>
#include <QStringList>
#include <QCheckBox>
#include <QTimer>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void pbClient_released();

private:
    Robot* pRobot;
    QTimer* tmr;
    QPushButton *pbClient;
    ARCCode_t SendRecJoint(Robot*);

public slots:
    void Timeout();
};

#endif // MAINWINDOW_H

#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "global.h"

#include "robi.h"
#include "path_linear.h"
#include "trajectory_scurve.h"

#include <QDialog>
#include <QLabel>
#include <QWidget>
#include <QFormLayout>
#include <QPushButton>
#include <QDebug>

class Kinematic : public QDialog {
private:
    QLabel *lbTitle;
    QPushButton *pbPushButton;

private slots:
    void pbPushButton_released();

public:
    explicit Kinematic(QWidget *parent = 0);
    ~Kinematic();
    void test();
};

#endif // KINEMATIC_H
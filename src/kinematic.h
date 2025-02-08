#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "robi.h"

#include <QDialog>
#include <QLabel>
#include <QWidget>
#include <QFormLayout>
#include <QPushButton>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Kinematic : public QDialog {
private:
    QLabel *lbTitle;
    QPushButton *pbPushButton;
    Robi *robi;

private slots:
    void pbPushButton_released();

public:
    explicit Kinematic(QWidget *parent = 0);
    ~Kinematic();
};

#endif // KINEMATIC_H
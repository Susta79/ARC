#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <QDialog>
#include <QLabel>
#include <QWidget>

class Kinematic : public QDialog {
private:
    QLabel *lbTitle;

public:
    explicit Kinematic(QWidget *parent = 0);
    ~Kinematic();
};

#endif // KINEMATIC_H
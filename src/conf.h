#ifndef CONF_H
#define CONF_H

#include <QGroupBox>
#include <QRadioButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

class Conf
{
private:
    QGroupBox *gbFrontBack;
    QRadioButton *cbFront;
    QRadioButton *cbBack;
    QGroupBox *gbUpDown;
    QRadioButton *cbUp;
    QRadioButton *cbDown;
    QGroupBox *gbPosNeg;
    QRadioButton *cbPositive;
    QRadioButton *cbNegative;

public:
    QGroupBox *gbConf;

    Conf();
    ~Conf();

    // Front
    bool get_front(){ return this->cbFront->isChecked(); }
    void set_front(){ this->cbFront->setChecked(true); }

    // Back
    bool get_back(){ return this->cbBack->isChecked(); }
    void set_back(){ this->cbBack->setChecked(true); }

    // Up
    bool get_up(){ return this->cbUp->isChecked(); }
    void set_up(){ this->cbUp->setChecked(true); }

    // Down
    bool get_down(){ return this->cbDown->isChecked(); }
    void set_down(){ this->cbDown->setChecked(true); }

    // Positive
    bool get_positive(){ return this->cbPositive->isChecked(); }
    void set_positive(){ this->cbPositive->setChecked(true); }

    // Negative
    bool get_negative(){ return this->cbNegative->isChecked(); }
    void set_negative(){ this->cbNegative->setChecked(true); }

};

#endif // CONF_H
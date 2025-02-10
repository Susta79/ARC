#include "kinematic.h"

Kinematic::Kinematic(QWidget *parent) : QDialog(parent) {

    this->lbTitle = new QLabel("Label");
    this->pbPushButton = new QPushButton("Button");

    QHBoxLayout *HLayout = new QHBoxLayout(this);
    HLayout->addWidget (lbTitle);
    HLayout->addWidget (pbPushButton);

    this->setLayout (HLayout);

    connect(pbPushButton, &QPushButton::released, this, &Kinematic::pbPushButton_released);
}

Kinematic::~Kinematic() {
    if (lbTitle) {
        delete lbTitle;
        lbTitle = nullptr;
    }
    if (this->pbPushButton) {
        delete this->pbPushButton;
        this->pbPushButton = nullptr;
    }
}

void Kinematic::pbPushButton_released() {
    this->test();
}

void Kinematic::test(){
    // link dimensions of the robot in mm
    double a1x = 0;
    double a1y = 0;
    double a1z = 650;

    double a2x = 400;
    double a2y = 0;
    double a2z = 680;

    double a3x = 0;
    double a3y = 0;
    double a3z = 1100;

    double a4x = 766;
    double a4y = 0;
    double a4z = 230;

    double a5x = 345;
    double a5y = 0;
    double a5z = 0;

    double a6x = 244;
    double a6y = 0;
    double a6z = 0;

    double a7x = 1;
    double a7y = 0;
    double a7z = 0;

    Eigen::Vector3d offset;
    offset << 0,0,a1z;

    Robi *robi = new Robi(a1z, a2x, a2z, a3z, a4z, a4x, a5x, a6x);
    //this->init(a1z, a2x, a2z, a3z, a4z, a4x, a5x, a6x);


    // parameters for determining optimal trajectory
    double N = 500;
    double lr = .995;

    double fr = 20;
    double dt = 1/fr;
    double buffer = 10;

    Eigen::VectorXd speed_limits(6);
    Eigen::VectorXd accel_limits(6);
    speed_limits << 10,10,10,10,10,10;
    accel_limits << 10,10,10,10,10,10;
    speed_limits *= DEG_TO_RAD;
    accel_limits *= DEG_TO_RAD;

    //Eigen::VectorXd s(500);
    //s = this->linspace(0,1,N);
    Eigen::VectorXd s;
    s.setLinSpaced(N,0,1);
    std::cout << "s: " << std::endl << s << std::endl;


    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2;
    double x3, y3, z3, a3, b3, c3;
    
    x1 = 100;
    y1 = 1500;
    z1 = 1000;
    a1 = 50;
    b1 = 50;
    c1 = 50;

    x2 = 200;
    y2 = 100;
    z2 = 500;

    x3 = 200;
    y3 = 0;
    z3 = 0;
    a3 = 250;
    b3 = 0;
    c3 = -90;

    Eigen::Vector3d euler1;
    Eigen::Vector3d euler2;
    euler1 << a1,b1,c1;
    euler2 << a3,b3,c3;
    euler1 *= DEG_TO_RAD;
    euler2 *= DEG_TO_RAD;

    Eigen::Vector3d P0;
    Eigen::Vector3d P1;
    Eigen::Vector3d P2;
    P0 << x1,y1,z1;
    P1 << x2,y2,z2;
    P2 << x3,y3,z3;
    Eigen::Vector3d CC, U, V;
    double CR, alpha;
    Path_circular *path_circular = new Path_circular(P0, P1, P2, euler1, euler2, N, dt);
    std::cout << "CC: " << path_circular->get_CC().transpose() << std::endl;
    std::cout << "CR: " << path_circular->get_CR() << std::endl;
    std::cout << "U: " << path_circular->get_U().transpose() << std::endl;
    std::cout << "V: " << path_circular->get_V().transpose() << std::endl;
    std::cout << "alpha: " << path_circular->get_alpha() << std::endl;

    double L;
    if (path_circular->path_lenght(&L) != ARC_CODE_OK)
        std::cout << "Error" << std::endl;
    else
        std::cout << "L: " << L << std::endl;

    Eigen::MatrixXd xyzs0 = path_circular->xyz_array();
    //std::cout << "xyzs0: " << std::endl << xyzs0 << std::endl;

    Eigen::MatrixXd abcs0 = path_circular->slerp(true, true);
    //std::cout << "abcs0: " << std::endl << abcs0 << std::endl;

    Eigen::MatrixXd xyzabcs0(xyzs0.rows(), xyzs0.cols() + abcs0.cols());
    xyzabcs0 << xyzs0, abcs0;
    std::cout << "xyzabcs0: " << std::endl << xyzabcs0 << std::endl;

    delete robi;
}

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

    double x0, y0, z0, a0, b0, c0;
    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2, a2, b2, c2;
    
    /*
    x0 = 100;
    y0 = 1500;
    z0 = 1000;
    a0 = 50;
    b0 = 50;
    c0 = 50;

    x1 = 200;
    y1 = 100;
    z1 = 500;
    a1 = 100;
    b1 = 0;
    c1 = 0;

    x2 = 200;
    y2 = 0;
    z2 = 0;
    a2 = 250;
    b2 = 0;
    c2 = -90;

    Eigen::Vector3d P0(x0, y0, z0);
    Eigen::Vector3d P1(x1, y1, z1);
    Eigen::Vector3d P2(x2, y2, z2);
    Eigen::Vector3d euler0(a0, b0, c0);
    Eigen::Vector3d euler1(a1, b1, c1);
    Eigen::Vector3d euler2(a2, b2, c2);
    euler0 *= DEG_TO_RAD;
    euler1 *= DEG_TO_RAD;
    euler2 *= DEG_TO_RAD;

    Path_circular *path_circular = new Path_circular(P0, P1, P2, euler0, euler1, euler2, N, dt);
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
    //std::cout << "xyzabcs0: " << std::endl << xyzabcs0 << std::endl;
    std::cout << "xyzabcs0: " << std::endl;
    for (size_t i = 0; i < xyzabcs0.rows(); i++)
    {
        std::cout << "i: " << i << "; " << xyzabcs0.row(i) << std::endl;
    }
    */
    
    x0 = 100;
    y0 = 1500;
    z0 = 1000;
    a0 = 50;
    b0 = 50;
    c0 = 50;

    x1 = 500;
    y1 = -500;
    z1 = 1000;
    a1 = 250;
    b1 = 0;
    c1 = -90;

    Eigen::Vector3d P0(x0, y0, z0);
    Eigen::Vector3d P1(x1, y1, z1);
    Eigen::Vector3d euler0(a0, b0, c0);
    Eigen::Vector3d euler1(a1, b1, c1);

    Path_linear *path_linear = new Path_linear(P0, P1, euler0, euler1, N, dt, true, false);
    double L;
    path_linear->path_lenght(&L, false);
    qDebug() << "L: " << L;
    Trajectory_scurve *trajectory_scurve = new Trajectory_scurve(2, 10, 100, path_linear);
    double T = trajectory_scurve->get_T();
    qDebug() << "T: " << T;
    Eigen::Vector<double, 6> pose;
    Eigen::Vector<double, 6> joint;
    double dist;
    ARCCode_t ret;
    for (size_t i = 0; i <= T*1000; i+=5)
    {
        dist = trajectory_scurve->get_dist_at_t((double)i/1000.0, false);
        //std::cout << "t: " << i/1000.0 << "; " << dist << std::endl;
        qDebug() << "t: " << i/1000.0 << "; " << dist;
        ret = path_linear->get_pose_at_s((dist / L), false, false, &pose);
        if ( ret == ARC_CODE_OK ) {
            //std::cout << "\tpose: X=" << pose(0) << "; Y=" << pose(1) << "; Z=" << pose(2) << "; A=" << pose(3) << "; B=" << pose(4) << "; C=" << pose(5) << std::endl;
            qDebug() << "\tpose: X=" << pose(0) << "; Y=" << pose(1) << "; Z=" << pose(2) << "; A=" << pose(3) << "; B=" << pose(4) << "; C=" << pose(5);
            ret = robi->inv_kin(pose, true, true, false, false, &joint);
            if ( ret == ARC_CODE_OK ) {
                qDebug() << "\t\tjoint: J1=" << joint(0) << "; J2=" << joint(1) << "; J3=" << joint(2) << "; J4=" << joint(3) << "; J5=" << joint(4) << "; J6=" << joint(5);
            }
            else{
                qDebug() << "Error: " << ret;
            }
        }
        else{
            //std::cout << "Error: " << ret << std::endl;
            qDebug() << "Error: " << ret;
        }
    }
    
    delete robi;
}

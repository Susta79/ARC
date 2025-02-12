#include "path_circular.h"

Path_circular::Path_circular(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d euler0, Eigen::Vector3d euler1, Eigen::Vector3d euler2, double N, double dt) : Path (P0, P1, euler0, euler1, N, dt){
    Eigen::Vector3d v01, v12, NN, vc0, vc1, vc2, cross1;
    this->P2 = P2;
    this->euler2 = euler2;
    this->CC = circumcenter();
    this->CR = circumradius();
    v01 = this->P1 - this->P0;
    v12 = this->P2 - this->P1;
    NN = v01.cross(v12);
    if (NN == Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        this->CC = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->CR = 0.0;
        this->U = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->V = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->alpha = 0.0;
        return;
    }

    NN.normalize();
    this->U = this->P0 - this->CC;
    this->U.normalize();
    this->V = NN.cross(this->U);
    this->V.normalize();

    vc0 = this->P0 - this->CC;
    vc1 = this->P1 - this->CC;
    vc2 = this->P2 - this->CC;

    this->alpha = acos(vc0.dot(vc2)/(vc0.norm()*vc2.norm()));
    if (abs(vc0.dot(vc2)) != 1){
        cross1 = vc0.cross(vc2);
        cross1.normalize();
        if (NN.dot(cross1) < 0)
            this->alpha = M_2_PI - this->alpha;
    }

    this->alpha1 = acos(vc0.dot(vc1)/(vc0.norm()*vc1.norm()));
    if (abs(vc0.dot(vc1)) != 1){
        cross1 = vc0.cross(vc1);
        cross1.normalize();
        if (NN.dot(cross1) < 0)
            this->alpha1 = M_2_PI - this->alpha1;
    }
}

double Path_circular::alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads){
    double alpha;
    alpha = q1.angularDistance(q2);
    if (out_rads == false)
        alpha *= RAD_TO_DEG;
    return alpha;
}

//path functions
Eigen::Vector3d Path_circular::circumcenter(){
    Eigen::Vector3d cc;
    double a = (this->P2 - this->P1).dot(this->P2 - this->P1);
    double b = (this->P0 - this->P2).dot(this->P0 - this->P2);
    double c = (this->P1 - this->P0).dot(this->P1 - this->P0);        
    cc = ((a)*(b+c-a)*this->P0 + b*(c+a-b)*this->P1 +c*(a+b-c)*this->P2)/((a)*(b+c-a) + b*(c+a-b) + c*(a+b-c));
    return cc;
}

double Path_circular::circumradius(){
    double a = (this->P0-this->P1).norm();
    double b = (this->P1-this->P2).norm();
    double c = (this->P2-this->P0).norm();
    double cr = a*b*c/ (2*(this->P0-this->P1).cross(this->P1-this->P2).norm());
    return cr;
}

ARCCode_t Path_circular::circle_params(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    Eigen::Vector3d v01, v12, NN, vc0, vc2, cross1;
    this->P0 = P0;
    this->P1 = P1;
    this->P2 = P2;
    this->CC = circumcenter();
    this->CR = circumradius();
    v01 = this->P1 - this->P0;
    v12 = this->P2 - this->P1;
    NN = v01.cross(v12);
    if (NN == Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        this->CC = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->CR = 0.0;
        this->U = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->V = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->alpha = 0.0;
        return ARC_ERR_APP_CIRC_POINTS_COLLIN;
    }

    NN.normalize();
    this->U = (this->P0 - this->CC);
    this->U = this->U / this->U.norm();
    this->V = NN.cross(this->U);

    vc0 = this->P0 - this->CC;
    vc2 = this->P2 - this->CC;

    this->alpha = acos(vc0.dot(vc2)/(vc0.norm()*vc2.norm()));

    if (abs(vc0.dot(vc0)) != 1){
        cross1 = vc0.cross(vc2);
        cross1.normalize();
        if (NN.dot(cross1) < 0)
        this->alpha = M_2_PI - this->alpha;
    }

    return ARC_CODE_OK;
}

ARCCode_t Path_circular::circum_alpha(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, double *alpha){
    Eigen::Vector3d CC = circumcenter();
    Eigen::Vector3d v01 = P1-P0;
    Eigen::Vector3d v12 = P2-P1;
    Eigen::Vector3d N = v01.cross(v12);
    if (N(0) == 0 and N(1)==0 and N(2) ==0)
        return ARC_ERR_APP_CIRC_POINTS_COLLIN;

    Eigen::Vector3d vc0 = P0-CC;
    Eigen::Vector3d vc2 = P2-CC;

    *alpha = acos(vc0.dot(vc2)/(vc0.norm()*vc2.norm()));

    if (abs(vc0.dot(vc0)) != 1){
        Eigen::Vector3d cross1 = vc0.cross(vc2);
        cross1.normalize();
        if (N.dot(cross1) < 0)
            *alpha = M_2_PI - *alpha;
    }

    return ARC_CODE_OK;
}



Eigen::Vector3d Path_circular::circular_path_lambda(Eigen::Vector3d CC, double CR, Eigen::Vector3d U, Eigen::Vector3d V, double theta){
    return CC + CR*cos(theta)*U + CR*sin(theta)*V;
}




ARCCode_t Path_circular::path_lenght(double *arc_lenght){
    double CR, alpha;
    ARCCode_t ret;
    CR = this->circumradius();
    ret = this->circum_alpha(P0, P1, P2, &alpha);
    if (ret == ARC_CODE_OK)
        *arc_lenght = CR * alpha;
    else
        *arc_lenght = 0.0;
    return ret;
}

Eigen::MatrixXd Path_circular::xyz_array(){
    size_t len_s = this->s.size();
    Eigen::VectorXd s_array = this->s * this->alpha;
    Eigen::Vector3d xyz;
    Eigen::MatrixXd xyz_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        xyz = this->CC + this->CR*cos(s_array(i))*this->U + this->CR*sin(s_array(i))*this->V;
        xyz_array.row(i) << xyz(0), xyz(1), xyz(2);
    }
    return xyz_array;
}

Eigen::MatrixXd Path_circular::slerp(bool in_rads, bool out_rads){
    Eigen::Quaterniond q0, q1, q2, qSlerp;
    Eigen::Vector3d euler;
    Eigen::Matrix3d R, R0, R1, R2;
    size_t len_s;

    R0 = rot_mat_from_euler(this->euler0, in_rads = in_rads);
    R1 = rot_mat_from_euler(this->euler1, in_rads = in_rads);
    R2 = rot_mat_from_euler(this->euler2, in_rads = in_rads);

    q0 = q_from_rot_mat(R0);
    q1 = q_from_rot_mat(R1);
    q2 = q_from_rot_mat(R2);

    double alpha_ratio = this->alpha1/this->alpha;
    len_s = this->s.size();
    Eigen::MatrixXd euler_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        if (i<alpha_ratio)
            qSlerp = q0.slerp(s(i)/alpha_ratio, q1);
        else
            qSlerp = q1.slerp((s(i)-alpha_ratio)/(1-alpha_ratio), q2);
        //qSlerp = q0.slerp(s(i), q1);
        R = rot_mat_from_q(qSlerp);
        euler = euler_from_rot_mat(R, out_rads);
        euler_array.row(i) << euler(0), euler(1), euler(2);
    }

    if (out_rads == false)
        euler_array *= RAD_TO_DEG;
    
    return euler_array;
}

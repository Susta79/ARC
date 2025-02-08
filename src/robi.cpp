#include "robi.h"

Robi::Robi(){
    this->init(650.0, 400.0, 680.0, 1100.0, 766.0, 230.0, 345.0, 244.0);
}

Robi::Robi(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x){
    this->init(a1z, a2x, a2z, a3z, a4z, a4x, a5x, a6x);
    std::cout << "Robi constructor" << std::endl;
}

Robi::~Robi(){
    std::cout << "Robi distructor" << std::endl;
}

void Robi::init(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x){
    this->a1z = a1z;
    this->a2x = a2x;
    this->a2z = a2z;
    this->a3z = a3z;
    this->a4z = a4z;
    this->a4x = a4x;
    this->a5x = a5x;
    this->a6x = a6x;
}

Eigen::Matrix3d Robi::rotx(double angle){
    Eigen::Matrix3d R;
    //R << 1, 0, 0,
    //    0, cos(angle), -sin(angle),
    //    0, sin(angle), cos(angle);
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return R;
}

Eigen::Matrix3d Robi::roty(double angle){
    Eigen::Matrix3d R;
    //R << cos(angle), 0, sin(angle),
    //    0, 1, 0,
    //    -sin(angle), 0, cos(angle);
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return R;
}

Eigen::Matrix3d Robi::rotz(double angle){
    Eigen::Matrix3d R;
    //R << cos(angle), -sin(angle), 0,
    //    sin(angle), cos(angle), 0,
    //    0, 0, 1;
    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return R;
}

//double Robi::rads_d(double angle){
//    return angle * RAD_TO_DEG;
//}
//
//double Robi::degs_d(double angle){
//    return angle * DEG_TO_RAD;
//}
//
//Eigen::Vector3d Robi::rads_vec3d(Eigen::Vector3d vec){
//    return vec * M_PI / 180.0;
//}
//
//Eigen::Vector3d Robi::degs_vec3d(Eigen::Vector3d vec){
//    return vec * 180.0 / M_PI;
//}
//
//Eigen::MatrixXd Robi::rads_matXd(Eigen::MatrixXd mat){
//    return mat * M_PI / 180.0;
//}
//
//Eigen::MatrixXd Robi::degs_matXd(Eigen::MatrixXd mat){
//    return mat * 180.0 / M_PI;
//}
//
//Eigen::Array<double, 6, 1> rads_arr6d(Eigen::Array<double, 6, 1> angles){
//    return angles * M_PI / 180.0;
//}
//
//Eigen::Array<double, 6, 1> degs_arr6d(Eigen::Array<double, 6, 1> angles){
//    return angles * 180.0 / M_PI;
//}

Eigen::Affine3d Robi::trans_mat(Eigen::Matrix3d rot, Eigen::Vector3d trans){
    Eigen::Affine3d T;
    T.linear() = rot;
    T.translation() = trans;
    return T;
}

// Orientation functions
Eigen::Matrix3d Robi::rot_mat_from_euler(Eigen::Vector3d euler, bool in_rads){
    if (in_rads == false)
        euler *= DEG_TO_RAD;
    double a = euler(0);
    double b = euler(1);
    double c = euler(2);
    Eigen::Matrix3d R = rotz(c) * roty(b) * rotx(a) ;
    return R;
}

Eigen::Quaterniond Robi::q_from_rot_mat(Eigen::Matrix3d R){
    //Eigen::Quaterniond q;
    //double coeff;
    //q.w() =.5*sqrt(1 + R(0,0) + R(1,1) + R(2,2));
    //coeff = .25*(1/q.w());
    //q.x() = coeff*(R(2,1) - R(1,2));
    //q.y() = coeff*(R(0,2) - R(2,0));
    //q.z() = coeff*(R(1,0)- R(0,1));
    Eigen::Quaterniond q(R);
    return q;
}

Eigen::Matrix3d Robi::rot_mat_from_q(Eigen::Quaterniond q){
    Eigen::Matrix3d R;
    R = q.normalized().toRotationMatrix();
    return R;
}

Eigen::Vector3d Robi::euler_from_rot_mat(Eigen::Matrix3d R, bool out_rads){
    double a = atan2(R(2,1),R(2,2));
    double b = atan2(-R(2,0), sqrt(1-pow(R(2,0),2)));
    double c = atan2( R(1,0),R(0,0));
    Eigen::Vector3d euler;
    euler << a, b, c;
    if (out_rads == false)
        euler *= RAD_TO_DEG;
    return euler;
}

double Robi::alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads){
    double alpha;
    //double dot = q1.dot(q2);
    //double lq1 = q1.norm();
    //double lq2 = q2.norm();
    //if (dot<0){
    //    q1.coeffs() = -q1.coeffs();
    //    dot = q1.dot(q2);
    //}
    //double alpha = acos(dot/(lq1*lq2));
    alpha = q1.angularDistance(q2);
    if (out_rads == false)
        alpha *= RAD_TO_DEG;
    return alpha;
}

Eigen::MatrixXd Robi::slerp(Eigen::Vector3d euler_array1, Eigen::Vector3d euler_array2, Eigen::VectorXd s, bool in_rads, bool out_rads){
    Eigen::Quaterniond q1, q2, qSlerp;
    Eigen::Vector3d euler;
    Eigen::Matrix3d R, R1, R2;
    R1 = rot_mat_from_euler(euler_array1, in_rads = in_rads);
    R2 = rot_mat_from_euler(euler_array2, in_rads = in_rads);

    q1 = q_from_rot_mat(R1);
    q2 = q_from_rot_mat(R2);
    
    int len_s = s.size();
    Eigen::MatrixXd euler_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        qSlerp = q1.slerp(s(i), q2);
        R = rot_mat_from_q(qSlerp);
        euler = euler_from_rot_mat(R, out_rads);
        euler_array.row(i) << euler(0), euler(1), euler(2);
        //euler_array(i,0) = euler(0);
        //euler_array(i,1) = euler(1);
        //euler_array(i,2) = euler(2);
    }
    
    /*
    double alpha = alpha_slerp(q1, q2, out_rads = true);

    Eigen::Quaterniond q;
    for (size_t i = 0; i < len_s; i++)
    {
        q.coeffs() = (sin(alpha*(1-i))/sin(alpha))*q1.coeffs() + (sin(alpha*i)/sin(alpha))*q2.coeffs();
        R = rot_mat_from_q(q);
        euler = euler_from_rot_mat(R, out_rads);
        euler_array(i,0) = euler(0);
        euler_array(i,1) = euler(1);
        euler_array(i,2) = euler(2);
    }
    */

    if (out_rads == false)
        euler_array *= RAD_TO_DEG;
    
    return euler_array;
}

//path functions
Eigen::Vector3d Robi::circumcenter(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    Eigen::Vector3d cc;
    double a = (P2-P1).dot(P2-P1);
    double b = (P0-P2).dot(P0-P2);
    double c = (P1-P0).dot(P1-P0);        
    cc = ((a)*(b+c-a)*P0 + b*(c+a-b)*P1 +c*(a+b-c)*P2)/((a)*(b+c-a) + b*(c+a-b) + c*(a+b-c));
    return cc;
}

Eigen::Vector3d Robi::circumcenter2(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    double CR = circumradius(P0, P1, P2);
    Eigen::Vector3d y = (P0-P1).cross(P2-P1);
    Eigen::Vector3d z = y.cross(P0-P1);
    z = z.normalized();
    Eigen::Vector3d x = sqrt(pow(CR, 2) - (1/4)*pow((P0-P1).norm(), 2)) * z;
    Eigen::Vector3d CC = (P0+P1)/2 + x;
    return CC;
}

Eigen::Vector3d Robi::circumcenter3(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    double denom = 2 * pow((P0-P1).cross(P1-P2).norm(), 2);
    double alpha = pow((P1-P2).norm(), 2) * (P0-P1).dot(P0-P2);
    double beta = pow((P0-P2).norm(), 2) * (P1-P0).dot(P1-P2);
    double gamma = pow((P0-P1).norm(), 2) * (P2-P1).dot(P2-P0);
    Eigen::Vector3d CC = (alpha*P0 + beta*P1 + gamma*P2) / denom;
    return CC;
}

double Robi::circumradius(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    double a = (P0-P1).norm();
    double b = (P1-P2).norm();
    double c = (P2-P0).norm();
    double cr = a*b*c/ (2*(P0-P1).cross(P1-P2).norm());
    return cr;
}

ARCCode_t Robi::circle_params(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d *CC, double *CR, Eigen::Vector3d *U, Eigen::Vector3d *V, double *alpha){
    *CC = circumcenter(P0, P1, P2);
    *CR = circumradius(P0,P1,P2);
    Eigen::Vector3d v01 = P1-P0;
    Eigen::Vector3d v12 = P2-P1;
    Eigen::Vector3d N = v01.cross(v12);
    if (N(0) == 0 and N(1)==0 and N(2) ==0)
        return ARC_ERR_APP_CIRC_POINTS_COLLIN;

    N.normalize();
    *U = P0 - *CC;
    U->normalize();
    *V = N.cross(*U);

    Eigen::Vector3d vc0 = P0 - *CC;
    Eigen::Vector3d vc2 = P2 - *CC;

    *alpha = acos(vc0.dot(vc2)/(vc0.norm()*vc2.norm()));

    if (abs(vc0.dot(vc0)) != 1){
        Eigen::Vector3d cross1 = vc0.cross(vc2);
        cross1.normalize();
        if (N.dot(cross1) < 0)
            *alpha = M_2_PI - *alpha;
    }

    return ARC_CODE_OK;
}

ARCCode_t Robi::circum_alpha(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, double *alpha){
    Eigen::Vector3d CC = circumcenter(P0,P1,P2);
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

Eigen::MatrixXd Robi::circular_path(Eigen::Vector3d CC, double CR, Eigen::Vector3d U, Eigen::Vector3d V, Eigen::VectorXd s){
    int len_s = s.size();
    Eigen::Vector3d xyz;
    Eigen::MatrixXd xyz_array(len_s,3);
    for (size_t i = 0; i < len_s; i++){
        xyz = CC + CR*cos(s(i))*U + CR*sin(s(i))*V;
        xyz_array.row(i) << xyz(0), xyz(1), xyz(2);
    }
    return xyz_array;
}

//def circular_path_lambda(CC, CR,U,V):
//    return lambda theta: np.array([CC + CR*np.cos(theta)*U + CR*np.sin(theta)*V])
double Robi::arc_length(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
    double CR, alpha, arc_length;
    CR = circumradius(P0, P1, P2);
    circum_alpha(P0, P1, P2, &alpha);
    arc_length = CR * alpha;
    return arc_length;
}

//Eigen::VectorXd Robi::linspace(double start, double end, int num) {
//    Eigen::VectorXd linspace(num);
//    double step = (end - start) / (num - 1);
//    for (int i = 0; i < num; ++i) {
//        linspace(i) = start + step * i;
//    }
//    return linspace;
//}

ARCCode_t Robi::for_kin(Eigen::Array<double, 6, 1> angles, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *xyzabc, Eigen::Affine3d *t16){
    Eigen::Vector3d trans1, trans2, trans3, trans4, trans5, trans6, xyz, abc;
    Eigen::Matrix3d rot1, rot2, rot3, rot4, rot5, rot6, r16;
    Eigen::Affine3d t1, t2, t3, t4, t5, t6;

    trans1 << 0, 0, this->a1z;
    trans2 << this->a2x, 0, this->a2z;
    trans3 << 0, 0, this->a3z;
    trans4 << this->a4x, 0, this->a4z;
    trans5 << this->a5x, 0, 0;
    trans6 << this->a6x, 0, 0;

    if (in_rads == false)
        angles *= DEG_TO_RAD;

    rot1 = this->rotz(angles[0]);
    rot2 = this->roty(angles[1]);
    rot3 = this->roty(angles[2]);
    rot4 = this->rotx(angles[3]);
    rot5 = this->roty(angles[4]);
    rot6 = this->rotx(angles[5]);

    t1 = trans_mat(rot1, trans1);
    t2 = trans_mat(rot2, trans2);
    t3 = trans_mat(rot3, trans3);
    t4 = trans_mat(rot4, trans4);
    t5 = trans_mat(rot5, trans5);
    t6 = trans_mat(rot6, trans6);

    // Combined transformation: from Frame 6 to Frame 1
    *t16 = t1 * t2 * t3 * t4 * t5 * t6;

    r16 = t16->linear();
    xyz = t16->translation();
    double a = atan2(r16(2,1),r16(2,2));
    double b = atan2(-r16(2,0), pow(sqrt(1-r16(2,0)),2));
    double c = atan2( r16(1,0),r16(0,0));
    abc <<  atan2(r16(2,1),r16(2,2)),
            atan2(-r16(2,0), pow(sqrt(1-r16(2,0)),2)),
            atan2( r16(1,0),r16(0,0));

    if (out_rads == false)
        abc *= RAD_TO_DEG;

    *xyzabc << xyz, abc;

   return ARC_CODE_OK;
}

ARCCode_t Robi::inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> *joint){
    double x, y, z, a, b, c, j1, j2, j3, j4, j5, j6;
    double WPxy, l, h, ro, b4x, alpha, cosbeta, beta, cosgamma, gamma, delta;
    Eigen::Vector3d TCPxyz, x_hat, WP;
    Eigen::Matrix3d R, Rarm, Rwrist;
    x = xyzabc(0);
    y = xyzabc(1);
    z = xyzabc(2);
        
    a = xyzabc(3);
    b = xyzabc(4);
    c = xyzabc(5);
    if (in_rads == false){
        a *= DEG_TO_RAD;
        b *= DEG_TO_RAD;
        c *= DEG_TO_RAD;
    }

    TCPxyz << x, y, z;
    R = rotz(c) * roty(b) *rotx(a);

    // x_hat = R[:,0]
    x_hat = R * Eigen::Vector3d::UnitX();
    WP = TCPxyz - (this->a6x * x_hat.normalized());
    j1 = atan2(WP(1), WP(0));
    WPxy = sqrt(pow(WP(0),2) + pow(WP(1),2));
    if (front_pose)
        l = WPxy - this->a2x;
    else
        l = WPxy + this->a2x;

    h = WP(2) - this->a1z - this->a2z;
    ro = sqrt(pow(h,2) + pow(l,2));
    b4x = sqrt(pow(this->a4z,2) + pow(this->a4x + this->a5x,2));
        
    if (ro > (this->a3z+b4x)){
        // print("arm cannot stretch this far")
        // It is not possible to reach that point
        return ARC_ERR_KIN__POSE_NOT_REACHABLE;
    }
    if (ro < abs(this->a3z-b4x)){
        //print("arm cannot reach this point")
        // J2 too close to the robot himself
        return ARC_ERR_APP_J2_TOO_CLOSE;
    }
    alpha = atan2(h,l);
    cosbeta = (pow(ro,2) + pow(this->a3z,2)-pow(b4x,2))/(2*ro*this->a3z);
    beta = atan2(sqrt(1-pow((cosbeta),2)), cosbeta);
    
    cosgamma = (pow(this->a3z,2) + pow(b4x,2) - pow(ro,2))/(2*this->a3z*b4x);
    gamma = atan2(sqrt(1-pow(cosgamma,2)), cosgamma);
    delta = atan2(this->a4x+this->a5x,this->a4z);

    if (front_pose){
        if (up_pose){
            j2 = M_PI_2 - alpha - beta;
            j3 = M_PI-gamma-delta;
        } else {
            j2 = M_PI/2 - alpha + beta;
            j3 = -M_PI+gamma-delta;
        }
    }

    if (front_pose == false){
        j1 = j1 + M_PI;
        if (up_pose){
            j2 = -M_PI_2+ alpha+beta;
            j3 = -M_PI+gamma-delta;
        }
        else{
            j2 = -M_PI_2 - beta + alpha;
            j3 = M_PI - gamma - delta;
        }
    }

    Rarm = rotz(j1) * roty(j2+j3);
    Rwrist = Rarm.transpose() * R;

    j4 = atan2(Rwrist(1,0),-Rwrist(2,0));
    j5 = atan2(sqrt(1-pow(Rwrist(0,0),2)), Rwrist(0,0));
    j6 = atan2(Rwrist(0,1), Rwrist(0,2));

    *joint << j1,j2,j3,j4,j5,j6;
    
    if (out_rads == false)
        *joint *= RAD_TO_DEG;
    
    return ARC_CODE_OK;
}

ARCCode_t Robi::Jacobian(Eigen::Array<double, 6, 1> thetas, bool in_rads, Eigen::Matrix<double, 6, 6> *J){
    double j1, j2, j3, j4, j5, j6;
    j1 = thetas[0];
    j2 = thetas[1];
    j3 = thetas[2];
    j4 = thetas[3];
    j5 = thetas[4];
    j6 = thetas[5];
    if (in_rads == false){
        j1 *= DEG_TO_RAD;
        j2 *= DEG_TO_RAD;
        j3 *= DEG_TO_RAD;
        j4 *= DEG_TO_RAD;
        j5 *= DEG_TO_RAD;
        j6 *= DEG_TO_RAD;
    }

    Eigen::Matrix<double, 6, 6> jacobian;
        
        
    jacobian(0, 0) = -a2x*sin(j1) - a3z*sin(j1)*sin(j2) + a4x*(sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3)) + a4z*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a5x*(sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3)) + a6x*((-(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3))*cos(j5));
    jacobian(0, 1) = a3z*cos(j1)*cos(j2) + a4x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a4z*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a6x*(-(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5));
    jacobian(0, 2) = a4x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a4z*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a6x*(-(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5));
    jacobian(0, 3) = a6x*((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*sin(j4) - sin(j1)*cos(j4))*sin(j5);
    jacobian(0, 4) = a6x*((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*cos(j5) - (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5));
    jacobian(0, 5) = 0;

    jacobian(1, 0) = a2x*cos(j1) + a3z*sin(j2)*cos(j1) + a4x*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a4z*(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2)) + a5x*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a6x*((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5));
    jacobian(1, 1) = a3z*sin(j1)*cos(j2) + a4x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a4z*(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a6x*(-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5));
    jacobian(1, 2) = a4x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a4z*(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a6x*(-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5));
    jacobian(1, 3) = a6x*((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*sin(j4) + cos(j1)*cos(j4))*sin(j5);
    jacobian(1, 4) = a6x*((-(sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) + sin(j4)*cos(j1))*cos(j5) - (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5));
    jacobian(1, 5) = 0;

    jacobian(2, 0) = 0;
    jacobian(2, 1) = -a3z*sin(j2) + a4x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a4z*(-sin(j2)*cos(j3) - sin(j3)*cos(j2)) + a5x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4));
    jacobian(2, 2) = a4x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a4z*(-sin(j2)*cos(j3) - sin(j3)*cos(j2)) + a5x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4));
    jacobian(2, 3) = -a6x*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j4)*sin(j5);
    jacobian(2, 4) = a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j4)*cos(j5) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5));
    jacobian(2, 5) = 0;

    jacobian(3, 0) = 0;
    jacobian(3, 1) = ((((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*sin(j6) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + (-((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*cos(j6) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j4)*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    jacobian(3, 2) = ((((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*sin(j6) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + (-((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*cos(j6) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j4)*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    jacobian(3, 3) = ((-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6)*cos(j5) + (-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j5)*cos(j6) - (sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j6)*cos(j4))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    jacobian(3, 4) = ((-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*sin(j6)/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) - ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*cos(j6)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    jacobian(3, 5) = (1 + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*(-(sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));

    jacobian(4, 0) = 0;
    jacobian(4, 1) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5)*cos(j4))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + 2*(sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    jacobian(4, 2) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5)*cos(j4))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + 2*(sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    jacobian(4, 3) = (-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j5)/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) - (sin(j2)*sin(j3) - cos(j2)*cos(j3))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*sin(j4)*sin(j5)/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    jacobian(4, 4) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j4)*cos(j5) - 2*(-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    jacobian(4, 5) = 0;

    jacobian(5, 0) = ((-(-(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) - (sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3))*cos(j5))*(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + ((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    jacobian(5, 1) = ((-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + (-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    jacobian(5, 2) = ((-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + (-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    jacobian(5, 3) = (-(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*sin(j4) - sin(j1)*cos(j4))*sin(j5)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + ((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*sin(j4) + cos(j1)*cos(j4))*sin(j5)/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    jacobian(5, 4) = (((-(sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) + sin(j4)*cos(j1))*cos(j5) - (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)) + (-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*(-(-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*cos(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    jacobian(5, 5) = 0;

    *J = jacobian;

    return ARC_CODE_OK;
}

void Robi::test(){
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

    this->init(a1z, a2x, a2z, a3z, a4z, a4x, a5x, a6x);


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
    if (this->circle_params(P0, P1, P2, &CC, &CR, &U, &V, &alpha) != ARC_CODE_OK)
        std::cout << "Error" << std::endl;
    else{
        std::cout << "CC: " << CC.transpose() << std::endl;
        std::cout << "CR: " << CR << std::endl;
        std::cout << "U: " << U.transpose() << std::endl;
        std::cout << "V: " << V.transpose() << std::endl;
        std::cout << "alpha: " << alpha << std::endl;
    }

    double L = this->arc_length(P0,P1,P2);
    std::cout << "L: " << L << std::endl;

    Eigen::MatrixXd xyzs0 = this->circular_path(CC, CR, U, V, s*alpha);
    std::cout << "xyzs0: " << std::endl << xyzs0 << std::endl;

    Eigen::MatrixXd abcs0 = slerp(euler1, euler2, s, true, true);
    std::cout << "abcs0: " << std::endl << abcs0 << std::endl;

    Eigen::MatrixXd xyzabcs0(xyzs0.rows(), xyzs0.cols() + abcs0.cols());
    xyzabcs0 << xyzs0, abcs0;
    std::cout << "xyzabcs0: " << std::endl << xyzabcs0 << std::endl;
}

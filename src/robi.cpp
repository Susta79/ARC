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

ARCCode_t Robi::interp_value(double si, Eigen::ArrayXd s_array, Eigen::ArrayXd val_array, double *val, int *idx){
    int _idx;
    double _val, ds, N;
    N = s_array.size();
    Eigen::ArrayXd s_array_si = (s_array - si).abs();
    s_array_si.minCoeff(&_idx);
    
    if (s_array(_idx) == si)
        _val = val_array(_idx);
    
    if (s_array(_idx) > si){
        ds = s_array(_idx) - s_array(_idx-1);
        _val = (s_array(_idx)-si)*val_array(_idx-1)/ds + (si- s_array(_idx-1))*val_array(_idx)/ds;
    }

    if (s_array(_idx) < si){
        ds = s_array(_idx+1)- s_array(_idx);
        _val = (si - s_array(_idx))*val_array(_idx+1)/ds + (s_array(_idx+1)-si)*val_array(_idx)/ds;
    }

    *val = _val;
    *idx = _idx;

    return ARC_CODE_OK;
}

//def sd_limits_J(s_array, speed_limits, xyzabcs0, path_func, euler1, euler2, robot_obj,lr = .95, k =5, alpha = None, pose = [True, True]):
// path_func: 1=circle, 2=linear, 3=bezier
/*
ARCCode_t Robi::sd_limits_J(Eigen::ArrayXd s_array, Eigen::ArrayXd speed_limits, Eigen::MatrixXd xyzabcs0, int path_func, Eigen::Vector3d euler1, Eigen::Vector3d euler2, Robi *robot_obj, double lr, int k, double alpha, bool front_pose, bool up_pose){
    Eigen::Array<double, 6, 1> xyzabc0, thetas0, xyzabc1;
    Eigen::Array<bool, 6, 1> violations;
    Eigen::Matrix<double, 6, 6> J, Jinv;
    Eigen::MatrixXd abc1;
    Eigen::Vector3d xyz1;
    double N, s0, s1, ds;
    ARCCode_t ret;
    N = s_array.size();

    Eigen::MatrixXd sd_limit_curve(1,N); // array to store sd limit curve
    sd_limit_curve.setZero();

    //sd_limit_curve = np.zeros([1,N]) ## array to store sd limit curve

    //sd_limit_accels = np.zeros([1,N])
    for (size_t i = 0; i < N; i++)
    {
        s1 = 1.0;
        xyzabc0 = xyzabcs0(i);
        ret = robot_obj->inv_kin(xyzabc0, front_pose, up_pose, true, true, &thetas0);
        if (ret != ARC_CODE_OK)
            return ret;
        
        ret = robot_obj->Jacobian(thetas0, true, &J);
        Jinv = J.inverse();

        violations << true, true, true, true, true, true;

        while (true)
        {
            s0 = s_array(i);
            ds = s1-s0;

            switch (path_func)
            {
            case 2: //linear
                xyz1 = robot_obj->linear_path_lambda(xyzabcs0(i).head(3), xyzabcs0(i).tail(3), Eigen::ArrayXd::LinSpaced(2, s0, s1));
                break;
            
            default:
                break;
            }
            //if alpha is not None: 
            //    xyz1 = path_func(s1*alpha)
            //else:
            //    xyz1 = path_func(s1)  
            Eigen::VectorXd s_1(1);
            s_1 << s1;
            abc1 = robot_obj->slerp(euler1, euler2, s_1, true, true);
            //xyzabc1 = np.concatenate([xyz1,abc1], axis = 1)[0]
            xyzabc1.row(0) << xyz1(0), abc1(0);

            dXtcpdt = (xyzabc1-xyzabc0)/dt
            speeds = np.abs(Jinv@dXtcpdt)

            violations = np.greater(speeds, speed_limits)
            s1 = s0 + (ds)*lr

        }
        

        while True in violations:
        sd =  (ds)/dt

        sd_limit_curve[0,i] = sd
    }
    

    sd_limit_curve = running_mean_filter(sd_limit_curve, k = k)[0]
    
    return sd_limit_curve
}
*/

ARCCode_t Robi::for_kin(Eigen::Vector<double, 6> angles, bool in_rads, bool out_rads, Eigen::Vector<double, 6> *xyzabc, Eigen::Affine3d *t16){
    Eigen::Vector3d trans1, trans2, trans3, trans4, trans5, trans6, xyz, abc;
    Eigen::Matrix3d rot1, rot2, rot3, rot4, rot5, rot6, r16;
    Eigen::Affine3d t1, t2, t3, t4, t5, t6;

    trans1 << 0, 0, this->a1z;
    trans2 << this->a2x, 0, this->a2z;
    trans3 << 0, 0, this->a3z;
    trans4 << this->a4x, 0, this->a4z;
    trans5 << this->a5x, 0, 0;
    trans6 << this->a6x, 0, 0;

    rot1 = rotz(angles[0], in_rads);
    rot2 = roty(angles[1], in_rads);
    rot3 = roty(angles[2], in_rads);
    rot4 = rotx(angles[3], in_rads);
    rot5 = roty(angles[4], in_rads);
    rot6 = rotx(angles[5], in_rads);

    t1 = trans_mat(rot1, trans1, true);
    t2 = trans_mat(rot2, trans2, true);
    t3 = trans_mat(rot3, trans3, true);
    t4 = trans_mat(rot4, trans4, true);
    t5 = trans_mat(rot5, trans5, true);
    t6 = trans_mat(rot6, trans6, true);

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

ARCCode_t Robi::inv_kin(Eigen::Vector<double, 6> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Vector<double, 6> *joint){
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

    TCPxyz << x, y, z;
    R = rotz(c, in_rads) * roty(b, in_rads) *rotx(a, in_rads);

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

    Rarm = rotz(j1, true) * roty(j2+j3, true);
    Rwrist = Rarm.transpose() * R;

    j4 = atan2(Rwrist(1,0),-Rwrist(2,0));
    j5 = atan2(sqrt(1-pow(Rwrist(0,0),2)), Rwrist(0,0));
    j6 = atan2(Rwrist(0,1), Rwrist(0,2));

    *joint << j1,j2,j3,j4,j5,j6;
    
    if (out_rads == false)
        *joint *= RAD_TO_DEG;
    
    return ARC_CODE_OK;
}

ARCCode_t Robi::Jacobian(Eigen::Vector<double, 6> thetas, bool in_rads, Eigen::Matrix<double, 6, 6> *J){
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
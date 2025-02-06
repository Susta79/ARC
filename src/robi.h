#ifndef ROBI_H
#define ROBI_H

#include <include/ARC/error_def.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Robi
{
private:
    double a1z;
    double a2x;
    double a2z;
    double a3z;
    double a4z;
    double a4x;
    double a5x;
    double a6x;

    // Basic functions
    Eigen::Matrix3d rotx(double angle){
        Eigen::Matrix3d R;
        R << 1, 0, 0,
            0, cos(angle), -sin(angle),
            0, sin(angle), cos(angle);
        return R;
    }
    Eigen::Matrix3d roty(double angle){
        Eigen::Matrix3d R;
        R << cos(angle), 0, sin(angle),
            0, 1, 0,
            -sin(angle), 0, cos(angle);
        return R;
    }
    Eigen::Matrix3d rotz(double angle){
        Eigen::Matrix3d R;
        R << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;
        return R;
    }
    double rads(double angle){ return angle * M_PI / 180; }
    double degs(double angle){ return angle * 180 / M_PI; }
    Eigen::Vector3d vec_norm(Eigen::Vector3d vec){ return vec.normalized(); }
    double vec_len(Eigen::Vector3d vec){ return vec.norm(); }
    Eigen::Affine3d trans_mat(Eigen::Matrix3d rot, Eigen::Vector3d trans){
        Eigen::Affine3d T;
        T.linear() = rot;
        T.translation() = trans;
        return T;
    }
    // Orientation functions
    Eigen::Matrix3d rot_mat_from_euler(Eigen::Vector3d euler, bool in_rads){
        if (in_rads == false)
            euler = euler * M_PI / 180;
        double a = euler(0);
        double b = euler(1);
        double c = euler(2);
        Eigen::Matrix3d R = rotz(c) * roty(b) * rotx(a) ;
        return R;
    }
    Eigen::Quaterniond q_from_rot_mat(Eigen::Matrix3d R){
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
    Eigen::Matrix3d rot_mat_from_q(Eigen::Quaterniond q){
        Eigen::Matrix3d R;
        R = q.normalized().toRotationMatrix();
        return R;
    }
    Eigen::Vector3d euler_from_rot_mat(Eigen::Matrix3d R, bool out_rads){
        double a = atan2(R(2,1),R(2,2));
        double b = atan2(-R(2,0), sqrt(1-pow(R(2,0),2)));
        double c = atan2( R(1,0),R(0,0));
        Eigen::Vector3d euler;
        euler << a, b, c;
        if (out_rads == false)
            euler= euler * 180 / M_PI;
        return euler;
    }
    double alpha_slerp(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool out_rads){
        double dot = q1.dot(q2);
        double lq1 = q1.norm();
        double lq2 = q2.norm();
        if (dot<0){
            q1.coeffs() = -q1.coeffs();
            dot = q1.dot(q2);
        }
        double alpha = acos(dot/(lq1*lq2));
        if (out_rads == false)
            alpha = degs(alpha);
        return alpha;
    }
    // N.B. Posso riscrivere questa funzione utilizzando la funzione Slerp del Quaternione di Eigen
    Eigen::MatrixXd slerp(Eigen::Vector3d euler_array1, Eigen::Vector3d euler_array2, Eigen::VectorXd s, bool in_rads, bool out_rads){
        Eigen::Matrix3d R1 = rot_mat_from_euler(euler_array1, in_rads = in_rads);
        Eigen::Matrix3d R2 = rot_mat_from_euler(euler_array2, in_rads = in_rads);

        Eigen::Quaterniond q1, q2;
        q1 = q_from_rot_mat(R1);
        q2 = q_from_rot_mat(R2);

        double alpha = alpha_slerp(q1, q2, out_rads = true);
        int len_s = s.size();
        Eigen::MatrixXd euler_array(len_s,3);

        //Eigen::VectorXd q_array(len_s);
        Eigen::Quaterniond q;
        Eigen::Matrix3d R;
        Eigen::Vector3d euler;
        for (size_t i = 0; i < len_s; i++)
        {
            q.coeffs() = (sin(alpha*(1-i))/sin(alpha))*q1.coeffs() + (sin(alpha*i)/sin(alpha))*q2.coeffs();
            R = rot_mat_from_q(q);
            euler = euler_from_rot_mat(R, out_rads);
            euler_array(i,0) = euler(0);
            euler_array(i,1) = euler(1);
            euler_array(i,2) = euler(2);
        }

        if (out_rads == false)
            euler_array = euler_array * 180 / M_PI;
        
        return euler_array;
    }
    //path functions
    Eigen::Vector3d circumcenter(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
        Eigen::Vector3d cc;
        double a = (P2-P1).dot(P2-P1);
        double b = (P0-P2).dot(P0-P2);
        double c = (P1-P0).dot(P1-P0);        
        cc = ((a)*(b+c-a)*P0 + b*(c+a-b)*P1 +c*(a+b-c)*P2)/((a)*(b+c-a) + b*(c+a-b) + c*(a+b-c));
        return cc;
    }
    Eigen::Vector3d circumcenter2(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
        double CR = circumradius(P0, P1, P2);
        Eigen::Vector3d y = (P0-P1).cross(P2-P1);
        Eigen::Vector3d z = y.cross(P0-P1);
        z = z.normalized();
        Eigen::Vector3d x = sqrt(pow(CR, 2) - (1/4)*pow((P0-P1).norm(), 2)) * z;
        Eigen::Vector3d CC = (P0+P1)/2 + x;
        return CC;
    }
    Eigen::Vector3d circumcenter3(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
        double denom = 2 * pow((P0-P1).cross(P1-P2).norm(), 2);
        double alpha = pow((P1-P2).norm(), 2) * (P0-P1).dot(P0-P2);
        double beta = pow((P0-P2).norm(), 2) * (P1-P0).dot(P1-P2);
        double gamma = pow((P0-P1).norm(), 2) * (P2-P1).dot(P2-P0);
        Eigen::Vector3d CC = (alpha*P0 + beta*P1 + gamma*P2) / denom;
        return CC;
    }
    double circumradius(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2){
        double a = (P0-P1).norm();
        double b = (P1-P2).norm();
        double c = (P2-P0).norm();
        double cr = a*b*c/ (2*(P0-P1).cross(P1-P2).norm());
        return cr;
    }
    // N.B.: Change the error code ARC_ERR_APP_J2_TOO_CLOSE
    ARCCode_t circle_params(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d &CC, double CR, Eigen::Vector3d &U, Eigen::Vector3d &V, double &alpha){
        CC = circumcenter(P0, P1, P2);
        CR = circumradius(P0,P1,P2);
        Eigen::Vector3d v01 = P1-P0;
        Eigen::Vector3d v12 = P2-P1;
        Eigen::Vector3d N = v01.cross(v12);
        if (N(0) == 0 and N(1)==0 and N(2) ==0)
            return ARC_ERR_APP_J2_TOO_CLOSE;
    
        N.normalize();
        U = P0-CC;
        U.normalize();
        V = N.cross(U);
    
        Eigen::Vector3d vc0 = P0-CC;
        Eigen::Vector3d vc2 = P2-CC;

        alpha = acos(vc0.dot(vc2)/(vc0.norm()*vc2.norm()));

        if (abs(vc0.dot(vc0)) != 1){
            Eigen::Vector3d cross1 = vc0.cross(vc2);
            cross1.normalize();
            if (N.dot(cross1) < 0)
                alpha = M_2_PI - alpha;
        }

        return ARC_CODE_OK;
    }

public:
    Robi(double a1z, double a2x, double a2z, double a3z, double a4z, double a4x, double a5x, double a6x){
        this->a1z = a1z;
        this->a2x = a2x;
        this->a2z = a2z;
        this->a3z = a3z;
        this->a4z = a4z;
        this->a4x = a4x;
        this->a5x = a5x;
        this->a6x = a6x;
    }
    //~Robi();
    // Robot object
    ARCCode_t for_kin(Eigen::Array<double, 6, 1> joint, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &xyzabc, Eigen::Affine3d &t16);
    // inv_kin: TODO is still to complete
    ARCCode_t inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &joint);
    ARCCode_t Jacobian(Eigen::Array<double, 6, 1> joint, bool in_rads, Eigen::Matrix<double, 6, 6> &J);
};

#endif // ROBI_H
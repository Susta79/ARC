#include "robi.h"

ARCCode_t Robi::for_kin(Eigen::Array<double, 6, 1> angles, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &xyzabc, Eigen::Affine3d &t16){
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
        angles = angles * M_PI / 180;

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
    t16 = t1 * t2 * t3 * t4 * t5 * t6;

    r16 = t16.linear();
    xyz = t16.translation();
    double a = atan2(r16(2,1),r16(2,2));
    double b = atan2(-r16(2,0), pow(sqrt(1-r16(2,0)),2));
    double c = atan2( r16(1,0),r16(0,0));
    abc <<  atan2(r16(2,1),r16(2,2)),
            atan2(-r16(2,0), pow(sqrt(1-r16(2,0)),2)),
            atan2( r16(1,0),r16(0,0));

    if (out_rads == false)
        abc = abc * 180 / M_PI;

    xyzabc << xyz, abc;

   return ARC_CODE_OK;
}

ARCCode_t Robi::inv_kin(Eigen::Array<double, 6, 1> xyzabc, bool front_pose, bool up_pose, bool in_rads, bool out_rads, Eigen::Array<double, 6, 1> &joint){
    
    /*
        x = xyzabc[0]
        y = xyzabc[1]
        z = xyzabc[2]
        
        if in_rads == True:
            a = xyzabc[3]
            b = xyzabc[4]
            c = xyzabc[5]
        else:
            a = rads(xyzabc[3])
            b = rads(xyzabc[4])
            c = rads(xyzabc[5])
        
        TCPxyz = np.array([x, y, z])
        R = rotz(c)@roty(b)@rotx(a)
        WP = TCPxyz - self.a6x*R[:,0]
        j1 = np.arctan2(WP[1], WP[0])
        
        
        WPxy = sqrt(WP[0]**2 + WP[1]**2)
        

        if front_pose == True:
            l = WPxy - self.a2x
        else:
            l = WPxy + self.a2x
        
        h = WP[2] - self.a1z - self.a2z
        ro = sqrt(h**2 + l**2)
        b4x = sqrt(self.a4z**2 + (self.a4x + self.a5x)**2)
        
        if ro > (self.a3z+b4x):
            print("arm cannot stretch this far")
        if ro < np.abs(self.a3z-b4x):
            print("arm cannot reach this point")
            
        alpha = np.arctan2(h,l)
        cosbeta = (ro**2 + self.a3z**2-b4x**2)/(2*ro*self.a3z)
        beta = np.arctan2(sqrt(1-cosbeta**2), cosbeta)#
        
        
        cosgamma = (self.a3z**2 +b4x**2 -ro**2)/(2*self.a3z*b4x)
        gamma = np.arctan2(sqrt(1-cosgamma**2), cosgamma)
        delta = np.arctan2(self.a4x+self.a5x,self.a4z)
        
        
        if front_pose == True:
            if up_pose == True:
                j2 = np.pi/2 - alpha - beta
                j3 = np.pi-gamma-delta
            else:
                j2 = np.pi/2 - alpha + beta
                j3 = -np.pi+gamma-delta
            

        if front_pose == False:
            j1 = j1 + np.pi
            if up_pose == True:
                j2 = -np.pi/2+ alpha+beta
                j3 = -np.pi+gamma-delta
            else:
                j2 = -np.pi/2 - beta + alpha
                j3 = np.pi - gamma - delta

        
        
        Rarm = rotz(j1)@roty(j2+j3)
        Rwrist = Rarm.T@R

        j4 = np.arctan2(Rwrist[1,0],-Rwrist[2,0])

        j5 = np.arctan2(sqrt(1-Rwrist[0,0]**2), Rwrist[0,0])
        j6 = np.arctan2(Rwrist[0,1], Rwrist[0,2])
        

    
        joint_angles = np.array([j1,j2,j3,j4,j5,j6])
        
        if out_rads == False:
            joint_angles = degs(joint_angles)
        
        return joint_angles
    */
    return ARC_CODE_OK;
}

ARCCode_t Robi::Jacobian(Eigen::Array<double, 6, 1> thetas, bool in_rads, Eigen::Matrix<double, 6, 6> &J){
    double j1, j2, j3, j4, j5, j6;
    j1 = thetas[0];
    j2 = thetas[1];
    j3 = thetas[2];
    j4 = thetas[3];
    j5 = thetas[4];
    j6 = thetas[5];
        
        
    J(0, 0) = -a2x*sin(j1) - a3z*sin(j1)*sin(j2) + a4x*(sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3)) + a4z*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a5x*(sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3)) + a6x*((-(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3))*cos(j5));
    J(0, 1) = a3z*cos(j1)*cos(j2) + a4x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a4z*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a6x*(-(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5));
    J(0, 2) = a4x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a4z*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2)) + a6x*(-(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5));
    J(0, 3) = a6x*((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*sin(j4) - sin(j1)*cos(j4))*sin(j5);
    J(0, 4) = a6x*((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*cos(j5) - (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5));
    J(0, 5) = 0;

    J(1, 0) = a2x*cos(j1) + a3z*sin(j2)*cos(j1) + a4x*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a4z*(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2)) + a5x*(-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3)) + a6x*((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5));
    J(1, 1) = a3z*sin(j1)*cos(j2) + a4x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a4z*(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a6x*(-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5));
    J(1, 2) = a4x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a4z*(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3)) + a5x*(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2)) + a6x*(-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5));
    J(1, 3) = a6x*((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*sin(j4) + cos(j1)*cos(j4))*sin(j5);
    J(1, 4) = a6x*((-(sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) + sin(j4)*cos(j1))*cos(j5) - (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5));
    J(1, 5) = 0;

    J(2, 0) = 0;
    J(2, 1) = -a3z*sin(j2) + a4x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a4z*(-sin(j2)*cos(j3) - sin(j3)*cos(j2)) + a5x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4));
    J(2, 2) = a4x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a4z*(-sin(j2)*cos(j3) - sin(j3)*cos(j2)) + a5x*(sin(j2)*sin(j3) - cos(j2)*cos(j3)) + a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4));
    J(2, 3) = -a6x*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j4)*sin(j5);
    J(2, 4) = a6x*((sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j4)*cos(j5) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5));
    J(2, 5) = 0;

    J(3, 0) = 0;
    J(3, 1) = ((((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*sin(j6) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + (-((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*cos(j6) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j4)*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    J(3, 2) = ((((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*sin(j6) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + (-((sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j4)*cos(j5))*cos(j6) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j4)*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    J(3, 3) = ((-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6)*cos(j5) + (-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j6))/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j5)*cos(j6) - (sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j6)*cos(j4))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    J(3, 4) = ((-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*sin(j6)/(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6)) - ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*cos(j6)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));
    J(3, 5) = (1 + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))*(-(sin(j2)*sin(j3) - cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6))/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*cos(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*sin(j6),2)/pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j6) + ((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))*cos(j6),2));

    J(4, 0) = 0;
    J(4, 1) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5)*cos(j4))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + 2*(sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    J(4, 2) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j5) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5)*cos(j4))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j5) + 2*(sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5)*cos(j4))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    J(4, 3) = (-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j4)*sin(j5)/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) - (sin(j2)*sin(j3) - cos(j2)*cos(j3))*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*sin(j4)*sin(j5)/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    J(4, 4) = (((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*cos(j4)*cos(j5) - (sin(j2)*cos(j3) + sin(j3)*cos(j2))*sin(j5))/sqrt(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)) + (1/2)*(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5))*(2*(sin(j2)*sin(j3) - cos(j2)*cos(j3))*cos(j4)*cos(j5) - 2*(-sin(j2)*cos(j3) - sin(j3)*cos(j2))*sin(j5))/pow(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2),(3/2)))/(1 + pow((-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)/(1 - pow(-(-sin(j2)*sin(j3) + cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j2)*cos(j3) - sin(j3)*cos(j2))*cos(j5),2)));
    J(4, 5) = 0;

    J(5, 0) = ((-(-(-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) - (sin(j1)*sin(j2)*sin(j3) - sin(j1)*cos(j2)*cos(j3))*cos(j5))*(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + ((-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    J(5, 1) = ((-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + (-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    J(5, 2) = ((-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) - (-sin(j2)*cos(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*cos(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + (-(-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5)*cos(j4) + (-sin(j1)*sin(j2)*cos(j3) - sin(j1)*sin(j3)*cos(j2))*cos(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    J(5, 3) = (-(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*sin(j4) - sin(j1)*cos(j4))*sin(j5)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + ((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*sin(j4) + cos(j1)*cos(j4))*sin(j5)/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    J(5, 4) = (((-(sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) + sin(j4)*cos(j1))*cos(j5) - (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*sin(j5))/(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5)) + (-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5))*(-(-(sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) - sin(j1)*sin(j4))*cos(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*sin(j5))/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2))/(pow(-((sin(j1)*sin(j2)*cos(j3) + sin(j1)*sin(j3)*cos(j2))*cos(j4) - sin(j4)*cos(j1))*sin(j5) + (-sin(j1)*sin(j2)*sin(j3) + sin(j1)*cos(j2)*cos(j3))*cos(j5),2)/pow(-((sin(j2)*cos(j1)*cos(j3) + sin(j3)*cos(j1)*cos(j2))*cos(j4) + sin(j1)*sin(j4))*sin(j5) + (-sin(j2)*sin(j3)*cos(j1) + cos(j1)*cos(j2)*cos(j3))*cos(j5),2) + 1);
    J(5, 5) = 0;

    return ARC_CODE_OK;
}

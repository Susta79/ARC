#include "trajectory_scurve.h"

Trajectory_scurve::Trajectory_scurve(double V, double A, double J, Path *path) : Trajectory(V, path){
    ARCCode_t ret;
    this->set_A(A);
    this->set_J(J);
    // Delta time for each segment
    this->T1 = 0.0;
    this->T2 = 0.0;
    this->T3 = 0.0;
    this->T4 = 0.0;
    this->T5 = 0.0;
    this->T6 = 0.0;
    this->T7 = 0.0;
    // Distance done in each segment
    this->D1 = 0.0;
    this->D2 = 0.0;
    this->D3 = 0.0;
    this->D4 = 0.0;
    this->D5 = 0.0;
    this->D6 = 0.0;
    this->D7 = 0.0;
    // Speed at the end of each segment
    this->V1 = 0.0;
    this->V2 = 0.0;
    this->V3 = 0.0;
    this->V4 = 0.0;
    this->V5 = 0.0;
    this->V6 = 0.0;
    this->V7 = 0.0;
    ret = this->process();
}

double Trajectory_scurve::get_dist_at_t(double t, bool out_mm){
    double dist, dt;
    double T01 = this->T1;
    double T02 = T01 + this->T2;
    double T03 = T02 + this->T3;
    double T04 = T03 + this->T4;
    double T05 = T04 + this->T5;
    double T06 = T05 + this->T6;
    double T07 = T06 + this->T7;

    if (t <= 0) {
        dist = 0.0;
    }
    if ((t > 0) && (t <= T01)) {
        dist = (1.0 / 6.0) * this->J * pow(t, 3);
    }
    if ((t > T01) && (t <= T02)) {
        dt = t - T01;
        dist = this->V1 * dt + (1.0 / 2.0) * this->A * pow(dt, 2);
        dist += this->D1;
    }
    if ((t > T02) && (t <= T03)) {
        dt = t - T02;
        dist = (this->V  - (1.0 / 2.0) * (pow(this->A, 2) / this->J)) * dt - (1.0 / 6.0) * this->J * pow(dt, 3) + (1.0 / 2.0) * this->A * pow(dt, 2);
        dist += (this->D1 + this->D2);
    }
    if ((t > T03) && (t <= T04)) {
        dt = t - T03;
        dist = this->V * dt;
        dist += (this->D1 + this->D2 + this->D3);
    }
    if ((t > T04) && (t <= T05)) {
        dt = t - T04;
        dist = this->V * dt - (1.0 / 6.0) * this->J * pow(dt, 3);
        dist += (this->D1 + this->D2 + this->D3 + this->D4);
    }
    if ((t > T05) && (t <= T06)) {
        dt = t - T05;
        dist = (this->V - this->V1) * dt - (1.0 / 2.0) * this->A * pow(dt, 2);
        dist += (this->D1 + this->D2 + this->D3 + this->D4 + this->D5);
    }
    if ((t > T06) && (t <= T07)) {
        dt = t - T06;
        dist = (1.0 / 6.0) * this->J * pow(dt, 3) - (1.0 / 2.0) * this->A * pow(dt, 2) + (1.0 / 2.0) * this->J * pow(T01, 2) * dt;
        dist += (this->D1 + this->D2 + this->D3 + this->D4 + this->D5 + this->D6);
    }
    if (t > T07) {
        dist = this->D1 + this->D2 + this->D3 + this->D4 + this->D5 + this->D6 + this->D7;
    }

    if (out_mm)
        dist *= 1000.0;

    return dist;
}

ARCCode_t Trajectory_scurve::process(){
    ARCCode_t ret;
    double L;
    // get path lenght in m
    ret = this->path->path_lenght(&L, false);
    if (ret != ARC_CODE_OK)
        return ret;
    // Segment 1: Jerk positive constant
    this->T1 = this->A / this->J;
    this->V1 = (1.0 / 2.0) * this->J * pow(T1, 2);
    this->D1 = (1.0 / 6.0) * this->J * pow(T1, 3);
    // Segment 2:
    this->T2 = (this->V / this->A) - (this->A / this->J);
    this->V2 = this->V1 + this->A * this->T2;
    this->D2 = this->V1 * this->T2 + (1.0 / 2.0) * this->A * pow(this->T2, 2);
    // Segment 3:
    this->T3 = this->A / this->J;
    this->V3 = this->V;
    this->D3 = this->V * this->T1 - (1.0 / 6.0) * this->J * pow(T1, 3);
    // Segment 5:
    this->T5 = this->T3;
    this->V5 = this->V2;
    this->D5 = this->D3;
    // Segment 6:
    this->T6 = this->T2;
    this->V6 = this->V1;
    this->D6 = this->D2;
    // Segment 7:
    this->T7 = this->T1;
    this->V7 = 0;
    this->D7 = this->D1;
    // Segment 4:
    this->D4 = L - (this->T1 + this->T2 + this->T3 + this->T5 + this->T6 + this->T7);
    this->T4 = this->D4 / this->V;
    this->V4 = this->V;
    // Total time
    this->set_T(this->T1 + this->T2 + this->T3 + this->T4 + this->T5 + this->T6 + this->T7);
    //
    qDebug() << "T1: " << this->T1;
    qDebug() << "T2: " << this->T2;
    qDebug() << "T3: " << this->T3;
    qDebug() << "T4: " << this->T4;
    qDebug() << "T5: " << this->T5;
    qDebug() << "T6: " << this->T6;
    qDebug() << "T7: " << this->T7;
    qDebug() << "D1: " << this->D1;
    qDebug() << "D2: " << this->D2;
    qDebug() << "D3: " << this->D3;
    qDebug() << "D4: " << this->D4;
    qDebug() << "D5: " << this->D5;
    qDebug() << "D6: " << this->D6;
    qDebug() << "D7: " << this->D7;
    qDebug() << "V1: " << this->V1;
    qDebug() << "V2: " << this->V2;
    qDebug() << "V3: " << this->V3;
    qDebug() << "V4: " << this->V4;
    qDebug() << "V5: " << this->V5;
    qDebug() << "V6: " << this->V6;
    qDebug() << "V7: " << this->V7;

    return ARC_CODE_OK;
}
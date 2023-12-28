#include "PIDController.h"


PIDController::PIDController()
{
    c1_ = 0.;
    c2_ = 0.;
    d0_ = 0.;
    d1_ = 0.;
    d2_ = 0.;
    prev_err_[0] = 0.;
    prev_err_[1] = 0.;
    prev_out_[0] = 0.;
    prev_out_[1] = 0.;
}

void PIDController::init(int mode, float kp, float ti, float td, float ff, float fc, float cp, bool is_active)
{
    c1_ = 0.;
    c2_ = 0.;
    d0_ = 0.;
    d1_ = 0.;
    d2_ = 0.;
    double Kp = kp;
    double Kd = Kp*td;
    double KiTs = Kp/ti*cp;
    
    if (mode == 1) {
        c1_ = 1.;
        d0_ = Kp + KiTs;
        d1_ = -Kp;
        
        
        
        is_active_ = is_active;
        }
    else if (mode == 0) {
        double Kp = kp;
        d0_ = Kp;
        
        is_active_ = is_active;
    }
    else if (mode == 2) {
        double Kp = kp;
        double KdN = Kp*td*fc;
        double _1_NTs1 = 1 / (1+fc*cp);
        c1_ = _1_NTs1;
        d0_ = Kp + KdN*_1_NTs1;
        d1_ = -(Kp + KdN)*_1_NTs1;
        
        is_active_ = is_active;
    }
    else if (mode == 3) {
        double ki = kp / ti;
        double kd = kp * td;
//        double c1 =  2+fc*cp;
//        double c2 = -2+fc*cp;
//        c1_ = (c1-c2)/c1;
//        c2_ = c2/c1;
//        d0_ = kp+0.5*ki*cp+2*kd*fc/c1;
//        d1_ = kp*(c2-c1)/c1+0.5*ki*cp*(c1+c2)/c1-4*kd*fc/c1;
//        d2_ = -kp*c2/c1+0.5*ki*cp*c2/c1+2*kd*fc/c1;
        
        double c1 = (4/cp/cp)*(kp+kd*fc);
        double c2 = 2/cp*(kp*fc+ki);
        double c3 = ki*fc;
        double c4 = 4/cp/cp+2*fc/cp;
        double c5 = -8/cp/cp;
        double c6 = 4/cp/cp-2*fc/cp;
        
        d0_ = (c1+c2+c3)/c4;
        d1_ = (-2*c1+2*c3)/c4;
        d2_ = (c1-c2+c3)/c4;
        c1_ = c5/c4;
        c2_ = c6/c4;
        prev_err_[0] = 0.;
        prev_err_[1] = 0.;
        prev_out_[0] = 0.;
        prev_out_[1] = 0.;
        is_active_ = is_active;
    }
}

double PIDController::clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}
double PIDController::compute_action(double target, double feedback, float ff)
{
    double err = target - feedback;
    double out = 0.;
    
    if(is_active_)
    {
        out = c1_ * prev_out_[0] + c2_ * prev_out_[1] + d0_ * err + d1_ * prev_err_[0] + d2_ * prev_err_[1];
        out = clamp(out, -1., 1.);
    }

    prev_err_[1] = prev_err_[0];
    prev_err_[0] = err;
    prev_out_[1] = prev_out_[0];
    prev_out_[0] = out;

    if(is_active_) {
        if (target == 0) {
            return 0.0;
        }
        else return clamp(out + ff * target, -1., 1.);
        }
    else { 
        return 0.0;
    }
}

void PIDController::setActive(bool command_active) {
    is_active_ = command_active;
}
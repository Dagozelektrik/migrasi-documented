#include "MotorDAGOZ.h"



MotorDagoz :: MotorDagoz (int period, PinName directionCW, PinName directionCCW, PinName pwmpin) : _period(period), _directionCW(directionCW), _directionCCW(directionCCW), _pwmpin(pwmpin)
{
    _pwmpin.period_us(_period);  
}

void MotorDagoz::setpwm(float inputpwm )
{
    if ( inputpwm >= 0 )
    {
        _directionCW = 1;
        _directionCCW = 0;
        _pwmpin.write(inputpwm);
    }
    else
    {
        _directionCW = 0;
        _directionCCW = 1;
        _pwmpin.write(-1*inputpwm);
    }   
}

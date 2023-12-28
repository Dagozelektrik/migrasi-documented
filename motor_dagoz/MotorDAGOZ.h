 #ifndef MOTOR_DAGOZ_H
#define MOTOR_DAGOZ_H

#include "mbed.h"

// mbed::DigitalOut* _directionCCW;
class MotorDagoz
{
public :
    // DigitalOut* _directionCW;
    // _directionCW = new DigitalOut(directionCW);
    MotorDagoz(int period, PinName directionCW, PinName directionCCW, PinName pwmpin);
    
    void setpwm(float inputpwm);
        
private :
    PwmOut _pwmpin;
    int _period;
    // _directionCCW = new DigitalOut(directionCCW);
    DigitalOut _directionCW;
    DigitalOut _directionCCW;
};
#endif

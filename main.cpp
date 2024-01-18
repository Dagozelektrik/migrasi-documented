#include "mbed.h"
#include "math.h"
#include <cstdint>
#include <vector>
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dgz_msgs/StampedHardwareCommand.h>
#include <dgz_msgs/StampedHardwareState.h>
#include <std_srvs/SetBool.h>
#include "PIDController.h"
#include "Clock.h"

//#include "rtos.h"

// For PID Param Logging Purpose (sprintf)
// #include <string>

/*****************************
        ROS node handle 
 *****************************/
ros::NodeHandle nh;
/******************************
  Publisher-Subscriber
******************************/
dgz_msgs::StampedHardwareState stateMsg;

ros::Publisher statePub("/nucleo/state/hardware", &stateMsg);
void commandCallback(const dgz_msgs::StampedHardwareCommand &commandMsg);
ros::Subscriber<dgz_msgs::StampedHardwareCommand> commandSub("/control/command/hardware", &commandCallback);
//Semaphore xSemaphorePulseData(4);

/******************************
         Service
******************************/
ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> client("control/state/notify_kick");


Mutex mutex_acc;
ConditionVariable cv(mutex_acc);
Mutex mutex_controlCalculation;
ConditionVariable cvCc(mutex_controlCalculation);
Mutex mutex_compass;
ConditionVariable cvCompass(mutex_compass);
Mutex mutex_main;
ConditionVariable cvMain(mutex_main);

//thread for RTOS
Thread thread1(osPriorityNormal1);
Thread thread2(osPriorityAboveNormal);
Thread thread3(osPriorityNormal);
Thread thread4(osPriorityRealtime7);
Thread thread5(osPriorityNormal);


// initialize controllers
PIDController ControllerFR = PIDController();
PIDController ControllerFL = PIDController();
PIDController ControllerBR = PIDController();
PIDController ControllerBL = PIDController();

//primitive function
void mainProcess();
void kickTarget();
void getCompass();
void controlCalculation();
void moveLever();
void publishMessage();
void moveDribbler();
void initPIDController();
void assignPIDParam();
void setBaseActive(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

// 
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> set_base_active_srv("/nucleo/command/set_base_active", &setBaseActive);
/*****************************
  Main Function
 *****************************/

int main()
{
    //init ros advertise and subscribe
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    ThisThread::sleep_for(4ms); // wait until nucleo node is set
    nh.advertise(statePub);
    nh.advertiseService(set_base_active_srv);
    nh.subscribe(commandSub);
    
    // Service Client for Kicker
    nh.serviceClient(client);
    initPIDController();

    t.start();

    //kicker mode
    kickerServo.calibrate(range, 0.0);
    kickerServo = position;
    //kicker setup
    kicker.period(0.01f);
    kicker = 0; //deactivate kicker, active HIGH

    thread1.start(mainProcess);
    thread2.start(getCompass);
    thread3.start(controlCalculation);
    thread4.start(kickTarget);
    thread5.start(publishMessage);

    while (true)
    {
        //do nothing
    }
}

void setBaseActive(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ControllerFL.setActive(req.data);
    ControllerFR.setActive(req.data);
    ControllerBL.setActive(req.data);
    ControllerBR.setActive(req.data);
    res.success = true;
    res.message = "success";
}
void initPIDController()
{
    assignPIDParam();
    ControllerFR.init(PIDMode, kpFR, tiFR, tdFR, ffFR, fcFR, cp, true);
    ControllerFL.init(PIDMode, kpFL, tiFL, tdFL, ffFL, fcFL, cp, true);
    ControllerBR.init(PIDMode, kpBR, tiBR, tdBR, ffBR, fcBR, cp, true);
    ControllerBL.init(PIDMode, kpBL, tiBL, tdBL, ffBL, fcBL, cp, true);
}
void assignPIDParam()
{
    while (!nh.getParam("/control/base/motors_pid/front_left/mode", &PIDMode, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/front_left/proportional", &kpFL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/front_left/feedforward", &ffFL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/front_left/integral_time", &tiFL, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/front_left/derivative_time", &tdFL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/front_right/proportional", &kpFR, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/front_right/feedforward", &ffFR, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/front_right/integral_time", &tiFR, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/front_right/derivative_time", &tdFR, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_left/proportional", &kpBL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_left/feedforward", &ffBL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_left/integral_time", &tiBL, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/back_left/derivative_time", &tdBL, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_right/proportional", &kpBR, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_right/feedforward", &ffBR, 1, 10000));

    while (!nh.getParam("/control/base/motors_pid/back_right/integral_time", &tiBR, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/back_right/derivative_time", &tdBR, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/front_left/filter_coefficient", &fcFL, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/front_right/filter_coefficient", &fcFR, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/back_left/filter_coefficient", &fcBL, 1, 10000));
    
    while (!nh.getParam("/control/base/motors_pid/back_right/filter_coefficient", &fcBR, 1, 10000));
        
    char pass_param[20];
    snprintf(pass_param, 20, "PIDMode : %d", PIDMode);
    nh.loginfo(pass_param);
    
    snprintf(pass_param, 20, "kpFL : %f", kpFL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "kpFR : %f", kpFR);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "kpBL : %f", kpBL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "kpBR : %f", kpBR);
    nh.loginfo(pass_param);
    
    snprintf(pass_param, 20, "tiFL : %f", tiFL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tiFR : %f", tiFR);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tiBL : %f", tiBL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tiBR : %f", tiBR);
    nh.loginfo(pass_param);
    
    snprintf(pass_param, 20, "tdFL : %f", tdFL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tdFR : %f", tdFR);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tdBL : %f", tdBL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "tdBR : %f", tdBR);
    nh.loginfo(pass_param);
    
    snprintf(pass_param, 20, "ffFL : %f", ffFL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "ffFR : %f", ffFR);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "ffBL : %f", ffBL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "ffBR : %f", ffBR);
    nh.loginfo(pass_param);
    
    snprintf(pass_param, 20, "fcFL : %f", fcFL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "fcFR : %f", fcFR);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "fcBL : %f", fcBL);
    nh.loginfo(pass_param);
    snprintf(pass_param, 20, "fcBR : %f", fcBR);
    nh.loginfo(pass_param);
    
    //ppr
    while (!nh.getParam("/control/base/motors/pulses_per_revolution_1", &WHEEL_PPR_1, 1, 10000));
    snprintf(pass_param, 20, "PPR_1 : %lf", WHEEL_PPR_1);
    nh.loginfo(pass_param);
    while (!nh.getParam("/control/base/motors/pulses_per_revolution_2", &WHEEL_PPR_2, 1, 10000));
    snprintf(pass_param, 20, "PPR_2 : %lf", WHEEL_PPR_2);
    nh.loginfo(pass_param);
    while (!nh.getParam("/control/base/motors/pulses_per_revolution_3", &WHEEL_PPR_3, 1, 10000));
    snprintf(pass_param, 20, "PPR_3 : %lf", WHEEL_PPR_3);
    nh.loginfo(pass_param);
    while (!nh.getParam("/control/base/motors/pulses_per_revolution_4", &WHEEL_PPR_4, 1, 10000));
    snprintf(pass_param, 20, "PPR_4 : %lf", WHEEL_PPR_4);
    nh.loginfo(pass_param);
    intEncFL.setPPR(WHEEL_PPR_1);
    intEncFR.setPPR(WHEEL_PPR_2);
    intEncBL.setPPR(WHEEL_PPR_3);
    intEncBR.setPPR(WHEEL_PPR_4);
}

void mainProcess()
{
//    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    ThisThread::sleep_for(1s);

    while (1)
    {
        if (nh.connected() && dc) {
            initPIDController();
            dc = 0;
        }
        else if (!nh.connected()) {
            dc = 1;   
        }

//        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;

        //ball distance from IR
        ball_distance = infraRed.read();
        // ball_distance = 0.9;

        //cur_dribbler_L = -dribblerEncL.GetCounter(1);
        //cur_dribbler_R = -dribblerEncR.GetCounter(1);

        moveDribbler();
        moveLever();

        // publishMessage();

        if (get_ms_count() - last_timer >= 1000)
        {

            locomotion_FL_target_vel = 0;
            locomotion_FR_target_vel = 0;
            locomotion_BL_target_vel = 0;
            locomotion_BR_target_vel = 0;

            dribbler_L_target_rate = 0;
            dribbler_R_target_rate = 0;

            kick_power_target = 0;
            kicker_shoot_mode = 0;
        }
        ThisThread::sleep_for(30ms);
        nh.spinOnce();
    }
}

// PID Calculation to generate PWM
void controlCalculation()
{
    int counter = 1;

    while (1)
    {
        // Get Internal Encoder Pulses
        mutex_controlCalculation.lock();
        Kernel::Clock::time_point end_time = Kernel::Clock::now() + 10ms;

        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
        
        //read encoder
        temp_cur_locomotion_R = locomotionEncR.GetCounter(1);
        temp_cur_locomotion_L = locomotionEncL.GetCounter(1);
        temp_cur_locomotion_B = locomotionEncB.GetCounter(1);

//        xSemaphorePulseData.wait();
        motorPulseFL += rotInFL;
        motorPulseFR += rotInFR;
        motorPulseBL += rotInBL;
        motorPulseBR += rotInBR; 
        
        cur_locomotion_L -= temp_cur_locomotion_L;
        cur_locomotion_R += temp_cur_locomotion_R;
        cur_locomotion_B -= temp_cur_locomotion_B;

//        xSemaphorePulseData.release();

        // Calculate Feedback Velocity from pulses
        locomotion_FL_vel = rotInFL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_1 * CONTROL_COMPUTE_PERIOD);
        locomotion_FR_vel = rotInFR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_2 * CONTROL_COMPUTE_PERIOD);
        locomotion_BL_vel = rotInBL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_3 * CONTROL_COMPUTE_PERIOD);
        locomotion_BR_vel = rotInBR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR_4 * CONTROL_COMPUTE_PERIOD);

        // Compute action drom PIDController to determine PWM
        locomotion_FR_target_rate = ControllerFR.compute_action(locomotion_FR_target_vel, locomotion_FR_vel, ffFR);
        locomotion_FL_target_rate = ControllerFL.compute_action(locomotion_FL_target_vel, locomotion_FL_vel, ffFL);
        locomotion_BR_target_rate = ControllerBR.compute_action(locomotion_BR_target_vel, locomotion_BR_vel, ffBR);
        locomotion_BL_target_rate = ControllerBL.compute_action(locomotion_BL_target_vel, locomotion_BL_vel, ffBL);

        // Execute PWM value to certain pin
        locomotionMotorFL.setpwm(locomotion_FL_target_rate);
        locomotionMotorFR.setpwm(locomotion_FR_target_rate);
        locomotionMotorBL.setpwm(locomotion_BL_target_rate);
        locomotionMotorBR.setpwm(locomotion_BR_target_rate);

        snprintf(pass_param,20,)

        // char pass_param[20];
        // snprintf(pass_param, 20, "motor_1 : %d", motorPulseFL);
        // nh.loginfo(pass_param);
 

        while (1){;
            if (cvCc.wait_until(end_time) == cv_status::timeout) {
                break;               
            }
        }

        if (counter%4==0){
            mutex_acc.lock();
            // nh.loginfo("---batas suci---");
            cv.notify_all();
            mutex_acc.unlock();
            counter = 0;
        } 
        counter++;

        mutex_controlCalculation.unlock();

        // ThisThread::sleep_for(5ms);
    }
}

void moveDribbler()
{
    dribblerMotorL.setpwm(dribbler_L_target_rate);
    dribblerMotorR.setpwm(-dribbler_R_target_rate);
}

void moveLever()
{

    if (kicker_shoot_mode==0)
    {
    // case 0:
        position = 0.06; /* Laplace */
        // position = 0.15; /* Kirchhoff */
        // break;
    // case 1:
    }
    else{
        position = 0.32; /* Laplace */
        // position = 0.4; /* Kirchhoff */
        // break;
    }

    kickerServo.calibrate(range, 0.0);
    kickerServo.write(position);
}

void publishMessage()

{
    char pass_param[50];
    Timer t_check;
    while(1){
        // t_check.start();

        mutex_acc.lock();
        Kernel::Clock::time_point end_time = Kernel::Clock::now() + 20ms;
        // Publish position data//
        stateMsg.data.base_motor_1_pulse_delta = motorPulseFL;
        stateMsg.data.base_motor_2_pulse_delta = motorPulseFR;
        stateMsg.data.base_motor_3_pulse_delta = motorPulseBL;
        stateMsg.data.base_motor_4_pulse_delta = motorPulseBR;

        // nh.loginfo("batas kotor");
        // snprintf(pass_param, 20, "Accumulated: %d", stateMsg.data.base_motor_1_pulse_delta);
        // nh.loginfo(pass_param);

        stateMsg.data.base_encoder_1_pulse_delta = cur_locomotion_L;
        stateMsg.data.base_encoder_2_pulse_delta = cur_locomotion_R;
        stateMsg.data.base_encoder_3_pulse_delta = cur_locomotion_B;

        stateMsg.data.dribbler_motor_l_pulse_delta = cur_dribbler_L;
        stateMsg.data.dribbler_motor_r_pulse_delta = cur_dribbler_R;

        stateMsg.data.dribbler_potentio_l_reading = 0.0;
        stateMsg.data.dribbler_potentio_r_reading = 0.0;

        stateMsg.data.ir_reading = ball_distance;
        stateMsg.data.compass_reading = theta_result;

        stateMsg.header.stamp = nh.now();

        statePub.publish(&stateMsg);

        // Reset pulse delta value for distinguished control thread frequency
        //xSemaphorePulseData.wait();
        motorPulseFL = 0;
        motorPulseFR = 0;
        motorPulseBR = 0;
        motorPulseBL = 0;
        cur_locomotion_L = 0;
        cur_locomotion_R = 0;
        cur_locomotion_B = 0;
        //xSemaphorePulseData.release();
        // nh.loginfo("last");

        cv.wait();
        // t_check.stop();
        // snprintf(pass_param, 75, "The time taken was %llu milliseconds\n", std::chrono::duration_cast<std::chrono::milliseconds>(t_check.elapsed_time()).count());

        while (1){
            if (cv.wait_until(end_time) == cv_status::timeout) {
                break;               
            }
        }
        // snprintf(pass_param, 50, "The time taken was %d", t_check.read_ms());
        // t_check.reset();
        // nh.loginfo(pass_param);
        mutex_acc.unlock();

        // ThisThread::sleep_for(20ms);

    }

}

void commandCallback(const dgz_msgs::StampedHardwareCommand &commandMsg)
{

    // Temporary, base_motor_x_pwm should be modified by new commandMsg (branch master)
    locomotion_FL_target_vel = commandMsg.data.base_motor_1_pwm;
    locomotion_FR_target_vel = commandMsg.data.base_motor_2_pwm;
    locomotion_BL_target_vel = commandMsg.data.base_motor_3_pwm;
    locomotion_BR_target_vel = commandMsg.data.base_motor_4_pwm;

    dribbler_L_target_rate = commandMsg.data.dribbler_motor_l_pwm;
    dribbler_R_target_rate = commandMsg.data.dribbler_motor_r_pwm;

    kick_power_target = commandMsg.data.kicker_pwm;
    kicker_shoot_mode = commandMsg.data.is_shoot;
    
    base_active = commandMsg.data.base_active;
    
    ControllerFL.setActive(base_active);
    ControllerFR.setActive(base_active);
    ControllerBL.setActive(base_active);
    ControllerBR.setActive(base_active);

    last_timer = get_ms_count();
}

//new function
void kickTarget(){
    while(1){
        nh.spinOnce();
        // pc.printf("Function Called\n");
        if (get_ms_count()-time_last_kick > kicker_ready_time && kick_power_target != 0)
        {
            kicker.write(kick_power_target);
            kick_power_target = 0;
            time_last_kick = get_ms_count();
            std_srvs::SetBool::Request req;
            std_srvs::SetBool::Response res;
            req.data = true;
            client.call(req, res);
            
            // pc.printf("Kicked\n");
        }
        else {
            kicker.write(0);
            kick_power_target = 0;
        }
        ThisThread::sleep_for(20ms);
        kicker.write(0);
        kick_power_target = 0;
        ThisThread::sleep_for(30ms);
    }
        
}

//cmps
void getCompass(){
   float theta0 = compass.readBearing()/10.0;
   ThisThread::sleep_for(1000ms);
   theta0 = compass.readBearing()/10.0;
   int compassLed = 0;
   nh.spinOnce();
   float theta_prev = theta0;
   
//    pc.printf("Theta : %.2f\n", theta0);
   
   float theta = 0;
   
   while(1){
       nh.spinOnce();
       
       float theta_temp = (compass.readBearing()/10.0) - theta0;
       
       if(theta_temp > 180.0 && theta_temp <= 360.0)
           theta_com = (theta_temp - 360.0)/-1.0;    
       else if(theta_temp < -180.0 && theta_temp >= -360.0)
           theta_com = (theta_temp + 360.0)/-1.0;
       else
           theta_com = theta_temp/-1.0;
       
       theta_result = theta_com - theta_prev;
       if (theta_result > 180) theta_result -= 360;
       else if (theta_result < -180) theta_result += 360;

       theta_prev = theta_com;

       if(fabs(theta) >= 10.0/RADTODEG) compassLed = 1;
       else compassLed = 0;
    //    Thread::wait(20);
    //    kicker.write(0);
    //    kick_power_target = 0;
       ThisThread::sleep_for(30ms);
   }
}
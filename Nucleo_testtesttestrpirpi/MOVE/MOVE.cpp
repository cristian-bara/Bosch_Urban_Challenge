#include "mbed.h"
#include "MOVE.h"
//#include <array.h>

MOVE::MOVE(PinName _pwm_servo, PinName _pwm_driver, PinName _ina_driver, PinName _inb_driver, PinName _current_driver):
servo(_pwm_servo),
vnh(_pwm_driver, _ina_driver, _inb_driver, _current_driver)
{
};

MOVE::~MOVE()
{
};

void MOVE::Steer(float angle)
{
    if ((angle <= 25) || (angle >= -25))
        servo.SetAngle(angle);
    
};

void MOVE::Speed(float speed)
{
    speed /=100;
    vnh.Run(speed);
};

void MOVE::TestCar()
{
        Steer(20);
        wait(1);
        Steer(0);
        wait(1);
        Steer(-20);
        wait(1);
        ResetCar();
        wait(1);
        Speed(25);
        wait(1);
        Speed(0);
        wait(1);
        Speed(-25);
        wait(1);
        ResetCar();
        wait(1);
};

void MOVE::ResetCar()
{
    Steer(0);
    Speed(0);
};
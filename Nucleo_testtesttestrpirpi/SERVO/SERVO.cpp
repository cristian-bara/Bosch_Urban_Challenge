#include "mbed.h"
#include "SERVO.h"

SERVO::SERVO(PinName _pwm):
pwm(_pwm)
{
    pwm.period_ms(20);    
    pwm.write(0.075);
};

SERVO::~SERVO()
{
};

void SERVO::SetAngle(float angle)
{
    pwm.write(conversion(angle));
};

float SERVO::conversion(float angle)
{
    return (0.05 + (angle + 25) / 1000);
};
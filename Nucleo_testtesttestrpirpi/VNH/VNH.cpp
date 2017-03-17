
#include "VNH.h"

VNH::VNH(PinName _pwm, PinName _ina, PinName _inb, PinName _current):
pwm(_pwm),
ina(_ina),
inb(_inb),
current(_current)
{  
    pwm.period_us(200);   
    increment = 0.01;
    current_speed = 0;
};

VNH::~VNH()
{
};

void VNH::Start(void)
{
    ina = !inb;
    pwm.write(0);
};

void VNH::Stop(void)
{
    ina = inb;
};

void VNH::Run(float speed)
{
    Go(speed);
};

float VNH::GetCurrent(void)
{
    return (current.read()) * 5 / 0.14;
};

void VNH::Go(float speed)
{
    //pwm.write(abs(speed));
    if (speed < 0)
    {
        ina = 0;
        inb = 1;
    }
    else
    {
        ina = 1;
        inb = 0;
    } 
    pwm = speed;    
};
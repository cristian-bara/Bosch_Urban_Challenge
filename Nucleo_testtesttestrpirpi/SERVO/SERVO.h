#ifndef _SERVO_
#define _SERVO_

class SERVO
{
public:
    SERVO(PinName _pwm);
    ~SERVO();
    void SetAngle(float angle); //-25 to 25 degr

private:
    float conversion(float angle); //angle to duty cycle
    PwmOut pwm;
    float current_angle;
};

#endif
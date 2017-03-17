#ifndef _MOVE_
#define _MOVE_
#include "SERVO.h"
#include "VNH.h"

class MOVE
{
public:
    MOVE(PinName, PinName, PinName, PinName, PinName);//A0
    ~MOVE();
    void Steer(float angle);// -25 to + 25 degrees, - (left), + (right)
    void Speed(float speed);//-100 to + 100 -(back), + (front)    
    void TestCar();
    void ResetCar();
    VNH& getVNH() { return vnh; }
            
private:
    SERVO       servo;
    VNH         vnh;
};

#endif
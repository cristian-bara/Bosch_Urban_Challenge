#ifndef MOTION_CONTROLLER_H 
#define MOTION_CONTROLLER_H 

#include <mbed.h>
#include <TaskManager.h>
#include <IMU.h>
#include <MOVE.h>
#include <SplineInterpreter.h>
#include <BezierMotionPlanner.h>

class CMotionController : public CTask
{
public:
    CMotionController(uint32_t f_period, Serial& f_serialPort, CIMU& f_imu, MOVE& f_car,CSplineInterpreter& f_splineInterpreter,float f_period_sec) : CTask(f_period)
    , m_serialPort(f_serialPort)
    , m_imu(f_imu)
    , m_car(f_car)
    , m_speed()
    , m_angle()
    , m_period_sec(f_period_sec)
    , m_isSplineActivated(false)
    , m_splineinterpreter(f_splineInterpreter)
    , m_motionPlanner()
    {

    }
    static void staticSerialCallback(void* obj,char const * a, char * b)
    {
        CMotionController* self = static_cast<CMotionController*>(obj);
        self->serialCallback(a,b);
    }
    void reset()
    {   
        m_speed = 0;
        m_angle = 0;
    }
    float getSpeed() 
    {
        return m_speed;
    }
    float getAngle() 
    {
        return m_angle;
    }
private:
    virtual void _run();
    
    void serialCallback(char const * a, char * b)
    {
        float l_speed;
        float l_angle;
        uint32_t l_res = sscanf(a,"%f;%f",&l_speed,&l_angle);
        if (2 == l_res)
        {
            m_speed = l_speed;
            m_angle = l_angle; 
            m_isSplineActivated=false;
            sprintf(b,"ack;;");
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }
    Serial& m_serialPort;
    CIMU& m_imu;
    MOVE& m_car;
    float m_speed;
    float m_angle;

    float m_period_sec;
    bool m_isSplineActivated;
    CSplineInterpreter& m_splineinterpreter;
    CBezierMotionPlanner m_motionPlanner;
};

#endif // MOTION_CONTROLLER_H 

#ifndef MOTION_CONTROLLER_H 
#define MOTION_CONTROLLER_H 

#include <mbed.h>
#include <TaskManager.h>
#include <IMU.h>
#include <MOVE.h>

class CMotionController : public CTask
{
public:
    CMotionController(uint32_t f_period, Serial& f_serialPort, CIMU& f_imu, MOVE& f_car) : CTask(f_period)
    , m_serialPort(f_serialPort)
    , m_imu(f_imu)
    , m_car(f_car)
    , m_speed()
    , m_angle()
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
    virtual void _run()
    {
        m_car.Steer(m_angle);
        m_car.Speed(m_speed);
        CIMU::CIMUData l_imuData = m_imu.getRawIMUData();        
        m_serialPort.printf("@MCTL:%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;;\r\n"
            ,m_speed
            ,m_angle
            ,m_car.getVNH().GetCurrent()
            ,l_imuData.ax
            ,l_imuData.ay
            ,l_imuData.az
            ,l_imuData.mx
            ,l_imuData.my
            ,l_imuData.mz
            ,l_imuData.gx
            ,l_imuData.gy
            ,l_imuData.gz
            );
    }
    void serialCallback(char const * a, char * b)
    {
        float l_speed;
        float l_angle;
        uint32_t l_res = sscanf(a,"%f;%f",&l_speed,&l_angle);
        if (2 == l_res)
        {
            m_speed = l_speed;
            m_angle = l_angle; 
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
};

#endif // MOTION_CONTROLLER_H 

#ifndef _IMU_
#define _IMU_

#include <TaskManager.h>
#include <L3GD20H.h>
#include <LSM303D.h>

class CIMU : public CTask
{
public:

    typedef struct 
    {
        float  ax, ay, az; //accelerometer data
        float  mx, my, mz; //magnetometer data
        float  gx, gy, gz; //gyroscope data
    } CIMUData;

    CIMU(uint32_t f_period, LSM303D& f_acmg, L3GD20H& f_gyro):CTask(f_period),m_acmg(f_acmg),m_gyro(f_gyro),m_dataAvailable(false) {}
    bool newDataAvailable() {return m_dataAvailable;}
    inline CIMUData getRawIMUData()
    {
        CIMUData l_imuData;
        l_imuData.ax = m_ax;
        l_imuData.ay = m_ay;
        l_imuData.az = m_az;
        l_imuData.mx = m_mx;
        l_imuData.my = m_my;
        l_imuData.mz = m_mz;
        l_imuData.gx = m_gyr[0]*1.0f;
        l_imuData.gx = m_gyr[1]*1.0f;
        l_imuData.gx = m_gyr[2]*1.0f;

        m_dataAvailable = false;
        return l_imuData;
    }
private:
    virtual void _run()
    {
        m_acmg.read(&m_ax, &m_ay, &m_az, &m_mx, &m_my, &m_mz);
        m_gyro.read(m_gyr);

        m_dataAvailable = true;
    }


    LSM303D& m_acmg;
    L3GD20H& m_gyro;
    bool m_dataAvailable;
    float  m_ax, m_ay, m_az; //accelerometer data
    float  m_mx, m_my, m_mz; //magnetometer data
    short  m_gyr[3];     //gyroscope data
};

#endif
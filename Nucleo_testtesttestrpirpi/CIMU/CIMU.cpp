#include "mbed.h"
#include "LSM303D.h"
#include "L3GD20H.h"
#include "CIMU.h"

Cimu::Cimu(PinName scl, PinName sda) : 
gyro(scl, sda),
acmg(scl, sda)
{
    m_cal_ax   = 0;
    m_cal_ay   = 0;
    m_cal_az   = 0;
    m_cal_mx   = 0;
    m_cal_my   = 0;
    m_cal_mz   = 0;
    m_cal_g[0] = 0;
    m_cal_g[1] = 0;
    m_cal_g[2] = 0;
}

float Cimu::get_ax(void)
{
    m_ax -= m_cal_ax;
    return m_ax;
}
 
float Cimu::get_ay(void)
{
    m_ay -= m_cal_ay;
    return m_ay;
}

float Cimu::get_az(void)
{
    m_az -= m_cal_az;
    return m_az;
}

float Cimu::get_mx(void)
{
    m_mx -= m_cal_mx;
    return m_mx;
}

float Cimu::get_my(void)
{
    m_my -= m_cal_my;
    return m_my;
}

float Cimu::get_mz(void)
{
    m_mz -= m_cal_mz;
    return m_mz;
}

float Cimu::get_gx(void)
{
    m_g[0] -= m_cal_g[0];
    return (float)m_g[0];
}

float Cimu::get_gy(void)
{
    m_g[1] -= m_cal_g[1];
    return (float)m_g[1];
}

float Cimu::get_gz(void)
{
    m_g[2] -= m_cal_g[2];
    return (float)m_g[2];
}

void Cimu::calibration(void)
{    
    m_cal_ax = m_ax;
    m_cal_ay = m_ay;
    m_cal_az = m_az;
    m_cal_mx = m_mx;
    m_cal_my = m_my;
    m_cal_mz = m_mz;
    m_cal_g[0] = m_g[0];
    m_cal_g[1] = m_g[1];
    m_cal_g[2] = m_g[2];
}

bool Cimu::update_acmg(void)
{
    //update acmg
    return (acmg.read(&m_ax, &m_ay, &m_az, &m_mx, &m_my, &m_mz));
}

bool Cimu::update_gyro(void)
{
    //update gyro
    return (gyro.read(m_g));
}

bool Cimu::update_all(void)
{
    return (update_acmg() && update_gyro());
}
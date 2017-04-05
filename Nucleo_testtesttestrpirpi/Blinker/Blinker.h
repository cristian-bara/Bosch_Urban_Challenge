#ifndef BLINKER_H
#define BLINKER_H

#include <mbed.h>
#include <TaskManager.h>

class CBlinker : public CTask
{
public:
    CBlinker(uint32_t f_period, DigitalOut f_led) : CTask(f_period), m_led(f_led) {m_led = 1;}
private:
    virtual void _run()
    {
        m_led = !m_led;
    }
    DigitalOut m_led;    
};

#endif
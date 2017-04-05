#ifndef ECHOER_H
#define ECHOER_H

#include <mbed.h>
#include <TaskManager.h>

class CEchoer : public CTask
{
public:
    CEchoer(uint32_t f_period, Serial& f_serialPort) : CTask(f_period), m_serialPort(f_serialPort){}
private:
    virtual void _run()
    {
        m_serialPort.printf(".\n\r");
    }   
    Serial& m_serialPort; 
};

#endif
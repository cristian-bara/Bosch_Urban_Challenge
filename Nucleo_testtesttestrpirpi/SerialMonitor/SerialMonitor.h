#ifndef SERIAL_MONITOR_H 
#define SERIAL_MONITOR_H 

#include <mbed.h>
#include <map>
#include <array>
#include <TaskManager.h>
#include <Queue.h>
#include <functional>

class CSerialMonitor : public CTask
{
public:
    typedef mbed::Callback<void(char const *, char *)> FCallback;
    typedef std::map<string,FCallback> CSerialSubscriberMap;

    CSerialMonitor(Serial& f_serialPort
        , CSerialSubscriberMap f_serialSubscriberMap
        ) 
        : CTask(0)
        , m_serialPort(f_serialPort)
        , m_RxBuffer()
        , m_TxBuffer()
        , m_parseBuffer()
        , m_parseIt(m_parseBuffer.begin())
        , m_serialSubscriberMap(f_serialSubscriberMap) 
        {
            m_serialPort.attach(mbed::callback(&CSerialMonitor::RxCallback, this), Serial::RxIrq);
            m_serialPort.attach(mbed::callback(&CSerialMonitor::TxCallback, this), Serial::TxIrq);
        }
private:
    static void RxCallback(void *thisPointer)
    {
        CSerialMonitor* self = static_cast<CSerialMonitor*>(thisPointer);
        self->serialRxCallback(); 
    }
    static void TxCallback(void *thisPointer)
    {
        CSerialMonitor* self = static_cast<CSerialMonitor*>(thisPointer);
        self->serialTxCallback(); 
    }
    void serialRxCallback()
    {
        __disable_irq();
        while ((m_serialPort.readable()) && (!m_RxBuffer.isFull())) {
            char l_c = m_serialPort.getc();
            m_RxBuffer.push(l_c);
        }
        __enable_irq();
        return;
    }
    void serialTxCallback()
    {
        __disable_irq();
        while ((m_serialPort.writeable()) && (!m_TxBuffer.isEmpty())) {
            m_serialPort.putc(m_TxBuffer.pop());
        }
        __enable_irq();
        return;
    }

    virtual void _run()
    {
        if ((!m_RxBuffer.isEmpty()) && (!m_TxBuffer.isFull()))
        {
            char l_c = m_RxBuffer.pop();
            m_serialPort.printf("%c",l_c);
            if ('#' == l_c)
            {
                m_parseIt = m_parseBuffer.begin();
                m_parseIt[0] = l_c;
                m_parseIt++;
                return;
            }
            if (m_parseIt != m_parseBuffer.end())
            {
                if (l_c == '\n')
                {
                    if ((';' == m_parseIt[-3]) && (';' == m_parseIt[-2]) && ('\r' == m_parseIt[-1]))
                    {
                        char msgID[5];
                        char msg[256];

                        uint32_t res = sscanf(m_parseBuffer.data(),"#%4s:%s;;",msgID,msg);
                        if (res == 2)
                        {
                            auto l_pair = m_serialSubscriberMap.find(msgID);
                            if (l_pair != m_serialSubscriberMap.end())
                            {
                                char l_resp[256] = "no response given";
                                string s(l_resp);
                                l_pair->second(msg,l_resp);
                                m_serialPort.printf("@%s:%s\r\n",msgID,l_resp);
                            }
                        }
                        m_parseIt = m_parseBuffer.begin();
                    }
                }
                m_parseIt[0] = l_c;
                m_parseIt++;
                return;
            }
        }
    }
    Serial& m_serialPort;
    CQueue<char,255> m_RxBuffer;
    CQueue<char,255> m_TxBuffer;
    array<char,256> m_parseBuffer;
    array<char,256>::iterator m_parseIt;
    CSerialSubscriberMap m_serialSubscriberMap;
};

#endif // SERIAL_MONITOR_H 

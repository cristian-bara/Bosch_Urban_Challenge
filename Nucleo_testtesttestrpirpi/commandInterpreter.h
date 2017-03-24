#ifndef COMMAND_INTERPRETER_H
#define COMMAND_INTERPRETER_H

#include "mbed.h"
#include "MOVE.h"

class CCommandInterpreter
{
public:
    CCommandInterpreter(MOVE& f_car) : m_buffer(), m_car(f_car), m_commandValue() {}
    inline void interpretChar(unsigned char f_c)
    {
        if ((f_c == 'A') || (f_c == 'S'))
        {
//            led1 = 1;
            reset();
            m_buffer.push(f_c);
            m_commandID = f_c;
//            char s[100];
//            unsigned int  l = sprintf(s,"storing command starting with %c, %c stored",f_c, m_buffer.peek());
//            g_rpiWriteBuffer.push(s,l);
        }
        else
        {
            m_buffer.push(f_c);
//            char s[100];
//            unsigned int  l = sprintf(s,"peeked %c, command_ID is %c, size is %d",m_buffer.peek(),m_commandID,m_buffer.getSize());
//            g_rpiWriteBuffer.push(s,l);
            if (m_buffer.getSize() == 5)
            {
//                char s[100];
//                unsigned int  l = sprintf(s,"interpreting command starting with %c",m_commandID);
//                g_rpiWriteBuffer.push(s,l);
                intepretCommand();
                reset();
            }
        }
    }
    inline void executeCommand()
    {
        m_car.Steer(m_angleValue);
        m_car.Speed(m_speedValue);
//        reset();
    }
private:
    inline void reset()
    {
        m_commandID = 0;
        m_buffer.empty();
    }
    inline void intepretCommand()
    {
        unsigned char test_char = m_buffer.pop();
        if (test_char != m_commandID)
        {
            reset();
//            led1 = 0;
//            char s[100];
//            unsigned int  l = sprintf(s,"interpretation failed at ID. Expected: %c, received: %c",m_commandID,test_char);
//            g_rpiWriteBuffer.push(s,l);
        }
        else
        {
            float l_sign = 0;
            if (m_buffer.peek() == '0')
            {
                l_sign = 1;
            }
            else if (m_buffer.peek() == '1')
            {
                l_sign = -1;
            }
            else
            {
                reset();
//                led1 = 0;
//                char s[100];
//                unsigned int  l = sprintf(s,"interpretation failed a sign");
//                g_rpiWriteBuffer.push(s,l);
                return;
            }
            m_buffer.pop();
            if ((m_buffer.peek() < '0') || (m_buffer.peek() > '9'))
            {
                reset();
//                led1 = 0;
//                char s[100];
//                unsigned int  l = sprintf(s,"interpretation failed at val 1");
//                g_rpiWriteBuffer.push(s,l);
                return;
            }
            else
            {
                m_commandValue = m_buffer.pop() - '0';
            }
            if ((m_buffer.peek() < '0') || (m_buffer.peek() > '9'))
            {
                reset();
//                led1 = 0;
//                char s[100];
//                unsigned int  l = sprintf(s,"interpretation failed at val 2, value is %c",m_buffer.peek());
//                g_rpiWriteBuffer.push(s,l);
                return;
            }
            else
            {
                m_commandValue *= 10;
                m_commandValue += m_buffer.pop() - '0';
                m_commandValue *= l_sign;
            }
            if (m_buffer.pop() != ';')
            {
                reset();
//                led1 = 0;
//                char s[100];
//                unsigned int  l = sprintf(s,"interpretation failed at terminator");
//                g_rpiWriteBuffer.push(s,l);
                return;
            }      
            else
            {
                if(m_commandID == 'A')
                {
                    m_angleValue = m_commandValue;
//                    char s[100];
//                    unsigned int  l = sprintf(s,"set angle to %f\n", m_angleValue);
//                    g_rpiWriteBuffer.push(s,l);
//                    led1 = 1;
                    reset();
                    return;
                }
                if(m_commandID == 'S')
                {
                    m_speedValue = m_commandValue;
//                    char s[100];
//                    unsigned int  l = sprintf(s,"set speed to %f\n", m_speedValue);
//                    g_rpiWriteBuffer.push(s,l);
//                    led1 = 1;
                    reset();
                    return;
                }
                reset();
            }      
        }
    }
    CQueue<unsigned char, 7> m_buffer;
    MOVE& m_car;
    unsigned char m_commandID;
    float m_commandValue;
    float m_speedValue;
    float m_angleValue;
};

#endif
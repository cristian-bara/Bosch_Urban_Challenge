#include "mbed.h"
#include "Queue.h"
#include "Timer/Timer.h"
#include "commandInterpreter.h"
#include "TaskManager.h"
//#include "millis.h"
#include "MOVE.h"
//#include <array>
#include <string>
//#include "rtos.h"
Serial     rpi(USBTX, USBRX); 
//DigitalOut led1(LED1);
MOVE       mycar(D9, D3, D2, D4, A0);
//DigitalOut led1(LED1);
//DigitalOut led2(LED2);
AnalogIn   currentSense(A2);

CQueue<256> g_rpiReadBuffer;
CQueue<256> g_rpiWriteBuffer;

//CTimer_ms timer_ms;
CTimer_100us timer_100us;

bool isBounded(float val, float min, float max)
{
    return ((val >= min) && (val < max));
}

class CBlinker : public CTask
{
public:
    CBlinker(uint32_t f_period) : CTask(f_period), led(LED1) {led = 1;}
private:
    virtual void _run()
    {
        led = !led;
    }
    DigitalOut led;    
};

class CEchoer : public CTask
{
public:
    CEchoer(uint32_t f_period) : CTask(f_period){}
private:
    virtual void _run()
    {
        rpi.printf(".");
    }    
};
template <uint32_t N = 8>
class CSlidingMeanFilter
{
public:
    CSlidingMeanFilter() : m_buffer(), m_idx(0), m_sum(0) {}
    virtual ~CSlidingMeanFilter(){}
    void push(float f_val)
    {
        m_sum -= m_buffer[m_idx];
        m_sum += f_val;
        m_buffer[m_idx] = f_val;
        m_idx = (m_idx+1) % N;
    }
    float getMean()
    {
        return m_sum/N;
    }
    float operator()(float f_val)
    {
        push(f_val);
        return getMean();
    }
private:
    float m_buffer[N];
    uint32_t m_idx;
    float m_sum;
};

class CCurrentReader : public CTask
{
public:
    CCurrentReader(uint32_t f_period) : CTask(f_period){}
private:
    virtual void _run()
    {
//        rpi.printf("%f %f%c%c",m_slidingMeanFilter(mycar.getVNH().GetCurrent()),mycar.getVNH().GetCurrent(),10,13);
        char s[100];
        uint32_t l = sprintf(s,"%f %f%c%c",m_slidingMeanFilter(mycar.getVNH().GetCurrent()),mycar.getVNH().GetCurrent(),10,13);
        g_rpiWriteBuffer.push(s,l);
    } 
    CSlidingMeanFilter<200> m_slidingMeanFilter;
};



CBlinker    g_blinker   (5000);
CEchoer     g_echoer    (100000);
CCurrentReader g_currentReader(100);

CTask* g_taskList[] = {
    &g_echoer,
    &g_currentReader,
    &g_blinker};
//    CTask(100)};

CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(CTask*), 0.0001);
//CTaskManager g_taskManager(g_taskList, 2, 0.0001);


CCommandInterpreter commandInterpreter(mycar);

int main() 
{
    rpi.baud(115200);    
    rpi.printf("%c%cI'm alive!%c%c",10,13,10,13);
    timer_100us.start();       
    while(1) 
    {
        if(!g_rpiWriteBuffer.isEmpty() && rpi.writeable())
        {
            rpi.putc(g_rpiWriteBuffer.pop());         
            continue;
        }
        if(!g_rpiReadBuffer.isEmpty() && !g_rpiWriteBuffer.isFull())
        {
            unsigned char l_c = g_rpiReadBuffer.pop();
            commandInterpreter.interpretChar(l_c);
            g_rpiWriteBuffer.push(l_c);    
            continue;
        }
        if(!g_rpiReadBuffer.isFull() && rpi.readable())
        {
            while(rpi.readable())
            {
                g_rpiReadBuffer.push(rpi.getc());  
            }      
            continue;
        }
        g_taskManager.mainCallback();
        if (timer_100us.get()%10 == 0)
        {
//            rpi.printf("%f%c%c",mycar.getVNH().GetCurrent(),10,13);
            commandInterpreter.executeCommand();
        }
    }
//    timer_ms.stop();
}

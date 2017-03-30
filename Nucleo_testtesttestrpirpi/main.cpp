#include "mbed.h"
#include "Queue.h"
#include "Timer/Timer.h"
#include "commandInterpreter.h"
#include "TaskManager.h"
#include "MOVE.h"
#include <string>
#include <sstream>
#include "L3GD20H.h"
#include "LSM303D.h"

/*

*/

Serial     rpi(USBTX, USBRX); 
MOVE       mycar(D9, D3, D2, D4, A0);
AnalogIn   currentSense(A2);

CQueue<char, 1000> g_rpiReadBuffer;
CQueue<char, 1000> g_rpiWriteBuffer;


CTimer_100us timer_100us;

class CInterfaceBase
{

};

template <class T>
class COutputPin
{
    uint64_t m_interfaceNumber_u64;
};

template <class T>
class CInputPin
{
public:
    void connectSource(const COutputPin<T>& f_sourcePin)
    {

    }
private:
    COutputPin<T>* const m_sourcePin;
};

bool isBounded(float val, float min, float max)
{
    return ((val >= min) && (val < max));
}

class CIMU : public CTask
{
public:

    typedef struct 
    {
        float  ax, ay, az; //accelerometer data
        float  mx, my, mz; //magnetometer data
        float  gx, gy, gz; //gyroscope data
    } CIMUData;

    CIMU(uint32_t f_period):CTask(f_period),acmg(D14,D15),gyro(D14,D15),dataAvailable(false) {}
    bool newDataAvailable() {return dataAvailable;}
    CIMUData getRawIMUData()
    {
        CIMUData l_imuData;
        l_imuData.ax = ax;
        l_imuData.ay = ay;
        l_imuData.az = az;
        l_imuData.mx = mx;
        l_imuData.my = my;
        l_imuData.mz = mz;
        l_imuData.gx = gyr[0]*1.0f;
        l_imuData.gx = gyr[1]*1.0f;
        l_imuData.gx = gyr[2]*1.0f;

        dataAvailable = false;
        return l_imuData;
    }
private:
    virtual void _run()
    {
        acmg.read(&ax, &ay, &az, &mx, &my, &mz);
        gyro.read(gyr);

        dataAvailable = true;
    }


    LSM303D acmg;
    L3GD20H gyro;
    bool dataAvailable;
    float  ax, ay, az; //accelerometer data
    float  mx, my, mz; //magnetometer data
    short  gyr[3];     //gyroscope data
};

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
        char s[] = ".\n\r";
        rpi.printf(s);
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
        char s[100];
        uint32_t l = sprintf(s,"#[%010d]CRNT:%10.4f;%10.4f;;\n\r",timer_100us.get(),m_slidingMeanFilter(mycar.getVNH().GetCurrent()),mycar.getVNH().GetCurrent());
        //g_rpiWriteBuffer.push(s,l);
        rpi.printf("%s",s);
    } 
    CSlidingMeanFilter<200> m_slidingMeanFilter;
};

enum {
    BLINKER = 0,
    ECHOER,
    CURRENT_READER,
    IMU,

    NUM_TASKS
} ETaskIndex;

const float g_baseTick = 0.0001; // seconds


CBlinker        g_blinker       (0.5    / g_baseTick);
CEchoer         g_echoer        (10     / g_baseTick);
CCurrentReader  g_currentReader (0.01   / g_baseTick);
CIMU            g_imu           (0.01   / g_baseTick);

CTask* g_taskList[] = {
    &g_blinker,
    &g_echoer,
    &g_currentReader,
    &g_imu
    };
//    CTask(100)};

CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(CTask*), g_baseTick);
//CTaskManager g_taskManager(g_taskList, 2, g_baseTick);


CCommandInterpreter commandInterpreter(mycar);

int main() 
{
    rpi.baud(115200);    
    rpi.printf("\n\r\n\r");
    rpi.printf("#################\n\r");
    rpi.printf("#               #\n\r");
    rpi.printf("#   I'm alive   #\n\r");
    rpi.printf("#               #\n\r");
    rpi.printf("#################\n\r");
    rpi.printf("\n\r");

    timer_100us.start();       
    while(1) 
    {
        while(!g_rpiWriteBuffer.isEmpty() && rpi.writeable())
        {
            rpi.putc(g_rpiWriteBuffer.pop());         
        //    continue;
        }
        while(!g_rpiReadBuffer.isEmpty() && !g_rpiWriteBuffer.isFull())
        {
            char l_c = g_rpiReadBuffer.pop();
            commandInterpreter.interpretChar(l_c);
            g_rpiWriteBuffer.push(l_c);    
            //continue;
        }
        while(!g_rpiReadBuffer.isFull() && rpi.readable())
        {
            while(rpi.readable())
            {
                char l_c = rpi.getc();
                g_rpiReadBuffer.push(l_c);  
            }      
            //continue;
        }
        if (g_imu.newDataAvailable())
        {
            CIMU::CIMUData l_imuData = g_imu.getRawIMUData();
            char s[150];
            uint32_t l = sprintf(s,"#[%010d]RIMU:%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;;\n\r",
                timer_100us.get(),
                l_imuData.ax,
                l_imuData.ay,
                l_imuData.az,
                l_imuData.mx,
                l_imuData.my,
                l_imuData.mz,
                l_imuData.gx,
                l_imuData.gy,
                l_imuData.gz
                );
            //g_rpiWriteBuffer.push(s,l);                
            rpi.printf("%s",s);
        }
        g_taskManager.mainCallback();
        if (timer_100us.get()%10 == 0)
        {
            commandInterpreter.executeCommand();
        }
    }
}

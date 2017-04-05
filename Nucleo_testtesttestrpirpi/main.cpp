<<<<<<< HEAD
#include "mbed.h"
#include "Queue.h"
#include "Timer/Timer.h"
#include "commandInterpreter.h"
#include "SplineInterpreter.h"
#include "TaskManager.h"
#include "MOVE.h"
#include <string>
#include <sstream>
#include "L3GD20H.h"
#include "LSM303D.h"
#include "CBNO055.h"
#include "MotionControl.h"



Serial     rpi(USBTX, USBRX); 
MOVE       mycar(D9, D3, D2, D4, A0);
AnalogIn   currentSense(A2);

CQueue< char, 1000> g_rpiReadBuffer;
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
        uint32_t l = sprintf(s,"#[%010d]CRNT:%10.4f;%10.4f;;\r\n",timer_100us.get(),m_slidingMeanFilter(mycar.getVNH().GetCurrent()),mycar.getVNH().GetCurrent());
        // g_rpiWriteBuffer.push(s,l);
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

=======
#include <mbed.h>
#include <Queue.h>
#include <Timer/Timer.h>
#include <commandInterpreter.h>
#include <TaskManager.h>
#include <MOVE.h>
#include <string>
#include <sstream>
#include <L3GD20H.h>
#include <LSM303D.h>
#include <IMU.h>
#include <Blinker.h>
#include <Echoer.h>
#include <map>
#include <array>
#include <SerialMonitor.h>
#include <MotionController.h>


Serial          g_rpi(USBTX, USBRX); 
MOVE            g_car(D9, D3, D2, D4, A0);
AnalogIn        currentSense(A2);
LSM303D         g_accelerometerMagnetometer(D14,D15);
L3GD20H         g_gyroscope(D14,D15);
CTimer_100us    timer_100us;
>>>>>>> refs/remotes/cristian-bara/master
const float g_baseTick = 0.0001; // seconds
CBlinker        g_blinker       (0.5    / g_baseTick, LED1);
CEchoer         g_echoer        (10     / g_baseTick, g_rpi);
CIMU            g_imu           (0.01   / g_baseTick, g_accelerometerMagnetometer, g_gyroscope);
CMotionController g_motionController(0.01 / g_baseTick, g_rpi, g_imu, g_car);

CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"MCTL",mbed::callback(CMotionController::staticSerialCallback,&g_motionController)},
    {"ASDF",[](char const *a, char *b){strcpy(b,a);}}
};

<<<<<<< HEAD

CBlinker        g_blinker       (0.5    / g_baseTick);
CEchoer         g_echoer        (10     / g_baseTick);
CCurrentReader  g_currentReader (0.5   / g_baseTick);
// CIMU            g_imu           (0.01   / g_baseTick);
CBNO055         g_bno055        (0.01   / g_baseTick);

CSplineInterpreter g_splineInterpreter;
CMotionControl  g_motionControl (0.01   / g_baseTick,mycar,g_splineInterpreter,0.01);


CTask* g_taskList[] = {
    &g_blinker,
    &g_echoer,
    &g_currentReader,
    &g_bno055,
    &g_motionControl
    // &g_imu
=======
CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

CTask* g_taskList[] = {
    &g_blinker,

    &g_imu,
    &g_motionController,
    &g_serialMonitor,

    &g_echoer
>>>>>>> refs/remotes/cristian-bara/master
    };

CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(CTask*), g_baseTick);

<<<<<<< HEAD
//CTaskManager g_taskManager(g_taskList, 2, g_baseTick);
// CCommandInterpreter commandInterpreter(mycar);


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
            char l_c=g_rpiWriteBuffer.pop();
            
        }
        g_rpiWriteBuffer.empty();
        while(!g_rpiReadBuffer.isEmpty() && !g_rpiWriteBuffer.isFull())
        {
            
            char l_c = g_rpiReadBuffer.pop();
            g_splineInterpreter.put(l_c);
            // commandInterpreter.interpretChar(l_c);
            g_rpiWriteBuffer.push(l_c);    
            //continue;
        }
        while(!g_rpiReadBuffer.isFull() && rpi.readable())
        {
            int l=rpi.readable();
            while(rpi.readable())
            { 
                 char l_c = rpi.getc();
                 rpi.printf("%c",l_c);
                 g_rpiReadBuffer.push(l_c);  
            }      
            //continue;
        }
        if(g_bno055.isNewDataAvailable()){
           CBNO055::CBNO055_Data data=g_bno055.getData();
           char s[150];
           int32_t l=sprintf(s,"#[%010d]BNO055:%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;;\r\n",
                    timer_100us.get(),
                    data.lin_acc_x,
                    data.lin_acc_y,
                    data.lin_acc_z,
                    data.grav_acc_x,
                    data.grav_acc_y,
                    data.grav_acc_z,
                    data.euler_h,
                    data.euler_p,
                    data.euler_r);
            // g_rpiWriteBuffer.push(s,l); 
            // rpi.printf("%s",s);
        }
        if(g_splineInterpreter.isNewDataAvailable(0)){
            CSplineInterpreter::SplineInterpreter_Data data=g_splineInterpreter.getData(0);
            char s[150];
            int32_t l=sprintf(s,"#[%010d]INTR0:%0.004f;%0.004f;%0.004f;%0.004f;%0.004f;%0.004f;%0.004f;%0.004f;%0.004f;;\r\n",
                     timer_100us.get(),
                     data.a.real(),
                     data.a.imag(),
                     data.b.real(),
                     data.b.imag(),
                     data.c.real(),
                     data.c.imag(),
                     data.d.real(),
                     data.d.imag(),
                     data.duration_sec
                     );
            // g_rpiWriteBuffer.push(s,l); 
            rpi.printf("%s",s);
        }
        g_taskManager.mainCallback();
        // if (timer_100us.get()%10 == 0)
        // {
        //     commandInterpreter.executeCommand();
        // }
=======
uint32_t setup()
{
    g_rpi.baud(115200);    
    g_rpi.printf("\r\n\r\n");
    g_rpi.printf("#################\r\n");
    g_rpi.printf("#               #\r\n");
    g_rpi.printf("#   I'm alive   #\r\n");
    g_rpi.printf("#               #\r\n");
    g_rpi.printf("#################\r\n");
    g_rpi.printf("\r\n");

    timer_100us.start();  
    return 0;    
}

uint32_t loop()
{
    g_taskManager.mainCallback();
    return 0;
}

int main() 
{
    uint32_t  l_errorLevel = setup();  
    while(!l_errorLevel) 
    {
        l_errorLevel = loop();
>>>>>>> refs/remotes/cristian-bara/master
    }
    g_rpi.printf("exiting with code: %d",l_errorLevel);
    return l_errorLevel;
}

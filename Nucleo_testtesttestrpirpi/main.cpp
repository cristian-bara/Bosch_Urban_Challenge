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

Serial          g_rpi(USBTX, USBRX); 
MOVE            mycar(D9, D3, D2, D4, A0);
AnalogIn        currentSense(A2);
LSM303D         g_accelerometerMagnetometer(D14,D15);
L3GD20H         g_gyroscope(D14,D15);
CTimer_100us    timer_100us;
const float g_baseTick = 0.0001; // seconds
CBlinker        g_blinker       (0.5    / g_baseTick, LED1);
CEchoer         g_echoer        (10     / g_baseTick, g_rpi);
CIMU            g_imu           (0.01   / g_baseTick, g_accelerometerMagnetometer, g_gyroscope);

std::pair<int,float> asdf = {1,2.3};
std::map<int,float> qwer = {{1,1.2},{2,3.4},{3,5.6}};
std::array<float,5> zxcv = {1.2,3.4,5.6,7.8,9.0};

class CSerialMonitor : public CTask
{
public:
    typedef void (* FCallback)(const string&, string&);
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
                            m_serialPort.printf("message ID is: %4s\r\n",msgID);
                            m_serialPort.printf("message is: %s\r\n",msg);
                            auto l_pair = m_serialSubscriberMap.find(msgID);
                            if (l_pair != m_serialSubscriberMap.end())
                            {
                                char l_resp[256] = "no response given";
                                l_pair->second(msg,l_resp);
                                m_serialPort.printf("response is: %s",l_resp);
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

// class CMotionController : public CTask
// {
// public:
//     CMotionController(uint32_t f_period, Serial& f_serialPort, CIMU& f_imu) : CTask(f_period), m_serialPort(f_serialPort), m_imu(f_imu) {}
//     void serialCallback(const string& f_message, string& f_response)
//     {

//     }
// private:
//     virtual void _run()
//     {

//     }
//     Serial& m_serialPort;
//     CIMU& m_imu;
// };

// CMotionController g_motionController(0.01 / g_baseTick, g_rpi, g_imu);
CSerialMonitor g_serialMonitor(g_rpi, {{"TEST",[](const string& a, string& b) {b = a;}}});//, g_motionController.serialCallback);

CTask* g_taskList[] = {
    &g_blinker,

    &g_imu,
    // &g_motionController,
    &g_serialMonitor,

    &g_echoer
    };
CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(CTask*), g_baseTick);

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
    }
    
    return l_errorLevel;
//        while(!g_g_rpiWriteBuffer.isEmpty() && g_rpi.writeable())
//        {
//            g_rpi.putc(g_g_rpiWriteBuffer.pop());   
//        }
//        while(!g_g_rpiReadBuffer.isEmpty() && !g_g_rpiWriteBuffer.isFull())
//        {
//            char l_c = g_g_rpiReadBuffer.pop();
//            //commandInterpreter.interpretChar(l_c);
//            g_g_rpiWriteBuffer.push(l_c);   
//        }
//        while(!g_g_rpiReadBuffer.isFull() && g_rpi.readable())
//        {
//            while(g_rpi.readable())
//            {
//                char l_c = g_rpi.getc();
//                g_g_rpiReadBuffer.push(l_c);  
//            }      
//       }
//        if (g_imu.newDataAvailable())
//        {
//            CIMU::CIMUData l_imuData = g_imu.getRawIMUData();
//            char s[150];
//            uint32_t l = sprintf(s,"#[%010d]RIMU:%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;%10.4f;;\r\n",
//                timer_100us.get(),
//                l_imuData.ax,
//                l_imuData.ay,
//                l_imuData.az,
//                l_imuData.mx,
//                l_imuData.my,
//                l_imuData.mz,
//                l_imuData.gx,
//                l_imuData.gy,
//                l_imuData.gz
//                );
//            g_rpi.printf("%s",s);
//        }
//        g_taskManager.mainCallback();
//        if (timer_100us.get()%10 == 0)
//        {
//            commandInterpreter.executeCommand();
//        }
//    }
}


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
#include <SplineInterpreter.h>


Serial          g_rpi(USBTX, USBRX); 
MOVE            g_car(D9, D3, D2, D4, A0);
AnalogIn        currentSense(A2);
LSM303D         g_accelerometerMagnetometer(D14,D15);
L3GD20H         g_gyroscope(D14,D15);
CTimer_100us    timer_100us;
const float g_baseTick = 0.0001; // seconds
CBlinker        g_blinker       (0.5    / g_baseTick, LED1);
CEchoer         g_echoer        (10     / g_baseTick, g_rpi);
CIMU            g_imu           (0.01   / g_baseTick, g_accelerometerMagnetometer, g_gyroscope);
CMotionController g_motionController(0.01 / g_baseTick, g_rpi, g_imu, g_car);
CSplineInterpreter g_splineInterpreter;

CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"MCTL",mbed::callback(CMotionController::staticSerialCallback,&g_motionController)},
    {"ASDF",[](char const *a, char *b){strcpy(b,a);}},
    {"SPLN",mbed::callback(CSplineInterpreter::staticSerialCallback,&g_splineInterpreter)}
};

CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

CTask* g_taskList[] = {
    &g_blinker,

    &g_imu,
    &g_motionController,
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
    g_rpi.printf("exiting with code: %d",l_errorLevel);
    return l_errorLevel;
}
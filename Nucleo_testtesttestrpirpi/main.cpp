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
#include <linalg.h>


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

CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"MCTL",mbed::callback(CMotionController::staticSerialCallback,&g_motionController)},
    {"ASDF",[](char const *a, char *b){strcpy(b,a);}}
};

CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

CTask* g_taskList[] = {
    &g_blinker,

    // &g_imu,
    // &g_motionController,
    // &g_serialMonitor,

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
    

    const uint32_t l_dim = 10;

    linalg::CMatrix<double,l_dim,l_dim> l_mat33({{
            {0.7482,    0.4427,    0.8001,    0.1450,    0.2400,    0.1112,    0.0598,    0.4509,    0.0811,    0.7948},
            {0.4505,    0.1067,    0.4314,    0.8530,    0.4173,    0.7803,    0.2348,    0.5470,    0.9294,    0.6443},
            {0.0838,    0.9619,    0.9106,    0.6221,    0.0497,    0.3897,    0.3532,    0.2963,    0.7757,    0.3786},
            {0.2290,    0.0046,    0.1818,    0.3510,    0.9027,    0.2417,    0.8212,    0.7447,    0.4868,    0.8116},
            {0.9133,    0.7749,    0.2638,    0.5132,    0.9448,    0.4039,    0.0154,    0.1890,    0.4359,    0.5328},
            {0.1524,    0.8173,    0.1455,    0.4018,    0.4909,    0.0965,    0.0430,    0.6868,    0.4468,    0.3507},
            {0.8258,    0.8687,    0.1361,    0.0760,    0.4893,    0.1320,    0.1690,    0.1835,    0.3063,    0.9390},
            {0.5383,    0.0844,    0.8693,    0.2399,    0.3377,    0.9421,    0.6491,    0.3685,    0.5085,    0.8759},
            {0.9961,    0.3998,    0.5797,    0.1233,    0.9001,    0.9561,    0.7317,    0.6256,    0.5108,    0.5502},
            {0.0782,    0.2599,    0.5499,    0.1839,    0.3692,    0.5752,    0.6477,    0.7802,    0.8176,    0.6225}
        }});


    // const uint32_t l_dim = 3;

    // linalg::CMatrix<double,l_dim,l_dim> l_mat33({{
    //         {0.7482,    0.4427,    0.8001},
    //         {0.4505,    0.1067,    0.4314},
    //         {0.0838,    0.9619,    0.9106}
    //     }});

    g_rpi.printf("\r\n");
    for (uint8_t i = 0; i < l_dim; i++)
    {
        for (uint8_t j = 0; j < l_dim; j++)
        {
            g_rpi.printf("%12.4f ",l_mat33[i][j]);
        }
        g_rpi.printf("\r\n");
    }

    // l_mat33 = l_mat33 * l_mat33;

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_mat33[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    // l_mat33 *= l_mat33;

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_mat33[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    // linalg::CLUDecomposition<double,l_dim> l_LU = linalg::CLUDecomposition<double,l_dim>(l_mat33);

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_LU.m_L[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_LU.m_U[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     g_rpi.printf("%d\r\n",l_LU.m_P[i]);
    // }

    // l_mat33 = l_LU;

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_mat33[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    l_mat33 = l_mat33.inv();

    g_rpi.printf("\r\n");
    for (uint8_t i = 0; i < l_dim; i++)
    {
        for (uint8_t j = 0; j < l_dim; j++)
        {
            g_rpi.printf("%12.4f ",l_mat33[i][j]);
        }
        g_rpi.printf("\r\n");
    }

    // linalg::CMatrix<double,l_dim,l_dim> l_LInv = l_LU.triLInv();

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_LInv[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    // l_LInv *= l_LU.m_L;

    // g_rpi.printf("\r\n");
    // for (uint8_t i = 0; i < l_dim; i++)
    // {
    //     for (uint8_t j = 0; j < l_dim; j++)
    //     {
    //         g_rpi.printf("%12.4f ",l_LInv[i][j]);
    //     }
    //     g_rpi.printf("\r\n");
    // }

    l_mat33 *= l_mat33.inv();

    g_rpi.printf("\r\n");
    for (uint8_t i = 0; i < l_dim; i++)
    {
        for (uint8_t j = 0; j < l_dim; j++)
        {
            g_rpi.printf("%12.4f ",l_mat33[i][j]);
        }
        g_rpi.printf("\r\n");
    }



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

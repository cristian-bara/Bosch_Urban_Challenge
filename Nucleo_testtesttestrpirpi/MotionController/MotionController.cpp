#include <MotionController.h>


 void CMotionController::_run()
    {
        if(m_splineinterpreter.isNewDataAvailable()){
            m_isSplineActivated=true;
            CSplineInterpreter::SplineInterpreter_Data spline=m_splineinterpreter.getData();
            m_motionPlanner.setMotionPlannerParameters(spline.a,spline.b,spline.c,spline.d,spline.duration_sec,m_period_sec);
            m_serialPort.printf("@SPLN:%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;;\r\n"
                                    ,spline.a.real()
                                    ,spline.a.imag()
                                    ,spline.b.real()
                                    ,spline.b.imag()
                                    ,spline.c.real()
                                    ,spline.c.imag()
                                    ,spline.d.real()
                                    ,spline.d.imag()
                                    ,spline.duration_sec);
        }
        if(m_isSplineActivated ){
            if(m_motionPlanner.hasValidValue()){
                std::pair<float,float> motion=m_motionPlanner.getNextVelocity();  
                m_speed=motion.first;
                m_angle=motion.second;
                m_serialPort.printf("@SPLN:%4.4f;%4.4f;;\r\n",m_speed,m_angle);
            }else{
                m_serialPort.printf("@SPLN:Stop;;\r\n");
                m_speed=0.0;
                m_angle=0.0;
                m_isSplineActivated=false;
            }
        }

        m_car.Steer(m_angle);
        m_car.Speed(m_speed);
        CIMU::CIMUData l_imuData = m_imu.getRawIMUData();        
        // m_serialPort.printf("@MCTL:%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;%8.4f;;\r\n"
        //     ,m_speed
        //     ,m_angle
        //     ,m_car.getVNH().GetCurrent()
        //     ,l_imuData.ax
        //     ,l_imuData.ay
        //     ,l_imuData.az
        //     ,l_imuData.mx
        //     ,l_imuData.my
        //     ,l_imuData.mz
        //     ,l_imuData.gx
        //     ,l_imuData.gy
        //     ,l_imuData.gz
        //     );
    }
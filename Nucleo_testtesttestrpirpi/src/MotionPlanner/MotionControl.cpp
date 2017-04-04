#include "MotionControl.h"


void CMotionControl::_run(){
    if(this->m_splineInterpreter.isNewDataAvailable(1)){
        CSplineInterpreter::SplineInterpreter_Data l_spline_data=this->m_splineInterpreter.getData(1);
        if(m_period_sec>l_spline_data.duration_sec){
            this->m_motionPlanner.setMotionPlannerParameters(l_spline_data.a,l_spline_data.b,l_spline_data.c,l_spline_data.d,l_spline_data.duration_sec,1);
            this->m_bezValue_sec=0.0;
            m_bez_step=m_period_sec/l_spline_data.duration_sec;
        }
    }

    if(this->m_bezValue_sec<1.0 && this->m_bezValue_sec>=0){
        //Velocity and angle (degree)
        std::pair<float,float> commands=this->m_motionPlanner.getVelocity(this->m_bezValue_sec);
        m_bezValue_sec+=m_bez_step;
        m_car.Steer(commands.second);
        m_car.Speed(commands.first);
    }
    else{
        m_car.Steer(0);
        m_car.Speed(0);
    }
    
}
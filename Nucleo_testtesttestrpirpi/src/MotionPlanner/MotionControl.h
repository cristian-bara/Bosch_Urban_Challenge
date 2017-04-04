#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "MotionPlanner.h"

#include "mbed.h"
#include "TaskManager.h"
#include "MOVE.h"
#include "SplineInterpreter.h"

class CMotionControl: public CTask{
    public:
        CMotionControl( uint32_t                f_period,
                        MOVE&                   f_car,
                        CSplineInterpreter&     f_splineInterpreter_ptr,
                        float                   f_period_sec
                        ):CTask(f_period),m_car(f_car),m_splineInterpreter(f_splineInterpreter_ptr),m_period_sec(f_period_sec){
                            this->m_bezValue_sec=-1.0;
        }

        ~CMotionControl(){}

    private:
        virtual void _run();

        float m_bezValue_sec;
        float m_period_sec;
        float m_bez_step;
        CMotionPlanner m_motionPlanner;
        CSplineInterpreter& m_splineInterpreter;
        MOVE& m_car;

};

#endif
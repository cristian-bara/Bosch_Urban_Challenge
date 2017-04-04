#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include "BezierCurve.h"
#include <complex>
#include <utility>
#include <math.h>

#define WHEELBASE 1.0



class CMotionPlanner
{
    public:
        CMotionPlanner(){
                this->isInitialized=false;
        }

        CMotionPlanner( std::complex<float>     a,
                        std::complex<float>     b,
                        std::complex<float>     c,
                        std::complex<float>     d,
                        float                   motion_duration_i,
                        float                   timestep_i):bezierCurve(a,b,c,d),motion_duration(motion_duration_i),time_step(timestep_i)
        {
                this->nrStep=static_cast<int32_t>(this->motion_duration/this->time_step);
                this->bezierValueInput_step=1.0/(int)(this->motion_duration/this->time_step);
                this->next_bezierValueInput=0.0;
                this->isInitialized=true;
        }

        void setMotionPlannerParameters(        std::complex<float>     a,
                                                std::complex<float>     b,
                                                std::complex<float>     c,
                                                std::complex<float>     d,
                                                float                   motion_duration_i,
                                                float                   timestep_i);

        virtual ~CMotionPlanner();
        BezierCurve<float> getBezierCurve();
//        <dl,dr> Velocity longitudinal and angular
        std::pair<float,float> getNextVelocity();
        int32_t getNrStep();
//        <dl,dr> Velocity longitudinal and angular, Input value [0,1]
        std::pair<float,float> getVelocity(float input_value);

    protected:

    private:
        BezierCurve<float> bezierCurve;
        float motion_duration;
        float time_step;

        float bezierValueInput_step;
        float next_bezierValueInput;
        int32_t nrStep;
        bool isInitialized;

};

#endif // CMotionPlanner_H

#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include "BezierCurve.h"
#include <complex>
#include <utility>
#include <math.h>

#define WHEELBASE 0.265

class CBezierMotionPlanner
{
    public:
        CBezierMotionPlanner(){
                this->isInitialized=false;
        }

        CBezierMotionPlanner( std::complex<float>     a,
                        std::complex<float>     b,
                        std::complex<float>     c,
                        std::complex<float>     d,
                        float                   motion_duration_i,
                        float                   timestep_i):bezierCurve(a,b,c,d),motion_duration(motion_duration_i),time_step(timestep_i)
        {
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

        virtual ~CBezierMotionPlanner();
        BezierCurve<float> getBezierCurve();
//      Velocity longitudinal and angular
        std::pair<float,float> getNextVelocity();
        
//      Velocity longitudinal and angular, Input value [0,1]
        std::pair<float,float> getVelocity(float input_value);

        bool hasValidValue(){return (next_bezierValueInput>=0 && next_bezierValueInput<=1);}

    protected:

    private:
        BezierCurve<float> bezierCurve;
        float motion_duration;
        float time_step;

        float bezierValueInput_step;
        float next_bezierValueInput;
        bool isInitialized;

};

#endif // CMotionPlanner_H

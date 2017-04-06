#include "BezierMotionPlanner.h"

#include <iostream>

CBezierMotionPlanner::~CBezierMotionPlanner()
{
}


BezierCurve<float> CBezierMotionPlanner::getBezierCurve(){
    return this->bezierCurve;
}


std::pair<float,float> CBezierMotionPlanner::getNextVelocity(){
    std::pair<float,float> commands=this->getVelocity(next_bezierValueInput);
    this->next_bezierValueInput+=this->bezierValueInput_step;
    return commands;
}


std::pair<float,float> CBezierMotionPlanner::getVelocity(float input_value){
    if(!this->isInitialized) return std::pair<float,float>(0,0);
    
    std::complex<float> dS=this->bezierCurve.get_FO_DerivateValue(input_value);

    float dl_absolute=sqrt((dS*std::conj(dS)).real());//[0,1]//Length of the vector
    float dl_real=dl_absolute/this->motion_duration;

    std::complex<float> ddS=this->bezierCurve.get_SO_DerivateValue(input_value);

    std::complex<float> temp1=std::conj(dS)*ddS;
    float temp1_im2=-2.0*temp1.imag();

    float k=temp1_im2/pow(dl_absolute,3);
    float angle_rad=atan(k*WHEELBASE);
    float angle_deg=(180.f/M_PI)*angle_rad;

    std::pair<float,float> commands(dl_real,angle_deg);
    return commands;
}


void CBezierMotionPlanner::setMotionPlannerParameters(  std::complex<float>     a,
                                                        std::complex<float>     b,
                                                        std::complex<float>     c,
                                                        std::complex<float>     d,
                                                        float                   motion_duration_i,
                                                        float                   timestep_i)
{
    this->motion_duration=motion_duration_i;
    this->time_step=timestep_i;
    this->bezierCurve.setBezierCurve(a,b,c,d);
    this->bezierValueInput_step=1.0/(int)(this->motion_duration/this->time_step);
    this->next_bezierValueInput=0.0;
    this->isInitialized=true;                        

}
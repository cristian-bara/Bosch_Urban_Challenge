#include "MotionPlanner.h"

#include <iostream>
// CMotionPlanner::CMotionPlanner(std::complex<float> a,std::complex<float> b,std::complex<float>c,std::complex<float> d,float motion_duration,float timestep)
// {
//     std::complex<float> s[4]={a,b,c,d};
//     this->bezierCurve=BezierCurve<float>(s);
//     this->motion_duration=motion_duration;
//     this->time_step=timestep;
// //    this->motion_duration/this->time_step= Number of step

//     this->nrStep=static_cast<int32_t>(this->motion_duration/this->time_step);
//     this->bezierValueInput_step=1.0/(int)(this->motion_duration/this->time_step);
//     this->next_bezierValueInput=0.0;
// }

CMotionPlanner::~CMotionPlanner()
{
    //dtor
}


BezierCurve<float> CMotionPlanner::getBezierCurve(){
    return this->bezierCurve;
}


std::pair<float,float> CMotionPlanner::getNextVelocity(){
    std::pair<float,float> commands=this->getVelocity(next_bezierValueInput);
    this->next_bezierValueInput+=this->bezierValueInput_step;
    return commands;
}

int32_t CMotionPlanner::getNrStep(){
    return this->nrStep;
}

std::pair<float,float> CMotionPlanner::getVelocity(float input_value){
    if(!this->isInitialized) return std::pair<float,float>(0,0);
    
    std::complex<float> dS=this->bezierCurve.get_FO_DerivateValue(input_value);

    float dl_absolute=sqrt((dS*std::conj(dS)).real());//[0,1]//Length of the vector
    float dl_real=dl_absolute/this->motion_duration;

    std::complex<float> ddS=this->bezierCurve.get_SO_DerivateValue(input_value);

    std::complex<float> temp1=std::conj(dS)*ddS;
    float temp1_im2=2*temp1.imag();

    float k=temp1_im2/pow(dl_absolute,3);
    float angle_rad=atan(k*WHEELBASE);
    float angle_deg=(180.f/M_PI)*angle_rad;

    std::pair<float,float> commands(dl_real,angle_deg);

    this->next_bezierValueInput+=this->bezierValueInput_step;
    return commands;
}


void CMotionPlanner::setMotionPlannerParameters(     std::complex<float>     a,
                                                    std::complex<float>     b,
                                                    std::complex<float>     c,
                                                    std::complex<float>     d,
                                                    float                   motion_duration_i,
                                                    float                   timestep_i)
{
    this->bezierCurve.setBezierCurve(a,b,c,d);
    this->nrStep=static_cast<int32_t>(this->motion_duration/this->time_step);
    this->bezierValueInput_step=1.0/(int)(this->motion_duration/this->time_step);
    this->next_bezierValueInput=0.0;
    this->isInitialized=true;                        

}
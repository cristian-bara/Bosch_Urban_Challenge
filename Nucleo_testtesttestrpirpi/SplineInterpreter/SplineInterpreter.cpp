#include "SplineInterpreter.h"

void CSplineInterpreter::setData(   float       f_a_x,
                                    float       f_a_y,
                                    float       f_b_x,
                                    float       f_b_y,
                                    float       f_c_x,
                                    float       f_c_y,
                                    float       f_d_x,
                                    float       f_d_y,
                                    float       f_duration_sec){
    this->data.a=std::complex<float>(f_a_x,f_a_y);
    this->data.b=std::complex<float>(f_b_x,f_b_y);
    this->data.c=std::complex<float>(f_c_x,f_c_y);
    this->data.d=std::complex<float>(f_d_x,f_d_y);
    this->data.duration_sec=f_duration_sec;
    this->isNewData=true;
}


CSplineInterpreter::SplineInterpreter_Data CSplineInterpreter::getData(){
    SplineInterpreter_Data data;
    data.a=this->data.a;
    data.b=this->data.b;
    data.c=this->data.c;
    data.d=this->data.d;
    data.duration_sec=this->data.duration_sec;
    this->isNewData=false;
    return data;
}


void CSplineInterpreter::serialCallback(char const * f_message, char * f_response){
     float a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec;
     int32_t nrData=sscanf(f_message,"%f;%f;%f;%f;%f;%f;%f;%f;%f",
                                    &a_x,
                                    &a_y,
                                    &b_x,
                                    &b_y,
                                    &c_x,
                                    &c_y,
                                    &d_x,
                                    &d_y,
                                    &duration_sec);
    if(9==nrData){
        setData(a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec);
        sprintf(f_response,"ack;;");
    }
    else{
        sprintf(f_response,"sintax error;;");
    }
}
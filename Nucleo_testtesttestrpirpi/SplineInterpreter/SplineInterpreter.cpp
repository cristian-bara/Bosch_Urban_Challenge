#include "SplineInterpreter.h"

void CSplineInterpreter::put(char i_c){
    if(i_c=='#'){
        this->m_buffer.empty();
        this->m_buffer.push(i_c);
        
    }
    else if(i_c=='\r'){
        decodificateMessage();
    }
    else{
        this->m_buffer.push(i_c);
    }
}


void CSplineInterpreter::decodificateMessage(){
    // char l_buffer[SPLINE_MESSAGE_LENGTH];
    char l_buffer2[SPLINE_MESSAGE_LENGTH];
    int buffer_length=m_buffer.getSize();
    for(int i=0;i<buffer_length;++i) {
        l_buffer2[i]=this->m_buffer.pop();
    }
    SplineInterpreter_Data new_data;
    
    float a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec;
    int l_b=sscanf(l_buffer2,"#SPLN;%f;%f;%f;%f;%f;%f;%f;%f;%f;;\n\r",
                                    &a_x,
                                    &a_y,
                                    &b_x,
                                    &b_y,
                                    &c_x,
                                    &c_y,
                                    &d_x,
                                    &d_y,
                                    &duration_sec);
    if(l_b==9){
        setData(a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec);
    }   
}

void CSplineInterpreter::setData(   float       a_x,
                                    float       a_y,
                                    float       b_x,
                                    float       b_y,
                                    float       c_x,
                                    float       c_y,
                                    float       d_x,
                                    float       d_y,
                                    float       duration_sec){
    this->data.a=std::complex<float>(a_x,a_y);
    this->data.b=std::complex<float>(b_x,b_y);
    this->data.c=std::complex<float>(c_x,c_y);
    this->data.d=std::complex<float>(d_x,d_y);
    this->data.duration_sec=duration_sec;
    this->isNewData=true;
    setNotAccessedAll();
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


CSplineInterpreter::SplineInterpreter_Data CSplineInterpreter::getData(int32_t id){
     if(id>=0 && id<NR_ACCESSED_SPLN_INTR){
        this->accessed[id]=true;
    }
    SplineInterpreter_Data data;
    data.a=this->data.a;
    data.b=this->data.b;
    data.c=this->data.c;
    data.d=this->data.d;
    data.duration_sec=this->data.duration_sec;
    return data;
}


void CSplineInterpreter::setNotAccessedAll(){
    for(int32_t id=0;id<NR_ACCESSED_SPLN_INTR;++id){
        this->accessed[id]=false;
    }
}
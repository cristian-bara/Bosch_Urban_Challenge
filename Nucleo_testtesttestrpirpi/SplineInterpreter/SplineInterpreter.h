#ifndef SPLINE_INTERPRETER_H
#define SPLINE_INTERPRETER_H

#include "Queue.h"
#include "mbed.h"
#include <Complex>
#define SPLINE_MESSAGE_LENGTH 100

#define NR_ACCESSED_SPLN_INTR 2

class CSplineInterpreter{
    public:
    
        typedef struct{
            std::complex<float> a,b,c,d;
            float duration_sec;
        }SplineInterpreter_Data;

        static void staticSerialCallback(void* obj,char const * message, char * response){
                CSplineInterpreter* self=static_cast<CSplineInterpreter*>(obj);
                self->serialCallback(message,response);
        }
        
        bool isNewDataAvailable(){return this->isNewData;};
        SplineInterpreter_Data getData();

        CSplineInterpreter():data(){};
        ~CSplineInterpreter(){};

    private:
        void serialCallback(char const * f_message, char * f_response);
        inline void setData(float       f_a_x,
                            float       f_a_y,
                            float       f_b_x,
                            float       f_b_y,
                            float       f_c_x,
                            float       f_c_y,
                            float       f_d_x,
                            float       f_d_y,
                            float       f_duration_sec);
      bool isNewData;
      SplineInterpreter_Data data;
};


#endif
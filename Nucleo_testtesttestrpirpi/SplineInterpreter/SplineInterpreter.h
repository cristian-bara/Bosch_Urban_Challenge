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

        void put( char i_c);

        bool isNewDataAvailable(){return this->isNewData;};
        SplineInterpreter_Data getData();
        

        //Multi-access
        bool isNewDataAvailable(int32_t id){
            if(id<0 || id>NR_ACCESSED_SPLN_INTR){
                return false;
            }
            else{
                return !this->accessed[id];
            }

        }
        SplineInterpreter_Data getData(int32_t id);

        CSplineInterpreter():m_buffer(),data()
        {
            for(int32_t id=0;id<NR_ACCESSED_SPLN_INTR;++id){
                 this->accessed[id]=true;
            }
        };
        ~CSplineInterpreter(){};

    private:
        inline void setNotAccessedAll();

        inline void decodificateMessage();
        inline void setData(float       a_x,
                            float       a_y,
                            float       b_x,
                            float       b_y,
                            float       c_x,
                            float       c_y,
                            float       d_x,
                            float       d_y,
                            float       duration_sec);

      
      bool isNewData;
      bool accessed[2];

      CQueue<char, SPLINE_MESSAGE_LENGTH> m_buffer;
      
      SplineInterpreter_Data data;
};


#endif
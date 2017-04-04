#ifndef CBNO055_H
#define CBNO055_H

#include "BNO055.h"
#include "TaskManager.h"

class CBNO055: public CTask
{
    public:
        typedef struct{
            float lin_acc_x,lin_acc_y,lin_acc_z;
            float grav_acc_x,grav_acc_y,grav_acc_z;
            float euler_h,euler_p,euler_r;
        }CBNO055_Data;
        


        CBNO055(uint32_t f_period) : CTask(f_period),nbo055(D14,D15,D8) {
            dataAvailable=false;
            nbo055.reset();
        }

        bool isNewDataAvailable(){return dataAvailable;}
    
       CBNO055_Data getData(){
            CBNO055_Data temp;
            temp.lin_acc_x=this->data.lin_acc_x;temp.lin_acc_y=this->data.lin_acc_y;temp.lin_acc_z=this->data.lin_acc_z;
            temp.grav_acc_x=this->data.grav_acc_x;temp.grav_acc_y=this->data.grav_acc_y;temp.grav_acc_z=this->data.grav_acc_z;
            temp.euler_h=this->data.euler_h;temp.euler_r=this->data.euler_r;temp.euler_p=this->data.euler_p;
            dataAvailable=false;
            return temp;
        }

        

    private:
        virtual void _run(){
            BNO055_LIN_ACC_TypeDef linear_acceleration;
            nbo055.get_linear_accel(&linear_acceleration);
            BNO055_EULER_TypeDef euler_angles;
            nbo055.get_Euler_Angles(&euler_angles);
            BNO055_GRAVITY_TypeDef gravity_acc;
            nbo055.get_gravity(&gravity_acc);
            this->data.lin_acc_x=linear_acceleration.x;this->data.lin_acc_y=linear_acceleration.y;this->data.lin_acc_z=linear_acceleration.z;
            this->data.euler_h=euler_angles.h;this->data.euler_r=euler_angles.r;this->data.euler_p=euler_angles.p;
            this->data.grav_acc_x=gravity_acc.x;this->data.grav_acc_y=gravity_acc.y;this->data.grav_acc_z=gravity_acc.z;
            this->dataAvailable=true;
        }
    
    BNO055 nbo055;
    CBNO055_Data data;
    
    bool dataAvailable;
};

#endif
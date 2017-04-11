#ifndef CAR_SYSTEM_H
#define CAR_SYSTEM_H

#include <math.h>
// #include <linalg.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD 
    #define DEG2RAD M_PI/180.0
#endif


class  CVehicleDynamicsModel{
    public:
        typedef struct{
            float x,y;
            float x_dot,y_dot;
            float x_dot_prev,y_dot_prev;
            float teta_rad;
            float teta_rad_dot;
            float omega;// Rotation speed in global coordinate system 
            float i;// Current
        }TState;

        typedef struct{
            // Motor input voltage;
            float v;
            // Steering angle
            float alpha;
        }TInput;

        typedef struct{
            float x_ddot,y_ddot;
            float teta_rad_dot;
            float speed;
            float alpha;
        }TOutput;   

        TState getStates(){
            return m_states;
        }

        CVehicleDynamicsModel(   float       f_gamma
                                ,float      f_dt
                                ,float      f_wb
                                ,float      f_b
                                ,float      f_J
                                ,float      f_K
                                ,float      f_R
                                ,float      f_L)
            :m_gamma(f_gamma)
            ,m_dt(f_dt)
            ,m_wb(f_wb)
            ,m_bJ(f_b/f_J)
            ,m_KJ(f_K/f_J)
            ,m_KL(f_K/f_L)
            ,m_RL(f_R/f_L)
            {
                m_states.x=m_states.y=0;
                m_states.x_dot=m_states.y_dot=0;
                m_states.x_dot_prev=m_states.y_dot_prev=0;
                m_states.teta_rad=0;
                m_states.teta_rad_dot=0;
                m_states.omega=0;
                m_states.i=0;
            }

          CVehicleDynamicsModel(    float       f_gamma
                                ,float      f_dt
                                ,float      f_wb
                                ,float      f_b
                                ,float      f_J
                                ,float      f_K
                                ,float      f_R
                                ,float      f_L
                                ,float      f_x
                                ,float      f_y
                                ,float      f_teta_rad)
            :m_gamma(f_gamma)
            ,m_dt(f_dt)
            ,m_wb(f_wb)
            ,m_bJ(f_b/f_J)
            ,m_KJ(f_K/f_J)
            ,m_KL(f_K/f_L)
            ,m_RL(f_R/f_L)
            {
                m_states.x=f_x;
                m_states.y=f_y;
                m_states.x_dot=m_states.y_dot=0;
                m_states.x_dot_prev=m_states.y_dot_prev=0;
                m_states.teta_rad=f_teta_rad;
                m_states.teta_rad_dot=0;
                m_states.omega=0;
                m_states.i=0;
            }

        void update(TInput f_input){
            m_states.x+=m_dt*m_states.x_dot;
            m_states.y+=m_dt*m_states.y_dot;
            
            m_states.x_dot_prev=m_states.x_dot;
            m_states.y_dot_prev=m_states.y_dot;

            m_states.x_dot=m_gamma*m_states.omega*sin(m_states.teta_rad);
            m_states.y_dot=m_gamma*m_states.omega*cos(m_states.teta_rad);

            float l_alpha_rad=f_input.alpha*DEG2RAD;
            
            m_states.teta_rad_dot=m_dt*m_gamma*m_states.omega*tan(l_alpha_rad)/m_wb;
            m_states.teta_rad+=m_states.teta_rad_dot;
            
            float omega_k_1=(1-m_dt*m_bJ)*m_states.omega+m_dt*m_KJ*m_states.i;//next state of the motor's rotation speed
            m_states.i=(1-m_dt*m_KL)*m_states.omega-m_dt*m_RL*m_states.i+m_dt*f_input.v;
            m_states.omega=omega_k_1;
        }

        TOutput  getOutput(TInput f_input){
            TOutput output;
            output.x_ddot=m_states.x_dot-m_states.x_dot_prev;
            output.y_ddot=m_states.y_dot-m_states.y_dot_prev;
            output.teta_rad_dot=m_states.teta_rad_dot;
            output.speed=m_states.omega*m_gamma;
            output.alpha=f_input.alpha;
            return output;
        }



    private:
        TState m_states;
        //Constant values
        // gamma=Meter/Rotation
        const float m_gamma;
        // Time step
        const float m_dt;
        // Wheel base distance in meter
        const float m_wb;
        // Motor Constants
        const float m_bJ,m_KJ,m_KL,m_RL;

};


#endif
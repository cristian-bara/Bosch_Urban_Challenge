#ifndef JACOBIAN_MATRIX_H
#define JACOBIAN_MATRIX_H

#include "<linalg.h>"
#include "<VehicleModel.h>"
#include <array>
#include <math.h>

#define DEG2RAD M_PI/180.0

class CJacobianCalculator{
    public:
        CJacobianCalculator(    float       f_gamma
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
               m_JacobianOutputMatrix=calcJacobianOutputMatrix();
            }

    private:
        template<class T,uint32_t N,uint32_t M>
        void setZero(std::array<std::array<T,N>,M>& f_data){
            for(uint32_t i=0;i<M;++i){
                for(uint32_t j=0;j<N;++j){
                    f_data[i][j]=static_cast<T>(0);
                }
            }
        }

        inline void  setJacobianStateMatrixWithOne(std::array<std::array<float,12>,10>& f_matrix){
            f_matrix[0][0]=f_matrix[0][2]=f_matrix[1][1]=1;
            f_matrix[1][3]=f_matrix[4][0]=f_matrix[5][3]=1;
            f_matrix[6][6]=1;
            return;
        }

        linalg::CMatrix<float,5,12> calcJacobianOutputMatrix(){
            std::array<std::array<float,12>,5> l_data;
            setZero<float,12,5>(l_data);
            l_data[0][2]=l_data[1][3]=l_data[2][7]=l_data[4][11]=1;
            l_data[0][4]=-1;
            l_data[1][5]=-1;
            l_data[3][8]=m_gamma;
            linalg::CMatrix<float,5,12> l_matrix(l_data);
            return l_matrix;
        }
    
    public:

        linalg::CMatrix<float,5,12> getJacobianOutputMatrix(){
            return this->m_JacobianOutputMatrix;
        }
    
        linalg::CMatrix<float,10,12> getJacobianStateMatrix(CVehicleDynamicsModel::TState f_state,CVehicleDynamicsModel::TInput f_input){
            std::array<std::array<float,12>,10> l_data;
            setZero<float,12,10>(l_data);
            setJacobianStateMatrixWithOne(l_data);
            //Setting values in the jacobian matrix
            l_data[2][6]=m_gamma*f_state.omega*cos(f_state.teta_rad);
            l_data[2][8]=m_gamma*sin(f_state.teta_rad);

            l_data[3][6]=-m_gamma*f_state.omega*sin(f_state.teta_rad);
            l_data[3][8]=m_gamma*cos(f_state.teta_rad);

            float l_alpha_rad=f_input.alpha*DEG2RAD;
            l_data[7][8]=l_data[6][8]=m_dt*m_gamma/m_wb*tan(l_alpha_rad);
            l_data[7][11]=l_data[6][11]=m_dt*m_gamma*f_state.omega/(m_wb*pow(cos(l_alpha_rad),2));

            l_data[8][8]=(1-m_dt*m_bJ);
            l_data[8][9]=m_dt*m_KJ;
            l_data[9][8]=(1-m_dt*m_KL);
            l_data[9][9]=-m_dt*m_RL;
            l_data[9][10]=m_dt;
            
            linalg::CMatrix<float,10,12> l_matrix(l_data);
            return l_matrix;
        }

    private:
        


        //Constant values
        // gamma=Meter/Rotation
        const float m_gamma;
        // Time step
        const float m_dt;
        // Wheel base distance in meter
        const float m_wb;
        // Motor Constants
        const float m_bJ,m_KJ,m_KL,m_RL;
        //Please leave this matrix to be the last member in this class, as it will be initialized ;
        linalg::CMatrix<float,5,12> m_JacobianOutputMatrix;
};


#endif
#ifndef _IMU_
#define _IMU_

class Cimu
{
    public:
        Cimu(PinName, PinName);
        ~Cimu() {};
        float get_ax(void);    //coordonatele date de accelerometru
        float get_ay(void);
        float get_az(void);
        float get_mx(void);    //coordonatele date de giroscop
        float get_my(void);
        float get_mz(void);
        float get_gx(void);    //coordonatele date de magnetometru
        float get_gy(void);
        float get_gz(void);
        void calibration(void);   //calibrare 
        bool update_acmg(void);  //update coordonate accelerometru
        bool update_gyro(void);  //update coordonate giroscop
        bool update_all(void);   //update pt toate coordonatele
    private:   
        L3GD20H gyro;
        LSM303D acmg;
        unsigned int size;
        float m_ax, m_ay, m_az, m_mx, m_my, m_mz;
        float m_cal_ax, m_cal_ay, m_cal_az, m_cal_mx, m_cal_my, m_cal_mz;
        short m_g[3];
        short m_cal_g[3];
};

#endif
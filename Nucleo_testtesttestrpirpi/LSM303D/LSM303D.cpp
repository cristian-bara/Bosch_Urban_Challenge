#include "mbed.h"
#include "LSM303D.h"


const int addr_acc_mag = 0x3A;

enum REG_ADDRS {
    /* --- Mag --- */
    OUT_X_M     = 0x08,
    OUT_Y_M     = 0x0A,
    OUT_Z_M     = 0x0C,
    /* --- Acc --- */
    OUT_X_A     = 0x28,
    OUT_Y_A     = 0x2A,
    OUT_Z_A     = 0x2C,
    //
    CTRL0       = 0x1F,
    CTRL1       = 0x20,
    CTRL2       = 0x21,
    CTRL3       = 0x22,
    CTRL4       = 0x23,
    CTRL5       = 0x24,
    CTRL6       = 0x25,
    CTRL7       = 0x26
};

bool LSM303D::write_reg(int addr_i2c,int addr_reg, char v)
{
    char data[2] = {(char)addr_reg, v}; 
    return LSM303D::_LSM303.write(addr_i2c, data, 2) == 0;
}

bool LSM303D::read_reg(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg; 
    bool result = false;
    
    __disable_irq();
    if ((_LSM303.write(addr_i2c, &data, 1) == 0) && (_LSM303.read(addr_i2c, &data, 1) == 0)){
        *v = data;
        result = true;
    }
    __enable_irq();
    return result;
}

LSM303D::LSM303D(PinName sda, PinName scl):
    _LSM303(sda, scl)
{
    char reg_v;
    _LSM303.frequency(200000);
        
    reg_v = 0;
    
    //acc
    reg_v |= 0x57;          /* 50 Hz X/Y/Z axis enable. */
    write_reg(addr_acc_mag,CTRL1,reg_v);

    /* -- mag --- */
    reg_v = 0;
    reg_v |= 0x78;     //M_RES = 11 (high resolution mode); M_ODR = 100 (50 Hz ODR)
    write_reg(addr_acc_mag,CTRL5,reg_v);

 

    reg_v = 0;    
    reg_v |= 0x00;   //continuous mag
    write_reg(addr_acc_mag,CTRL7,reg_v);
}


bool LSM303D::read(float *ax, float *ay, float *az, float *mx, float *my, float *mz) {
    char acc[6], mag[6];
 
    if (recv(addr_acc_mag, OUT_X_A, acc, 6) && recv(addr_acc_mag, OUT_X_M, mag, 6)) {
        *ax =  float(short(acc[1] << 8 | acc[0]))/8192;  //32768/4=8192
        *ay =  float(short(acc[3] << 8 | acc[2]))/8192;
        *az =  float(short(acc[5] << 8 | acc[4]))/8192;
        //full scale magnetic readings are from -2048 to 2047
        //gain is x,y =1100; z = 980 LSB/gauss
        *mx = float(short(mag[0] << 8 | mag[1]))/1100;
        *mz = float(short(mag[2] << 8 | mag[3]))/980;
        *my = float(short(mag[4] << 8 | mag[5]))/1100;
 
        return true;
    }
 
    return false;
}


bool LSM303D::recv(char sad, char sub, char *buf, int length) {
    if (length > 1) sub |= 0x80;
 
    return _LSM303.write(sad, &sub, 1, true) == 0 && _LSM303.read(sad, buf, length) == 0;
}

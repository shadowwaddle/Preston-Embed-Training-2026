#include "BNO055.h"


//ADDRESSES

// Change mode register
#define BNO055_OPR_MODE 0x3D
#define BNO055_OPR_MODE_CONFIGMODE 0x00 


// Power setting register
#define BNO055_PWR_MODE 0x3E
#define BNO055_PWR_MODE_NORMAL 0x00

// First accel register
#define BNO055_ACC_DATA_X_LSB 0x08

// First gyro register
#define BNO055_GYR_DATA_X_LSB 0x14

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}

// Constructor
BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    :_i2c(i2c), chip_addr(addr), _res(p_reset)
{

}

// Init function, not sure if just setting it to normal power is all I need to do...
void BNO055::init() noexcept 
{
    // Set power to normal
    dt[0] = BNO055_PWR_MODE;
    dt[1] = BNO055_PWR_MODE_NORMAL;
    _i2c.write(chip_addr, dt, 2, false);
}
 
/*
Reset function
POR (power-on reset) executed at every power on, 
also can be triggered by applying a low signal to the nRESET pin for at least 20ns
*/
void BNO055::reset() noexcept
{
    // Setting reset pin to low to initiate reset
    _res = 0;
    ThisThread::sleep_for(1);
    // Setting reset pin back to high
    _res = 1;
    ThisThread::sleep_for(650);
}

// Get accel function
void BNO055::get_accel(BNO055_VECTOR_TypeDef *la)
{
    int16_t x, y, z;
    dt[0] = BNO055_ACC_DATA_X_LSB;

    // i2c communication
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    // setting values to x, y, z
    x = (dt[1] << 8) | dt[0];
    y = (dt[3] << 8) | dt[2];
    z = (dt[5] << 8) | dt[4];

    // 1 m/s^2 = 100 LSB therefore to get to m/s^2, divide by 100
    la->x = (double)x / 100;
    la->y = (double)y / 100;
    la->z = (double)z / 100;

}

// Get gyro function
void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr)
{
    int16_t x, y, z;
    dt[0] = BNO055_GYR_DATA_X_LSB;

    // i2c communication
    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);
    // setting values of x, y, z
    x = (dt[1] << 8) | dt[0];
    y = (dt[3] << 8) | dt[2];
    z = (dt[5] << 8) | dt[4];

    // 1 Dps = 16 LSB therfore to get to DPS divide by 16
    gr->x = (double)x / 16;
    gr->y = (double)y / 16;
    gr->z = (double)z / 16;
}

// Change fusion mode function
void BNO055::change_fusion_mode(uint8_t mode) 
{
    dt[0] = BNO055_OPR_MODE;
    dt[1] = BNO055_OPR_MODE_CONFIGMODE;
    // Go to Config mode
    _i2c.write(chip_addr, dt, 2, false);
    ThisThread::sleep_for(19);
    dt[1] = mode;
    // Go to Requested mode
    _i2c.write(chip_addr, dt, 2, false);
    ThisThread::sleep_for(7);
}

IMU::EulerAngles BNO055::read()
{
    get_angular_position_quat(&imuAngles);
    return imuAngles;
}

IMU::EulerAngles BNO055::getImuAngles()
{
    return imuAngles;
}

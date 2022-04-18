#ifndef I2CIMU_H_
#define I2CIMU_H_
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "math.h"

#define MPU_ADDRESS 0xD0
#define MPU_CONFIG_ADDRRESS  0x1A
#define POWER_CONFIG_ADDRESS 0x6B
#define GYRO_CONFIG_ADDRESS 0x1B
#define ACC_CONFIG_ADDRESS 0x1C
#define ACC_DATA_START_ADDRESS  0x3B
#define GYRO_DATA_START_ADDRESS  0x43
#define GYRO_SCALE  65.5
#define I2C_TIMOUT  100
class I2CIMU{

public:
                  //filtered gyro rates for first PID loop
float             gyro_pitch_rate_pid = 0, gyro_roll_rate_pid = 0,  
                  gyro_yaw_rate_pid = 0 ; 
                  //filtered absolute angles
float             angle_pitch, angle_roll; 

private:
I2C_HandleTypeDef hi2c1;
float             gyro_scale; 
float             gyro_scale_pi; 
uint8_t           i2c_data[14];
float             gyro_axis[3] = {0};
long long         gyro_axis_cal[3] = {0};
float             acc_axis[3]  = {0};
float             acc_axis_cal[3]  = {0};
float             acc_pitch,acc_roll,acc_total_vector;
bool              gyro_angle_set = false;
void              (*err_handler)(void);
// float gyro_pitch,gyro_roll,gyro_yaw;



public:
static I2CIMU&    getInstance();
void              init(float controller_freq);
void              updateData();
void              reset();
void              calibrateSensor(int cal_samples);
void              setI2cErrorHandler(void (*err_handler)(void));
                  I2CIMU(I2CIMU& other) = delete; //Singletons should not be cloneable

private:
        I2CIMU();
void    initMPU6050();

};



void I2CIMU::setI2cErrorHandler(void (*err_handler)(void)){
    this->err_handler = err_handler;
}

I2CIMU::I2CIMU(){

}

I2CIMU& I2CIMU::getInstance(){
    static I2CIMU instance;
    return instance;
}


void I2CIMU::init(float controller_freq){
    //init gyro scale
    gyro_scale  = 1.0f/(controller_freq *  GYRO_SCALE);
    gyro_scale_pi  = gyro_scale * M_PI / 180.0f; // scale gyro val, mult by dt and convert to radians

    // init I2C1 peripheral
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c1) ;

    // setup the MPU6050 by setting the control registers    
    uint8_t  mpu6050_config   = 0x03;
    uint8_t  gyro_config      = 0x08; // 65.5
    uint8_t  acc_config       = 0x10; // -+8g
    uint8_t  power_config     = 0x00; // TODO: maybe turn off the temp sensor with 0x08 config value
    // configure the power
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, POWER_CONFIG_ADDRESS, 1,&power_config, 1, I2C_TIMOUT);
	// configure the mpu6050,
    //TODO: try other higher DLPF params for less noise
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, MPU_CONFIG_ADDRRESS, 1, &mpu6050_config, 1, I2C_TIMOUT);
	// configure the gyro, 
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, GYRO_CONFIG_ADDRESS, 1, &gyro_config, 1, I2C_TIMOUT);
	// configure the accelerometer 
	if(
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, ACC_CONFIG_ADDRESS, 1, &acc_config, 1, I2C_TIMOUT) != HAL_OK
    ) err_handler(); // check last I2C write for timout errors

}

//reset all output vals to zero
void I2CIMU::reset(){
	angle_pitch = angle_roll =  0;
	gyro_pitch_rate_pid = gyro_roll_rate_pid = gyro_yaw_rate_pid = 0;
}

void I2CIMU::calibrateSensor(int cal_samples){
    uint8_t gyro_data[6];

    //reset cal vals in case this is another calibration
	gyro_axis_cal[0] = gyro_axis_cal[1] = gyro_axis_cal[2] = 0 ;
	acc_axis_cal[0] = acc_axis_cal[1] = acc_axis_cal[2] = 0 ;

	//Accelerometer calibration
	for (uint16_t i = 0; i < cal_samples; i++) {
		// read 6 bytes of data starting from GYRO_DATA_START_ADDRESS
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, GYRO_DATA_START_ADDRESS, 1, gyro_data, 6, I2C_TIMOUT);
		gyro_axis_cal[0] += (short)(gyro_data[0] << 8 | gyro_data[1]);
		gyro_axis_cal[1] += (short)(gyro_data[2] << 8 | gyro_data[3]);
		gyro_axis_cal[2] += (short)(gyro_data[4] << 8 | gyro_data[5]);
	}
	gyro_axis_cal[0] = (int)((float) gyro_axis_cal[0] / cal_samples);
	gyro_axis_cal[1] = (int)((float) gyro_axis_cal[1] / cal_samples);
	gyro_axis_cal[2] = (int)((float) gyro_axis_cal[2] / cal_samples);

	//Accelerometer calibration
    uint8_t acc_data[6];
	for (uint16_t i = 0; i < cal_samples; i++) {
		// read 6 bytes of data starting from ACC_DATA_START_ADDRESS
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, ACC_DATA_START_ADDRESS, 1, acc_data, 6, I2C_TIMOUT);
		acc_axis_cal[0] += (short)(acc_data[0] << 8 | acc_data[1]);
		acc_axis_cal[1] += (short)(acc_data[2] << 8 | acc_data[3]);
		acc_axis_cal[2] += (short)(acc_data[4] << 8 | acc_data[5]);
	}
	acc_axis_cal[0] = (int)((float) acc_axis_cal[0] / cal_samples);
	acc_axis_cal[1] = (int)((float) acc_axis_cal[1] / cal_samples);
	acc_axis_cal[2] = (int)((float) acc_axis_cal[2] / cal_samples);
}

void I2CIMU::updateData(void){

	//read and store acc data, cast values to shorts since they are 16-bit 2's comp values
	if( HAL_I2C_Mem_Read(   &hi2c1, 
                            MPU_ADDRESS, 
                            ACC_DATA_START_ADDRESS,
                            1,
                            i2c_data,
                            14, 
                            I2C_TIMOUT) == HAL_OK)
                            err_handler(); // check/handle last for timout errors

	acc_axis[0] = (short)(i2c_data[0] << 8 | i2c_data[1]) ; //pitch
	acc_axis[1] = (short)(i2c_data[2] << 8 | i2c_data[3]) ; //roll
	acc_axis[2] = (short)(i2c_data[4] << 8 | i2c_data[5]) ; //yaw

     //acc calibration.
     acc_axis[0] -= acc_axis_cal[0];  //Accelerometer calibration value for pitch.
     acc_axis[1] -= acc_axis_cal[1];   //Accelerometer calibration value for roll.


	// ignore temperature data

	//read and store gyro data (NOT-scaled)
 	gyro_axis[0] = (short)(i2c_data[8]  << 8 | i2c_data[9])  ; //pitch
 	gyro_axis[1] = (short)(i2c_data[10] << 8 | i2c_data[11]) ; //roll
 	gyro_axis[2] = (short)(i2c_data[12] << 8 | i2c_data[13]) ; //yaw

	gyro_axis[0] -= gyro_axis_cal[0]; //pitch
	gyro_axis[1] -= gyro_axis_cal[1]; //roll
	gyro_axis[2] -= gyro_axis_cal[2]; //yaw


    //filter gyro rates to be used by the first PID controller 
    gyro_pitch_rate_pid = (gyro_pitch_rate_pid * 0.7f) + ((gyro_axis[0] / GYRO_SCALE) * 0.3f);   //Gyro pid input is deg/sec.
    gyro_roll_rate_pid  = (gyro_roll_rate_pid * 0.7f) + ((gyro_axis[1] / GYRO_SCALE) * 0.3f);   //Gyro pid input is deg/sec.
    gyro_yaw_rate_pid   = (gyro_yaw_rate_pid * 0.7f) + ((gyro_axis[2] / GYRO_SCALE) * 0.3f);   //Gyro pid input is deg/sec.

    //integrate gyro
    angle_pitch += gyro_axis[0] * gyro_scale;      
    angle_roll  += gyro_axis[1] * gyro_scale;      

    // If the IMU has yawed transfer the roll angle to the pitch angel and  the pitch angle to the roll angel
    // NOTE: this is disabled as it affects the drone badly
    // angle_pitch -= angle_roll * sin(gyro_axis[2] * gyro_scale_pi);
    // angle_roll += angle_pitch * sin(gyro_axis[2] * gyro_scale_pi);


    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_axis[0]*acc_axis[0])+(acc_axis[1]*acc_axis[1])+(acc_axis[2]*acc_axis[2]));       //Calculate the total accelerometer vector.
    
    if(abs(acc_axis[1]) < acc_total_vector){ //Prevent the asin function to produce a NaN
        acc_pitch = asin((float)acc_axis[1]/acc_total_vector)* 57.296;  //Calculate the pitch angle.
    }
    if(abs(acc_axis[0]) < acc_total_vector){  //Prevent the asin function to produce a NaN
        acc_roll = asin((float)acc_axis[0]/acc_total_vector)* -57.296;  //Calculate the roll angle.
    }
  
    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    //TODO: add acc calibrated angle_val here (so maybe run few cycles for this as well!)
    acc_pitch -= 0.0f;  //Accelerometer calibration value for pitch.
    acc_roll -= 0.0f;   //Accelerometer calibration value for roll.
    
    angle_pitch = angle_pitch * 0.99 + acc_pitch * 0.01; //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.99 + acc_roll * 0.01;    //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    //init absolute angles with accelerometer reading
    if (!gyro_angle_set){
        angle_pitch = acc_pitch;
        angle_roll  = acc_roll;
        gyro_angle_set = true;
    }
    
}


#endif 

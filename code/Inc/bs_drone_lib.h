/**
  ******************************************************************************
  * File Name          : 
  * Description        : 
  ******************************************************************************
  *
  ******************************************************************************
  */

#ifndef __BS_DRONE_LIB_H
#define __BS_DRONE_LIB_H



#define ACCELEROMETER_SENSITIVITY   4096.0f  
#define GYROSCOPE_SENSITIVITY       16.4f  
#define Compass_SENSITIVITY       	1090.0f
#define M_PIf       								3.14159265358979323846f
#define M_PI                        M_PIf    
#define sampleFreq                  200.0f // 500 hz sample rate!   
#define period_dt                   0.005f // 500 hz sample rate!   
#define limmit_I                    300.0f    

#define acc_compensation 0.2f
#define acc_compensation_ground 2.0f
#define spi_buffer_size 96

#define LED_R_on()  //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_off() //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)	
#define LED_L_on()  //HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET)
#define LED_L_off() //HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_RESET)

#define ARM_MATH_CM0

#include "stm32f0xx_hal.h"
//#include "usb_device.h"
#include "arm_math.h" 
#include "math.h" 
#include "MPU6050.h"
//#include "usbd_cdc_if.h"

volatile uint8_t _Sampling_task_do = 0;

volatile uint8_t Mode = calibation_mode;

int16_t rawAccx_X = 1;
int16_t rawAccx_Y = 1;
int16_t rawAccx_Z = 1;
int16_t rawGyrox_X = 1;
int16_t rawGyrox_Y = 1;
int16_t rawGyrox_Z = 1;

typedef struct
{
  __IO float X2;
  __IO float X1;
  __IO float Y2;
  __IO float Y1;
  __IO float Y0;
} filted_data;

filted_data filed_yaw;
filted_data filed_pitch;

volatile uint8_t Flag_setPID_gain_success = 0;
volatile int16_t magneticDeclination = 0;
volatile float Ref_yaw=0, Ref_pitch=0, Ref_roll=0 ;
volatile int16_t q_yaw, q_pitch, q_roll;                               		// States value
volatile float beta = 0.3f   ;
volatile float q0=1, q1=0, q2=0, q3=0;
volatile float T_center =0, yaw_center=0;
volatile float Error_yaw=0, Errer_pitch=0, Error_roll=0; 									//States Error
volatile float Sum_Error_yaw=0, Sum_Error_pitch=0, Sum_Error_roll=0;     	// Sum of error
volatile float D_Error_yaw=0, D_Error_pitch=0, D_Error_roll=0; 						// error dot
volatile float Del_yaw=0, Del_pitch=0, Del_roll=0;												// Delta states value for rotate axis
volatile float t_compensate = 0;
volatile float T_center_minus = 0;
volatile float y_roll=0, y_pitch=0, y0_roll=0, y0_pitch=0 ; 
volatile float rMat[3][3] = {0};

volatile float ref_t= -30.5;
//87.0
volatile float Kp_roll = 50.0;
volatile float Ki_roll = 0.5000000;
// 4.8
volatile float Kd_roll = 2.800000;

volatile float Kp_pitch = 1;
volatile float Ki_pitch = 0;
volatile float Kd_pitch = 0;

volatile float Kp_yaw = 0;
volatile float Ki_yaw = 0;
volatile float Kd_yaw = 0;

volatile float Kp_flip = 0;
volatile float Kd_flip = 0;
		

// set gyro offset
int16_t gx_diff = 0;
int16_t gy_diff = 0;
int16_t gz_diff = 0;
int16_t ax_diff = 0;
int16_t ay_diff = 0;
int16_t az_diff = 0;

volatile uint8_t  calibation_pass = 0;
volatile float  Acc_start = ACCELEROMETER_SENSITIVITY;

volatile uint8_t  spi_rx_data[spi_buffer_size] = {0};
volatile uint8_t  spi_rx_data_index = 0;

uint16_t watchdog  = 0;
int16_t xxx = 0;

/* USER CODE for Receiver  */
volatile uint8_t _index = 0 ;
volatile float	ch1=0,ch2=0,ch3=0,ch4=0; 
volatile int16_t	_ch1=0,_ch2=0,_ch3=0,_ch4=0;      
volatile int16_t	motor_A=0, motor_B=0, motor_C=0, motor_D=0 ;// Motors output value 
uint8_t rx_tmp[14] = {0};
float T_center_buffer = 0;
volatile float a, b, c, d, e, f;

volatile float gyx_d = 1000;
volatile float gyy_d = 1000;
volatile float gyz_d = 1000;
volatile float acx_d = 1000;
volatile float acy_d = 1000;

float _gyx_d = INT16_MAX;
float _gyy_d = INT16_MAX;
float _gyz_d = INT16_MAX;
float _acx_d = INT16_MAX;
float _acy_d = INT16_MAX;
int16_t count;

void Function1_call(void);
void Function2_call(void);

void Initial_MPU6050(void);
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void Read_MPU6050(void);
void PID_controller(void);
void Drive_motor_output(void);
void Sampling_task(void);
void AHRS(void);
float Smooth_filter(float alfa, float new_data, float prev_data);
float invSqrt(float x) ;
void imuComputeRotationMatrix(void);
float sq(float x);
float constrain(float x, float lower_b, float upper_b);
void getPIDgain(uint8_t spi_rx_data_index);
void getRCcommand(uint8_t spi_rx_data_index);
void readCommand (void);
float abs_user(float x);
uint32_t milli(void);
void calibation_fn(void);
void stabilize_fn(void);
float Butterworth_filter(filted_data* filted, float x_data);
void SET_filter_value(filted_data *filted, float value);

void Sampling_task(void)
{
	stabilize_fn();				
}

void init_mode_pin()
{
GPIO_InitTypeDef GPIO_InitStruct;

/*Configure GPIO pin : Pin_mode_sw_Pin */
GPIO_InitStruct.Pin = GPIO_PIN_14;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
	
void calibation_fn(void)
{
	/*Read_MPU6050();
	count++;
	if(count >= 10)
	{
		count = 0;
		//HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	}

	float gx_x = abs_user(rawGyrox_X-gyx_d);
	float gy_x = abs_user(rawGyrox_Y-gyy_d);	
	float gz_x = abs_user(rawGyrox_Z-gyz_d);		
	float ax_x = abs_user(rawAccx_X-acx_d);
	float ay_x = abs_user(rawAccx_Y-acy_d);

	const	float deadband = 2;

	if ((gx_x < deadband)&&(gy_x < deadband)&&(gz_x < deadband)&&(ax_x < deadband)&&(ay_x < deadband))
	{
		Mode = stabilize_mode;
		calibation_pass = 1;
		
		gx_diff = gyx_d;
		gy_diff = gyy_d;
		gz_diff = gyz_d;
		ax_diff = acx_d;
		ay_diff = acy_d;

		//HAL_SPI_Receive_IT(&hspi1, (uint8_t*)(spi_rx_data + spi_rx_data_index), 1);

		//LED_R_off();
	}

	_gyx_d = gyx_d;
	_gyy_d = gyy_d;
	_gyz_d = gyz_d;
	_acx_d = acx_d; 
	_acy_d = acy_d;
	
	*/
}

void stabilize_fn(void)
{
	static int8_t ld_blink;
	
//	HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);
	
	ld_blink++;
	if(ld_blink >= 20)
	{
		ld_blink = 0;
		//HAL_GPIO_TogglePin(LED_L_GPIO_Port, LED_L_Pin);
	}
	
	Read_MPU6050();
	
	AHRS(); 
	
	//PID_controller();

	if (watchdog > 0) watchdog --;
	
	if((T_center < 30) || (watchdog == 0))		
	{		
		motor_A=0;
		motor_B=0;
		motor_C=0;
		motor_D=0;
	}

	//Drive_motor_output(); 
	
//	HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_RESET);
}

void Initial_MPU6050(void)
{
	HAL_Delay(150); // for stability
	//    Reset to defalt 
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, ENABLE);
	HAL_Delay(150);
	
	//    ENABLE
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, DISABLE);
	HAL_Delay(1);
	//	  SetClockSource(MPU6050_CLOCK_PLL_ZGYRO)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_ZGYRO);
	HAL_Delay(1);		

	//    SetFullScaleAccelRange(MPU6050_ACCEL_FS_8)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_ACCEL_FS_8);
	HAL_Delay(1);
	
	//    SetFullScaleGyroRange(MPU6050_GYRO_FS_2000)
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);
	HAL_Delay(1);
	
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_10);
	HAL_Delay(1);
			
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 7, 8, 5);
	HAL_Delay(1);

	//    interupt(Enable)
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
	HAL_Delay(1);
				
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;   // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data;    // combine data with existing byte
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    HAL_I2C_Mem_Write(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1, slaveAddr, regAddr, 1, &tmp, 1, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
	  HAL_I2C_Mem_Read(&hi2c1,slaveAddr,regAddr,1,&tmp,1,1);
    *data = tmp & (1 << bitNum);
}


void Read_MPU6050(void)
{
	int16_t AccelGyro[6]={0};  
    uint8_t tmpBuffer[14];
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,1,tmpBuffer,14,2);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
				
	rawAccx_X  =  AccelGyro[0];
	rawAccx_Y  =  AccelGyro[1];
	rawAccx_Z  =  AccelGyro[2];
	rawGyrox_X =  AccelGyro[3];
	rawGyrox_Y =  AccelGyro[4];
	rawGyrox_Z =  AccelGyro[5];

	//beta = acc_compensation;
	
	//if (T_center < 40) 
	//{
	//	beta =  acc_compensation_ground;			
	//}
	
	if (Mode == calibation_mode) 
	{
		gyx_d = Smooth_filter(0.005f, rawGyrox_X, gyx_d);
		gyy_d = Smooth_filter(0.005f, rawGyrox_Y, gyy_d);	
		gyz_d = Smooth_filter(0.005f, rawGyrox_Z, gyz_d);
		acx_d = Smooth_filter(0.005f, rawAccx_X, acx_d);
		acy_d = Smooth_filter(0.005f, rawAccx_Y, acy_d);	
		Acc_start = Smooth_filter(0.1f, (float)rawAccx_Z/ACCELEROMETER_SENSITIVITY, Acc_start);		
		
	}else{
	
		rawGyrox_X -= gx_diff;
		rawGyrox_Y -= gy_diff;
		rawGyrox_Z -= gz_diff;
		
		rawAccx_X -= ax_diff;
		rawAccx_Y -= ay_diff;
	}
}

void PID_controller(void)
{

	//float Buf_D_Error_yaw   =Error_yaw;
	//float Buf_D_Errer_pitch =Errer_pitch;
	float Buf_D_Error_roll  =Error_roll; 
     
	
	//static float cal_pitch ;
	static float cal_roll  ;
		
	//cal_pitch = Smooth_filter(0.95f, (float)q_pitch*0.1f, cal_pitch);
	//cal_roll = Smooth_filter(0.95f, (float)q_roll*0.1f, cal_roll);
	
	
	//T_center_buffer    = constrain((float)ch3 * 20.0f * flip_Jump_gain, 0.0f, 2200.0f);
	
	//T_center = Smooth_filter(0.6f, T_center_buffer, T_center);

	//Error_yaw 	=  -(float)ch4 * 3.0f - ((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY;
	//Errer_pitch =  -(float)0 * 0.3f - ((float)cal_pitch)	;
	Error_roll 	=  ((float)ref_t+((float)ch2/4.00) )+ ((float)q_roll)	;
	if(abs_user(Error_roll)>3.00f){
		Kp_roll = 87.00f;
		Kd_roll = 3.8f;
	}else{
		Kp_roll = 50.0f;
		Kd_roll = 2.8f;
	}
	if (abs_user(Del_roll) > 150)
	{
		//Sum_Error_yaw =   constrain((Sum_Error_yaw   + (Error_yaw   /sampleFreq)), -100.0f , 100.0f) ;
		//Sum_Error_pitch = constrain((Sum_Error_pitch + (Errer_pitch /sampleFreq)), -100.0f , 100.0f) ;
		Sum_Error_roll =  constrain((Sum_Error_roll  + (Error_roll  /sampleFreq)), -30.0f , 30.0f) ;
	}

	//D_Error_yaw   = (Error_yaw-Buf_D_Error_yaw)*sampleFreq ;
	//D_Error_pitch = Butterworth_filter(&filed_pitch,(Errer_pitch-Buf_D_Errer_pitch)*sampleFreq);
	D_Error_roll  = Butterworth_filter(&filed_yaw,(Error_roll-Buf_D_Error_roll)*sampleFreq);

	//Del_yaw		= (Kp_yaw   * Error_yaw)	+ (Ki_yaw	  * Sum_Error_yaw) + constrain((Kd_yaw * D_Error_yaw), -1500, 1500);
	//Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch) + constrain((Kd_pitch * D_Error_pitch), -1500, 1500);
	Del_roll	= (Kp_roll  * Error_roll)	+ (Ki_roll	* Sum_Error_roll)  + constrain((Kd_roll * D_Error_roll), -3000, 3000);

	
	//motor_A = T_center +Del_pitch	+Del_roll -Del_yaw;
	//motor_B = T_center +Del_pitch	-Del_roll +Del_yaw;
	//motor_C = T_center -Del_pitch	-Del_roll -Del_yaw;
	//motor_D = T_center -Del_pitch	+Del_roll +Del_yaw;	
	
// for v929
//	motor_A = T_center +Del_pitch	+Del_roll +Del_yaw;
//	motor_B = T_center +Del_pitch	-Del_roll -Del_yaw;
//	motor_C = T_center -Del_pitch	-Del_roll +Del_yaw;
//	motor_D = T_center -Del_pitch	+Del_roll -Del_yaw;	
	
	
//	Del_yaw		= (Kp_yaw   * Error_yaw)		+ (Ki_yaw	  * Sum_Error_yaw)   + constrain((Kd_yaw * D_Error_yaw), -1000, 1000);
//	Del_pitch	= (Kp_pitch * Errer_pitch)	+ (Ki_pitch	* Sum_Error_pitch) + constrain((Kd_pitch * D_Error_pitch), -1000, 1000);
//	Del_roll	= (Kp_roll  * Error_roll)		+ (Ki_roll	* Sum_Error_roll)  + constrain((Kd_roll * D_Error_roll), -1000, 1000);

//	motor_A = T_center + constrain((+Del_pitch	+Del_roll +Del_yaw), -1250, 1250);
//	motor_B = T_center + constrain((+Del_pitch	-Del_roll -Del_yaw), -1250, 1250) ;
//	motor_C = T_center + constrain((-Del_pitch	-Del_roll +Del_yaw), -1250, 1250) ;
//	motor_D = T_center + constrain((-Del_pitch	+Del_roll -Del_yaw), -1250, 1250) ;

}

void Drive_motor_output(void)
{
	//motor_A = 100 ;
	//motor_B = 100 ;
	//motor_C = 100 ;
	//motor_D = 100 ;
	
	// limmit output max, min
	/*if(motor_A < 0) motor_A = 0 ;
	if(motor_B < 0) motor_B = 0 ;
	if(motor_C < 0) motor_C = 0 ;
	if(motor_D < 0) motor_D = 0 ;
	
	if(motor_A > 2399) motor_A = 2399 ;
	if(motor_B > 2399) motor_B = 2399 ;
	if(motor_C > 2399) motor_C = 2399 ;
	if(motor_D > 2399) motor_D = 2399 ;
	
	TIM2->CCR1 = motor_C ;
	TIM2->CCR2 = motor_D ;
	TIM3->CCR1 = motor_A ;
	TIM3->CCR2 = motor_B ; */

//	TIM2->CCR1 = constrain(motor_C, 0, 2399);
//	TIM2->CCR2 = constrain(motor_D, 0, 2399);
//	TIM3->CCR1 = constrain(motor_A, 0, 2399);
//	TIM3->CCR2 = constrain(motor_B, 0, 2399);
}

void AHRS(void)
{
	const float dt = period_dt; 	
	float gx = (((float)rawGyrox_X)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float gy = (((float)rawGyrox_Y)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float gz = (((float)rawGyrox_Z)/GYROSCOPE_SENSITIVITY)*(M_PIf/180.0f);
	float ax = ((float)rawAccx_X)/ACCELEROMETER_SENSITIVITY;
	float ay = ((float)rawAccx_Y)/ACCELEROMETER_SENSITIVITY;
	float az = ((float)rawAccx_Z)/ACCELEROMETER_SENSITIVITY;

	float recipNorm;
	float ex = 0, ey = 0, ez = 0;
	float qa, qb, qc;

	
    // Use measured acceleration vector
    recipNorm = sq(ax) + sq(ay) + sq(az);
    if (recipNorm > 0.01f && recipNorm < sq(Acc_start)*1.2f) 
		{
        // Normalise accelerometer measurement
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);

    }

    // Apply proportional and integral feedback
    gx += beta * ex;
    gy += beta * ey;
    gz += beta * ez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
		
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
		/* Compute pitch/roll angles */
  //  q_pitch =  -(atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    q_roll  =  -(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (1800.0f / M_PIf));
//    q_yaw   =  ((atan2f(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination));
//		if (q_yaw < 0) q_yaw += 3600;
}


float constrain(float x, float lower_b, float upper_b)
{
	if(x < lower_b) x = lower_b;
	if(x > upper_b) x = upper_b;
	return x;
}

float sq (float x)
 {
	 return x*x;
 }
 
void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3; 

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}
float Smooth_filter(float alfa, float new_data, float prev_data)
{
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}

float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}


void getPIDgain(uint8_t spi_rx_data_index)
{
	/*int16_t Kp_yaw_tmp = (int16_t)spi_rx_data[spi_rx_data_index-21]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-20]; 
	int16_t	Ki_yaw_tmp = (int16_t)spi_rx_data[spi_rx_data_index-19]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-18];
	int16_t	Kd_yaw_tmp = (int16_t)spi_rx_data[spi_rx_data_index-17]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-16];

	int16_t	Kp_pitch_tmp = (int16_t)spi_rx_data[spi_rx_data_index-15]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-14];
	int16_t	Ki_pitch_tmp = (int16_t)spi_rx_data[spi_rx_data_index-13]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-12];
	int16_t	Kd_pitch_tmp = (int16_t)spi_rx_data[spi_rx_data_index-11]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-10];

	int16_t	Kp_roll_tmp = (int16_t)spi_rx_data[spi_rx_data_index-9]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-8];
	int16_t	Ki_roll_tmp = (int16_t)spi_rx_data[spi_rx_data_index-7]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-6];
	int16_t	Kd_roll_tmp = (int16_t)spi_rx_data[spi_rx_data_index-5]<<8 | (int16_t)spi_rx_data[spi_rx_data_index-4];
	int16_t sum = (int16_t)Kp_roll_tmp + (int16_t)Ki_roll_tmp + (int16_t)Kd_roll_tmp + (int16_t)Kp_pitch_tmp + (int16_t)Ki_pitch_tmp + (int16_t)Kd_pitch_tmp + (int16_t)Kp_yaw_tmp + (int16_t)Ki_yaw_tmp + (int16_t)Kd_yaw_tmp;
	int16_t	checksum_buffer =(((int16_t)spi_rx_data[spi_rx_data_index-3])<<8 | (int16_t)spi_rx_data[spi_rx_data_index-2]);

	if (checksum_buffer == sum)
	{
		Kp_roll = (float)Kp_roll_tmp * 0.050f;
		Ki_roll = (float)Ki_roll_tmp * 0.025f;
		Kd_roll = (float)Kd_roll_tmp * 0.025f;

		Kp_pitch = (float)Kp_pitch_tmp * 0.050f;
		Ki_pitch = (float)Ki_pitch_tmp * 0.025f;
		Kd_pitch = (float)Kd_pitch_tmp * 0.025f;

//		Kp_yaw = (float)Kp_yaw_tmp * 0.025f;
//		Ki_yaw = (float)Ki_yaw_tmp * 0.025f;
//		Kd_yaw = (float)Kd_yaw_tmp * 0.020f;

		Kp_flip = (float)Kp_yaw_tmp * 0.060f;
		Kd_flip = (float)Kd_yaw_tmp * 0.020f;
		flip_Jump_time = Ki_yaw_tmp;
		
		if (Kd_yaw_tmp < 1)
		{
			Kp_yaw = (float)Kp_yaw_tmp * 0.025f;
			Ki_yaw = (float)Ki_yaw_tmp * 0.025f;
			Kd_yaw = (float)Kd_yaw_tmp * 0.020f;
			
		} else {
			
			Kp_flip = (float)Kp_yaw_tmp * 0.060f;
			Kd_flip = (float)Kd_yaw_tmp * 0.020f;
			flip_Jump_time = Ki_yaw_tmp;		
			
		}

		
		Flag_setPID_gain_success = 1;
		
	}else{
		
		Flag_setPID_gain_success = 0;
	}*/
}

void getRCcommand(uint8_t spi_rx_data_index)
{
	
	int8_t	roll_tmp     = (int8_t)spi_rx_data[spi_rx_data_index-4];  
	int8_t	pitch_tmp    = (int8_t)spi_rx_data[spi_rx_data_index-3]; 
	int8_t	throttle_tmp = (int8_t)spi_rx_data[spi_rx_data_index-2];  
	int8_t	yaw_tmp      = (int8_t)spi_rx_data[spi_rx_data_index-1];  
	int8_t	sum_tmp      = (int8_t)spi_rx_data[spi_rx_data_index];
	int8_t	sum  = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

	if ((int8_t)sum_tmp == (int8_t)sum)
	{
		ch1 = roll_tmp;
		ch2 = pitch_tmp;
		ch3 = throttle_tmp;
		ch4 = yaw_tmp;
		//watchdog = 500;
		//if(ch3 < 3 && calibation_pass == 1) Mode = stabilize_mode;
	}
}
 
void readCommand (void)
{ 
	if (spi_rx_data_index >= 6)
	{
		for (uint8_t count = spi_rx_data_index ; count >= 6 ; count--)
		{
			uint8_t command_code = 0xfd ;
			if (spi_rx_data[count-1] == command_code && spi_rx_data[count] == command_code )
			{
				getRCcommand(count);
				spi_rx_data_index = 0;
				break;
			}
			uint8_t command_code2 = 0xfe ;
			if(count >= 21)
			{
				if (spi_rx_data[count-1] == command_code2 && spi_rx_data[count] == command_code2)
				{
					getPIDgain(count);
					spi_rx_data_index = 0;
					break;
				} 
			}
		}
	} 
}

float abs_user(float x)
{
	if(x<0) x=-x;
	return x;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static uint8_t count, command_code;
	count++;
	
	if(count == 5)
	{
		count = 0;
		HAL_GPIO_TogglePin(GPIOB, LED3_Pin);
	}
	
	spi_rx_data_index = spi_rx_data_index + 1;	
	
	if (spi_rx_data_index >= 2)
	{		
		/*----------------------------------FUNCTION-----------------------------------*/
		command_code = 0xf1 ;
		if (spi_rx_data[spi_rx_data_index-2] == command_code && spi_rx_data[spi_rx_data_index-1] == command_code )
		{
			spi_rx_data_index = 0;
			Mode = Function_1_mode;
			_function1_lock = 1;
			
			Mode = Function_2_mode;  // force flip
			_function2_lock = 1;		
		}
		
		command_code = 0xf2 ;
		if (spi_rx_data[spi_rx_data_index-2] == command_code && spi_rx_data[spi_rx_data_index-1] == command_code )
		{
			spi_rx_data_index = 0;
			Mode = Function_2_mode;
			_function2_lock = 1;			
		}		
			
		if (spi_rx_data_index >= 7)
		{
			/*----------------------------------RC COMMAND-----------------------------------*/
			command_code = 0xfd ;
			if (spi_rx_data[spi_rx_data_index-2] == command_code && spi_rx_data[spi_rx_data_index-1] == command_code )
			{
				getRCcommand(spi_rx_data_index-3);
				spi_rx_data_index = 0;
			}

			if(spi_rx_data_index >= 22)
			{
				/*----------------------------------PID TUNING-----------------------------------*/
				command_code = 0xfe ;
				if (spi_rx_data[spi_rx_data_index-2] == command_code && spi_rx_data[spi_rx_data_index-1] == command_code)
				{
					getPIDgain(spi_rx_data_index-1);
					spi_rx_data_index = 0;
				} 
			}
		}
	}
	
	if (spi_rx_data_index == spi_buffer_size) spi_rx_data_index = 0;

	HAL_SPI_Receive_IT(&hspi1, (uint8_t*)(spi_rx_data+spi_rx_data_index), 1);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_Receive_IT(&hspi1, (uint8_t*)(spi_rx_data+spi_rx_data_index), 1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	_Sampling_task_do = 1;
}

uint32_t milli(void)
{
	return HAL_GetTick();
}
        
float Butterworth_filter(filted_data *filted, float x_data)
{
	// lowpass filter butterworth order 2nd fc  248 hz sampling 500hz

	filted->Y2 = filted->Y1;
	filted->Y1 = filted->Y0 ;
	
	float nume = (x_data + 2.0f * filted->X1 +  filted->X2);
	float denom = (1.9644605802052317f * filted->Y1 +  0.96508117389913461f * filted->Y2);
	filted->Y0 = 0.98238543852609161f * nume - denom;
	
	filted->X2 = filted->X1;
	filted->X1 = x_data;
	
	return filted->Y0;
}

void SET_filter_value(filted_data *filted, float value)
{
	// lowpass filter butterworth order 2nd fc  248 hz sampling 500hz

	filted->Y2 = value;
	filted->Y1 = value;
	filted->Y0 = value;
	filted->X2 = value;
	filted->X1 = value;

}
#endif

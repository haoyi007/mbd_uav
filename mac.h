#include "inttypes.h"
extern int time2;

extern short rcData[4];
typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
	float x;
	float y;
	float z;
}xyz_f_t;

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
		PID4,
		PID5,
		PID6,

		PIDITEMS
};
typedef struct
{
	xyz_f_t err;
	xyz_f_t err_old;
	xyz_f_t err_i;
	xyz_f_t eliminate_I;
	xyz_f_t err_d;
	xyz_f_t damp;
	xyz_f_t out;
	pid_t 	PID[PIDITEMS];
	xyz_f_t err_weight;
	float FB;

}ctrl_t;

extern ctrl_t ctrl_1;
extern ctrl_t ctrl_2;

void receiveData(uint16_t *rcDataTmp);

void I2C_InitGPIO(void);

void NRF24L01_SPI_Init(void);

void MPU6050_Init(void);

void MPU6050ReadAcc(short *accData);

void MPU6050ReadGyro(short *gyroData);

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);

void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);

void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);

void IMUSO3Thread(float gyro[],float acc[],float euler[]);

void ParamSetDefault(void);

void ANO_DT_Data_Exchange(void);

void ANO_DT_Data_Receive_Prepare(uint8_t data);

// PID结构体
typedef struct
{
    float P;
    float I;
    float D;
    float Desired;
    float Error;
    float PreError;
    float PrePreError;
    float Increment;
    float Integ;
	float iLimit;
    float Deriv;
	float last_derivative;
    float Output;
 
}PID_Typedef;
//RC遥控
typedef struct int16_rcget
{
    float ROLL;
    float PITCH;
    float THROTTLE;
    float YAW;
}RC_GETDATA;

typedef struct IMU_tt
{
    uint8_t caliPass;
    uint8_t ready;
    int16_t accADC[3];
    int16_t gyroADC[3];
    int16_t magADC[3];
    float 	accRaw[3];		//m/s^2
    float 	gyroRaw[3];		//rad/s
    float 	magRaw[3];		//
    float   accOffset[3];		//m/s^2
    float   gyroOffset[3];
    float   accb[3];		//filted, in body frame
    float   accg[3];
    float   gyro[3];
    float   DCMgb[3][3];
    float   q[4];
    float   roll;				//deg
    float   pitch;
    float 	yaw;
    float   rollRad;				//rad
    float   pitchRad;
    float 	yawRad;
} imu_t;

extern imu_t imu;
extern RC_GETDATA RC_DATA; 
extern float Thro,RollOut,PitchOut,YawOut;

enum {ROLL1,PITCH2,YAW3};//THROTTLE,
void controller(void);

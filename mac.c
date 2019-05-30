#include "inttypes.h"
#include "mac.h"
int time2;
short rcData[4]= {1500,1500,1500,1500};
ctrl_t ctrl_1;
ctrl_t ctrl_2;
void I2C_InitGPIO(void)
{

}

void MPU6050_Init(void)
{

}

void MPU6050ReadAcc(short *accData)
{

}

void MPU6050ReadGyro(short *gyroData)
{

}

void NRF24L01_SPI_Init(void)
{
	
}

void NRF24L01_RX_Mode(void)
{

}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{

}

void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	
}

void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	
}

void receiveData(uint16_t *rcDataTmp)
{
    rcDataTmp[0]=rcData[0];
	rcDataTmp[1]=rcData[1];
	rcDataTmp[2]=rcData[2];
	rcDataTmp[3]=rcData[3];
}

void IMUSO3Thread(float gyro[],float acc[],float euler[])
{
	
}

void ParamSetDefault(void)
{
	
}

void ANO_DT_Data_Exchange(void)
{
	
}

void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
	
}

RC_GETDATA  RC_DATA;
imu_t imu= {0};
float Thro=0,RollOut=0,PitchOut=0,YawOut=0;//这里这个是输出值

void controller(void)
{

}

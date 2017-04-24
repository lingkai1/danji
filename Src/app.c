#include "app.h"
#include "usart.h"
#include "tim.h"
#include "fatfs.h"
TxUnion txData;
extern MotorRxDataUnion motorRxData;
MotorRxDataUnion motorRxBuf[800];
MotorRxDataUnion *ptrMotor = motorRxBuf;

extern uint8_t motorRxbuffer[31];
// file write variable
extern FATFS SDFatFs;
long byteNum = 0;
long long totalWrite = 1;
extern FIL MyFile;
char floattemp[200];
int32_t temp = 0;
extern FRESULT res;      
uint32_t writeFileNum = 0; 
uint32_t writeCout = 0;  /* FatFs function common result code */
extern uint32_t byteswritten;// bytesread;                     /* File write/read counts */
int countRev = 0;


extern gpsDataType gpsData;
extern gpsDataType gpsDataBuf[100];
extern gpsDataType *Ptr;
extern long countGps;
int GPSWriteTime = 0;
int gpsFileName = 1000;
long long totalWriteGps = 0;
void sendData(int32_t *gpsData, uint16_t speed,uint16_t angle,uint8_t model)
{
	
	txData.x.head[0] = 0xAA;
	txData.x.head[1] = 0x55;
	txData.x.motor[0] = speed;
	txData.x.motor[1] = angle;
	//txData.x.motor[2] = angle;
	txData.x.gps[0] = gpsData[0];
	txData.x.gps[1] = gpsData[1];
	txData.x.gps[2] = temp++;
	txData.x.model = model;
	txData.x.check = uint8Sum(txData.y, 19); // 写校验位；
	HAL_UART_Transmit_IT(&huart4, txData.y, sizeof(txData.y));
}
uint8_t uint8Sum(uint8_t arr[], uint8_t length)
{
    uint8_t i,sum;
		sum = 0;
    for(i = 0; i < length; i++)
    {
        sum += arr[i];
    }
    return sum;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == htim3.Instance)/* 1ms interupt*/
 {
//		int32_t gpsData[3] = {1,2,3};
//		sendData(gpsData, 2, 3,1);
 }
 if (htim->Instance == htim4.Instance) //100ms interupt
 {	 	
				int32_t gpsData[3] = {1,2,3};
				sendData(gpsData, 0,0,0);
 }
}



void writeSD()
{
		//memcpy(motorRxData.y,motorRxbuffer,sizeof(motorRxbuffer));
		memcpy((*(ptrMotor + countRev)).y,motorRxData.y,30);
		countRev ++;
		if(countRev >= 500)
		{			
			int i = 0;
			writeCout ++;
			
			
			if(writeCout >= 60)
			{
				writeFileNum++; // 写新的文件 状态全部至零
				writeCout = 0;
				totalWrite = 0;
			}
			__disable_irq();
			
			
			char fileName[80];
			sprintf(fileName,"%d.txt",writeFileNum);
			f_open(&MyFile, fileName, FA_CREATE_ALWAYS | FA_WRITE); 
			f_lseek(&MyFile, totalWrite);
			for(i = 0; i < countRev;i++ )
			{
				//f_lseek(&MyFile, totalWrite);  加入时间判定
				byteNum = sprintf(floattemp,"%d,%d,%d,%f,%f,%f,%f,%f,%f,%f\n",gpsData.hour,gpsData.minute,gpsData.ms,motorRxBuf[i].x.MotorPara[0],motorRxBuf[i].x.MotorPara[1],
										motorRxBuf[i].x.MotorPara[2],motorRxBuf[i].x.MotorPara[3],motorRxBuf[i].x.MotorPara[4],
										motorRxBuf[i].x.MotorPara[5],motorRxBuf[i].x.MotorPara[6]);
				f_lseek(&MyFile, totalWrite);
				totalWrite += byteNum;
				f_write(&MyFile,floattemp,byteNum,(void *)&byteswritten);	
			}
			f_close(&MyFile);
			__enable_irq();
			countRev = 0;
		}
}

void writeGPS()
{
	int i;
	char fileName[80];
	__disable_irq();
	sprintf(fileName,"%d.txt",gpsFileName);
	f_open(&MyFile, fileName, FA_CREATE_ALWAYS | FA_WRITE); 
	for(i = 0;i<countGps;i++)
	{
		byteNum = sprintf(floattemp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",gpsDataBuf[i].longitude,gpsDataBuf[i].latitude
		,gpsDataBuf[i].altitude,gpsDataBuf[i].northSpeed,gpsDataBuf[i].upSpeed,gpsDataBuf[i].eastSpeed,gpsDataBuf[i].starNumber,gpsDataBuf[i].PDOP
		,gpsDataBuf[i].GDOP,gpsDataBuf[i].year,gpsDataBuf[i].month,gpsDataBuf[i].day,gpsDataBuf[i].hour,gpsDataBuf[i].minute,gpsDataBuf[i].second,
		gpsDataBuf[i].ms);
		f_lseek(&MyFile, totalWriteGps);
		totalWriteGps += byteNum;
		f_write(&MyFile,floattemp,byteNum,(void *)&byteswritten);	
	}
	f_close(&MyFile);
	__enable_irq();
	GPSWriteTime ++;
	if(GPSWriteTime >= 99)
	{
		GPSWriteTime = 0;
		totalWriteGps = 0;
		gpsFileName ++;
	}
}

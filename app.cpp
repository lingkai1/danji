#include "app.h"
#include "usart.h"
void sendData(int32_t *gpsData, float speed, float angle,char model)
{
	TxUnion txData;
	txData.x.head[0] = 0xAA;
	txData.x.head[0] = 0x55;
	txData.x.motor[0] = speed;
	txData.x.motor[1] = angle;
	
	txData.x.gps[0] = gpsData[0];
	txData.x.gps[1] = gpsData[1];
	txData.x.gps[2] = gpsData[2];
	txData.x.model = model;
	txData.y[23] = uint8Sum(txData.y, 23); // Ð´Ð£ÑéÎ»£»
	HAL_UART_Transmit_IT(&huart4, txData.y, sizeof(txData.y));
}
uint8_t uint8Sum(uint8_t arr[], uint8_t length)
{
    uint8_t i,sum;
		uint8_t checksum;
		sum = 0;
    for(i = 0; i < length; i++)
    {
        sum += arr[i];
    }
		checksum = sum;
    return sum;
}
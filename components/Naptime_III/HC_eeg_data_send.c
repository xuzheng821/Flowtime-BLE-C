#include "HC_eeg_data_send.h"

ble_eeg_t               m_eeg;                                     /**< Structure used to identify the heart rate service. */

/*******************数据吞吐量测试*********************/
uint32_t Num_Time;       //每一帧数据包头2个字节，不断累加
uint8_t send_num = 0;    //一秒发送50帧数据，记录发送到第几帧数据
uint8_t Send_Flag = 0;   //第一帧数据发送完成
uint8_t Data_send[17];   //发送数据缓存
extern uint8_t EEG_DATA_SEND[750];
extern bool Global_connected_state;
extern bool ads1291_is_init;                  //1291初始化标志位

//调用该函数发送第一帧数据
uint32_t ble_send_data(uint8_t *pdata)
{
		uint32_t err_code;
    send_num = 0;
	
		Data_send[0] = Num_Time >> 8;    //添加包头--2个字节
		Data_send[1] = Num_Time & 0xFF;

	  for(uint8_t i = 0; i < 15 ; i++)
		{
			Data_send[i+2] = *(pdata+i);
		}
			 
		err_code = ble_EEG_DATA_send(&m_eeg, Data_send, 17);   //数据发送，长度17字节
		if (NRF_SUCCESS == err_code)
		{
			 send_num++;	
			 Num_Time++;		
			 Send_Flag = 1;  //该帧数据发送完成会触发BLE_EVT_TX_comPLETE,在main函数中通过该标志位判断是否继续发送剩余数据
		}
		return err_code;
}

//这个函数完成后续数据发送,放在BLE_EVT_TX_comPLETE时间处理函数中
void ble_send_more_data(uint8_t *pdata)
{
	uint32_t err_code;

	do{
			 if (send_num > 49)  //一秒数据全部发送完成,相关标志位复位
			 {
					send_num = 0x00;
					Send_Flag = 0;
					memset(&EEG_DATA_SEND,0,sizeof(EEG_DATA_SEND));
					return; 
			 }

			 Data_send[0] = Num_Time >> 8;
			 Data_send[1] = Num_Time & 0xFF;

			 for(uint8_t i = 0; i < 15 ;i++)
			 {
					Data_send[i+2]=*(pdata + send_num * 15 + i);
			 }
			 
			 err_code = ble_EEG_DATA_send(&m_eeg,Data_send, 17 );
			 if (NRF_SUCCESS == err_code)
			 {
					send_num++;	
					Num_Time++;		
	   }
	 }while(err_code != BLE_ERROR_NO_TX_PACKETS && Global_connected_state);
}

void ble_state_send(uint8_t pdata)
{
   uint32_t err_code;
	 do{
		 err_code = ble_EEG_ELE_STATE_send(&m_eeg,pdata, 1);
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"\r eeg_state_send:%x pdata:%x \r\n",err_code,pdata);
		 }
		 }while(err_code == BLE_ERROR_NO_TX_PACKETS && Global_connected_state && ads1291_is_init);
}

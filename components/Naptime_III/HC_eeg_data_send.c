#include "HC_eeg_data_send.h"
#include "HC_ads129x_driver.h"
#include "sdk_errors.h"

ble_EEG_t                         m_EEG;                                     /**< Structure used to identify the heart rate service. */

/*******************数据吞吐量测试*********************/
uint16_t Num_Time;
uint8_t send_num = 0;
uint8_t Send_Flag = 0;
uint8_t Data_send[17];
extern uint8_t ADCData2[750];

//发送数据时调用这个函数，传入buff和长度
uint32_t ble_send_data(uint8_t *pdata, uint8_t Send_Len)
{
	uint32_t err_code;
	int i;
  send_num = 0x00;

	for(i = 0; i < 15 ;i++)
	{
		Data_send[i+2]=*(pdata+i);
	}
	Data_send[0] = Num_Time >> 8;
	Data_send[1] = Num_Time & 0xFF;
    
	err_code = ble_EEG_DATA_send(&m_EEG, Data_send, 17);
	if (NRF_SUCCESS == err_code)
	{
    send_num++;	
    Num_Time++;		
	  Send_Flag = 1;
	}
	return err_code;
}

//这个函数完成后续数据发送,放在BLE_EVT_TX_comPLETE时间处理函数中
void ble_send_more_data(uint8_t *pdata, uint8_t Send_Len)
{
	uint32_t err_code;
  int i;
	do{
	   if (send_num > 49)
	   { 
	    	send_num = 0x00;
			  Send_Flag = 0;
			  memset(&ADCData2,0,sizeof(ADCData2));
		    return; 
	   }//数据全部发送完成

	   for(i = 0; i < 15 ;i++)
	   {
	    	Data_send[i+2]=*(pdata + send_num * 15 + i);
	   }
	   Data_send[0] = Num_Time >> 8;
	   Data_send[1] = Num_Time & 0xFF;

	   err_code = ble_EEG_DATA_send(&m_EEG,Data_send, 17 );
	   if (NRF_SUCCESS == err_code)
	   {
	    	send_num++;	
        Num_Time++;		
	   }
	 }while(err_code != BLE_ERROR_NO_TX_PACKETS);
}

void ble_state_send(uint8_t pdata)
{
   uint32_t err_code;
	 err_code = ble_EEG_ELE_STATE_send(&m_EEG,pdata, 1);
	 APP_ERROR_CHECK(err_code);
}

#include "HC_command_analysis.h"
#include "ble_conn.h"
#include "HC_data_flash.h"
#include "HC_random_number.h"
#include "HC_battery.h"
#include "HC_led.h"
#include "HC_timer.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

ble_conn_t                         m_conn;
uint16_t                           m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

bool ID_is_change  = false;
bool ID_is_receive = false;
bool Is_white_adv  = false;
bool Is_device_bond = false;  

uint8_t ID_receive_buff[4] = {0};
extern uint8_t User_ID[4];             
extern uint8_t Global_connected_state;

uint8_t Handshark_buff1[3];
uint8_t Handshark_buff2[5];
uint8_t Handshark_buff3[3];
uint8_t communocate_state[5] = {0x04,0x00,0x00,0x00,0xFF};

void ble_Com_ID_Analysis(uint8_t * p_data, uint16_t length)
{
	uint32_t err_code;
	ID_is_receive = false;
  communocate_state[4] = 0xFF;
  memset(Handshark_buff1, 0, sizeof(Handshark_buff1));
  memset(Handshark_buff2, 0, sizeof(Handshark_buff2));
  memset(Handshark_buff3, 0, sizeof(Handshark_buff3));
		
	if( *p_data == 0x00  && length == 5)
	{
     memcpy(ID_receive_buff,(p_data+1), 4);
 
		 if(Is_device_bond)        
     {	
			 if(Is_white_adv)
			 {
					if((memcmp(ID_receive_buff, User_ID, sizeof(User_ID)) == 0))  //if相等,=0
					{
							SEGGER_RTT_printf(0,"ID_is_receive \r\n");		
							ID_is_receive = true;
							ID_is_change	= false;					 
					}
					else
					{
							communocate_state[4] = 0x02;                       //设备已被绑定
							ble_State_string_send(&m_conn,communocate_state,5);
						  nrf_delay_ms(500);
							if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
					    {
								   err_code = sd_ble_gap_disconnect(m_conn_handle,
         			                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
								   APP_ERROR_CHECK(err_code);
					    }
					}
			 }
			 else  //绑定，普通广播
			 {
					if((memcmp(ID_receive_buff, User_ID, sizeof(User_ID)) == 0))  //if相等,=0
					{
							SEGGER_RTT_printf(0,"ID_is_receive \r\n");		
							ID_is_receive = true;
							ID_is_change	= false;					 
					}
					else
					{
							ID_is_receive = true;
							ID_is_change	= true;
					}
			 }
		 }
		 else
		 {
		     ID_is_receive = true;
				 ID_is_change	= true;			 
		 }
	}
	else
	{
		 communocate_state[4] = 0x01;                //ID接收错误
		 ble_State_string_send(&m_conn,communocate_state,5);
		 nrf_delay_ms(500);
	   if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
		 {
					err_code = sd_ble_gap_disconnect(m_conn_handle,
                                           BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
		 }
	}
}

void ble_Com_Shakehands_Analysis(uint8_t * p_data, uint16_t length)
{
	if( *p_data == 0x01 && length == 5)
	{
		 SEGGER_RTT_printf(0,"0x01 \r\n");		
		 Handshake_agreement_first(p_data + 1);
		 Handshake_agreement_Second();
	}
	if( *p_data == 0x03 && length == 5)
	{
		 SEGGER_RTT_printf(0,"0x03 \r\n");		
		 Handshake_agreement_third(p_data + 1);
	}
}

void Handshake_agreement_first(uint8_t * p_data)
{
    uint8_t i;
	  for(i = 0; i < 3; i++)
	  {
		   Handshark_buff1[i] = *(p_data + i) ^ *(p_data + 3);  //提取时间
	  }
}
void Handshake_agreement_Second(void)
{
		Handshark_buff2[0] = 0x02;
  	Handshark_buff2[4] = get_random_number();
		Handshark_buff2[1] = Handshark_buff1[0] ^ Handshark_buff2[4];
		Handshark_buff2[2] = Handshark_buff1[1] ^ Handshark_buff2[4];
		Handshark_buff2[3] = Handshark_buff1[2] ^ Handshark_buff2[4];
		ble_Shakehands_string_send(&m_conn,Handshark_buff2,5);
}
void Handshake_agreement_third(uint8_t * p_data)
{
    uint8_t i;
		uint32_t err_code;
	  for(i = 0; i < 3; i++)
	  {
		   Handshark_buff3[i] = *(p_data + i) ^ *(p_data + 3);  //提取时间
	  }
		if( Handshark_buff1[0] == Handshark_buff3[0] &&
			  Handshark_buff1[1] == Handshark_buff3[1] &&
		    Handshark_buff1[2] == Handshark_buff3[2] )
		 {
			  communocate_state[4] = 0x00;                        //握手成功
			  memcpy(User_ID,ID_receive_buff, 4);
			  Is_white_adv  = true;
			  connects_timer_stop();
			  Global_connected_state = 1;
		 }
		 else
		 {
			  communocate_state[4] = 0x05;                       //时间错误
		 }
		 ble_State_string_send(&m_conn,communocate_state,5);
		 if(communocate_state[4] != 0x00)
		 {
				nrf_delay_ms(500);		 
				if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
				{
							err_code = sd_ble_gap_disconnect(m_conn_handle,
																							 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
				    	APP_ERROR_CHECK(err_code);
				}
		 }
}



#include "HC_command_analysis.h"

ble_conn_t                         m_conn;
extern uint16_t                    m_conn_handle;   /**< Handle of the current connection. */

static uint8_t ID_receive_buff[4] = {0};  //存储接收到的ID数据
static uint8_t Handshark_buff1[3];        //握手数据存储
static uint8_t Handshark_buff2[5];
static uint8_t Handshark_buff3[3];

bool ID_is_change  = false;        //接收到的ID与原先ID不同，可能需要更新绑定ID
bool ID_is_receive = false;        //ID接收到
bool Is_white_adv  = false;        //是否白名单广播
bool Is_device_bond = false;       //设备是否绑定
uint8_t communocate_state[5] = {0x04,0x00,0x00,0x00,0xFF}; //握手状态返回

extern uint8_t User_ID[4];         //存储实际用户ID，如果更改ID，在握手成功后更新该数组并将该数组写入flash  
extern bool Global_connected_state;//连接+握手成功标志

//接收ID并进行解析
void ble_Com_ID_Analysis(uint8_t * p_data, uint16_t length)
{
	uint32_t err_code;
	ID_is_receive = false;
  communocate_state[4] = 0xFF;                            //0xFF代表没有进行握手
  memset(Handshark_buff1, 0, sizeof(Handshark_buff1));
  memset(Handshark_buff2, 0, sizeof(Handshark_buff2));
  memset(Handshark_buff3, 0, sizeof(Handshark_buff3));
		
	if( *p_data == 0x00  && length == 5)       //判断ID格式是否符合
	{
     memcpy(ID_receive_buff,(p_data+1), 4);  //将接收到ID数据缓存到ID_receive_buff
 
		 if(Is_device_bond)                      //如果设备绑定过，将接收到ID进行判断       
     {	
			 if(Is_white_adv)                      //绑定且进行白名单广播，对比ID不符合则需要断开连接，慢闪
			 {
					if((memcmp(ID_receive_buff, User_ID, sizeof(User_ID)) == 0))  //if相等,=0
					{
						  if(RTT_PRINT)
							{		 		 		 
									SEGGER_RTT_printf(0,"ID_is_receive \r\n");	
							}								
							ID_is_receive = true;          //ID_is_receive置1才能接收到握手数据
							ID_is_change	= false;		     //ID_is_change置0则不会进行flash操作			 
					}
					else                               //不相等，断开连接
					{
							communocate_state[4] = 0x02;   //设备已被绑定
							err_code = ble_State_string_send(&m_conn,communocate_state,5);
						  APP_ERROR_CHECK(err_code);
						  nrf_delay_ms(500);
							if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
					    {
								   err_code = sd_ble_gap_disconnect(m_conn_handle,
         			                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
								   APP_ERROR_CHECK(err_code);
					    }
					}
			 }
			 else                                  //绑定且进行普通广播，快闪
			 {
					if((memcmp(ID_receive_buff, User_ID, sizeof(User_ID)) == 0))  //if相等,=0
					{
						  if(RTT_PRINT)
							{
									SEGGER_RTT_printf(0,"ID_is_receive \r\n");		
							}
							ID_is_receive = true;          //ID_is_receive置1才能接收到握手数据
							ID_is_change	= false;         //ID_is_change置0则不会进行flash操作		     					 
					}
					else
					{
							ID_is_receive = true;          //ID_is_receive置1才能接收到握手数据		
							ID_is_change	= true;          //ID_is_change置1则会进行flash操作		
					}
			 }
		 }
		 else                                    //如果未绑定，ID_is_receive置1才能接收到握手数据，ID_is_change用于握手成功后判断是否存储到flash
		 {
		     ID_is_receive = true;               //ID_is_receive置1才能接收到握手数据		
				 ID_is_change	= true;			           //ID_is_change置1则会进行flash操作
		 }
	}
	else                                       //接收ID数据格式错误，返回错误码并断开
	{
		 communocate_state[4] = 0x01;            //ID接收错误
		 err_code = ble_State_string_send(&m_conn,communocate_state,5);
		 APP_ERROR_CHECK(err_code);
		 nrf_delay_ms(500);
	   if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
		 {
					err_code = sd_ble_gap_disconnect(m_conn_handle,
                                           BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
		 }
	}
}

//握手数据进行解析
void ble_Com_Shakehands_Analysis(uint8_t * p_data, uint16_t length)
{
	if( *p_data == 0x01 && length == 5)  //一次握手
	{
		 Handshake_agreement_first(p_data + 1);
		 Handshake_agreement_Second();
	}
	if( *p_data == 0x03 && length == 5)  //三次握手
	{
		 Handshake_agreement_third(p_data + 1);
	}
}
//一次握手
void Handshake_agreement_first(uint8_t * p_data)
{
    uint8_t i;
	  for(i = 0; i < 3; i++)
	  {
		   Handshark_buff1[i] = *(p_data + i) ^ *(p_data + 3);  //提取时间
	  }
}
//二次握手
void Handshake_agreement_Second(void)
{
	  uint32_t err_code;
		Handshark_buff2[0] = 0x02;
  	Handshark_buff2[4] = get_random_number();
		Handshark_buff2[1] = Handshark_buff1[0] ^ Handshark_buff2[4];
		Handshark_buff2[2] = Handshark_buff1[1] ^ Handshark_buff2[4];
		Handshark_buff2[3] = Handshark_buff1[2] ^ Handshark_buff2[4];
		err_code = ble_Shakehands_string_send(&m_conn,Handshark_buff2,5);
		APP_ERROR_CHECK(err_code);
}
//三次握手
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
			  Global_connected_state = true;
			  if(RTT_PRINT)
				{
						SEGGER_RTT_printf(0,"\r Handshake Success \r\n");
				}
		 }
		 else
		 {
			  communocate_state[4] = 0x05;                       //时间错误
		 }
		 err_code = ble_State_string_send(&m_conn,communocate_state,5);
		 APP_ERROR_CHECK(err_code);
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



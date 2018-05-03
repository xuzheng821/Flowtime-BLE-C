#include "HC_data_flash.h"


static pstorage_handle_t block_flash;
static uint8_t pstorage_wait_flag = 0; //flash操作完成标志

//全局变量
uint8_t User_ID[4]={0};               //用户ID
//接收app下发需要存储的数据
uint8_t device_id_receive[16] = {0};  //接收到的device_id
uint8_t device_sn_receive[16] = {0};  //接收到的SN
uint8_t device_id_sn[32] = {0};       //从flash读取的 device_id和SN
//从flash中读取并发回app验证写入成功
uint8_t device_id_send[17] = {0};     //发送的device_id
uint8_t device_sn_send[17] = {0};     //发送的SN
uint8_t user_id_send[5] = {0};        //发送的SN

extern bool Is_white_adv;
extern bool Is_device_bond;

extern void power_manage(void);


void flash_callback(pstorage_handle_t * handle,uint8_t op_code,uint32_t result,uint8_t * p_data, uint32_t data_len)
{
	switch(op_code)
	{
		case PSTORAGE_STORE_OP_CODE:
				if (result == NRF_SUCCESS)
				{
						pstorage_wait_flag = 0;
				}
				break;
				
				case PSTORAGE_LOAD_OP_CODE:
				if (result == NRF_SUCCESS)
				{
						pstorage_wait_flag = 0;
				}
				break;

				case PSTORAGE_CLEAR_OP_CODE:
				if (result == NRF_SUCCESS)
				{
						pstorage_wait_flag = 0;
				}
				break;

				case PSTORAGE_UPDATE_OP_CODE:
				if (result == NRF_SUCCESS)
				{
						pstorage_wait_flag = 0;
				}
				break;
	}
}

void flash_init(void)
{
	 uint32_t err_code;
	
	 pstorage_module_param_t module_param;
	
	 err_code = pstorage_init();    //初始化flash
   APP_ERROR_CHECK(err_code);
	
   module_param.block_count = 3;
   module_param.block_size = 16; 
   //0---User_id
	 //1---Device_id
	 //2---SN
	 
	 module_param.cb = flash_callback;// //操作回调
	
   err_code =pstorage_register(&module_param, &block_flash);//注册申请
	 APP_ERROR_CHECK(err_code);
}

void Read_User_ID(void)
{
		 pstorage_handle_t Flash_User_ID;
     uint8_t User[4] = {0}; 
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,">>[FLASH]: Read_User_ID \r\n");
		 }

     pstorage_block_identifier_get(&block_flash, 0, &Flash_User_ID);
		  
		 pstorage_wait_flag = 1;                            //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(User, &Flash_User_ID, 4, 0);         //读取块内数据
		 while(pstorage_wait_flag) { power_manage(); }      //Sleep until load operation is finished.
		 
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"User_ID:%x",User[0]);
				SEGGER_RTT_printf(0,"%x",User[1]);
				SEGGER_RTT_printf(0,"%x",User[2]);
				SEGGER_RTT_printf(0,"%x\r\n",User[3]);
		 }			 
     memcpy(User_ID,User, 4);
		 
		 if(User_ID[0] == 0xff && User_ID[1] == 0xff && User_ID[2] == 0xff && User_ID[3] == 0xff)  //flash内无用户ID
		 {
		  	Is_device_bond = false;
			  Is_white_adv = false;
				if(RTT_PRINT)
				{
						SEGGER_RTT_printf(0,"device_Is_no_bond\r\n");
				}
		 }
	 	 else
		 {
			  Is_device_bond = true;
			  Is_white_adv = false;
				if(RTT_PRINT)
				{
						SEGGER_RTT_printf(0,"device_Is_bond\r\n");
				}
		 }
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"<<[FLASH]: Read_User_ID \r\n\n");
		 }
}

void Story_User_ID(void) 
{
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"\n>>[FLASH]: Story_User_ID \r\n");
		 }

     uint8_t ID_buff[4] = {0};
	   pstorage_handle_t Flash_User_ID;
     memcpy(ID_buff,User_ID, 4);                                //App下发的User ID存储在User_ID中

     pstorage_block_identifier_get(&block_flash, 0, &Flash_User_ID);
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_clear(&Flash_User_ID, 16);                        //清除数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until clear operation is finished.

		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
		 pstorage_update(&Flash_User_ID, ID_buff, 4, 0);            //写入数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until update operation is finished.

     memset(ID_buff, 0, sizeof(ID_buff));	 
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(ID_buff, &Flash_User_ID, 4, 0);              //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.

		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"New_User_ID:%x",ID_buff[0]);
				SEGGER_RTT_printf(0,"%x",ID_buff[1]);
				SEGGER_RTT_printf(0,"%x",ID_buff[2]);
				SEGGER_RTT_printf(0,"%x\r\n",ID_buff[3]);		
				SEGGER_RTT_printf(0,"<<[FLASH]: Story_User_ID \r\n\n");
		 }
}

void Story_Device_ID(void)
{		 
		 if(RTT_PRINT)
		 {
				SEGGER_RTT_printf(0,"\n>>[FLASH]: Story_Device_ID \r\n");
		 }

	   pstorage_handle_t device_ID;
	   uint8_t device_id[16] = {0};
     memcpy(device_id,device_id_receive, 16);

     pstorage_block_identifier_get(&block_flash, 1, &device_ID);
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_clear(&device_ID, 16);                            //清除数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until clear operation is finished.

		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
		 pstorage_update(&device_ID, device_id, 16, 0);             //更新数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until update operation is finished.
		 
     memset(device_id, 0, sizeof(device_id));	                  //device_id清零 
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(device_id, &device_ID, 16, 0);               //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.
		 
	   device_id_send[0] = Nap_App_send_deviceid;                       //上传App
	   memcpy(device_id_send+1,device_id, 16);

		 if(RTT_PRINT)
		 {		 
		     SEGGER_RTT_printf(0,"<<[FLASH]: Story_Device_ID \r\n\n");
		 }
}

void Story_SN(void)
{
		 if(RTT_PRINT)
		 {		 		 
				 SEGGER_RTT_printf(0,"\n>>[FLASH]: Story_SN \r\n");
		 }
	   pstorage_handle_t Flash_SN;
	   uint8_t SN_buff[16] = {0};
     memcpy(SN_buff,device_sn_receive, 16);

     pstorage_block_identifier_get(&block_flash, 2, &Flash_SN);
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_clear(&Flash_SN, 16);                             //清除数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until clear operation is finished.

		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
		 pstorage_update(&Flash_SN, SN_buff, 16, 0);                //更新数据块
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until update operation is finished.
		 
     memset(SN_buff, 0, sizeof(SN_buff));	                      //SN_buff清零 
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(SN_buff, &Flash_SN, 16, 0);                  //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.
		 
	   device_sn_send[0] = Nap_App_send_SN;                             //上传App
	   memcpy(device_sn_send+1,SN_buff, 16);

		 if(RTT_PRINT)
		 {		 		 		 
				SEGGER_RTT_printf(0,"<<[FLASH]: Story_SN \r\n\n");
		 }
}

void Read_device_id_sn(void)                                     //设备信息服务初始化时先读取Flash中的device_id和SN
{
		 if(RTT_PRINT)
		 {		 		 		 
					SEGGER_RTT_printf(0,">>[FLASH]: Read_device_id_sn \r\n");
		 }
		 pstorage_handle_t device_ID;
		 pstorage_handle_t SN;
     uint8_t device_id_info[16] = {0};             
     uint8_t device_sn_info[16] = {0};            

     pstorage_block_identifier_get(&block_flash, 1, &device_ID);
     pstorage_block_identifier_get(&block_flash, 2, &SN);
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(device_id_info, &device_ID, 16, 0);          //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(device_sn_info, &SN, 16, 0);                 //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.
		 
     memcpy(device_id_sn,device_id_info, 16);
		 memcpy(device_id_sn+16,device_sn_info, 16);
		 if(RTT_PRINT)
		 {		 		 		 
					SEGGER_RTT_printf(0,"<<[FLASH]: Read_device_id_sn \r\n\n");
		 }
}

void delete_User_id(void)
{
		 if(RTT_PRINT)
		 {		 		 		 
					SEGGER_RTT_printf(0,"\n>>[FLASH]: delete_User_id \r\n");
		 }
	   uint8_t userid_buff[4] = {0};
	   pstorage_handle_t Flash_User_ID;
	
     pstorage_block_identifier_get(&block_flash, 0, &Flash_User_ID);
		 
		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_clear(&Flash_User_ID, 16);
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until clear operation is finished.

		 pstorage_wait_flag = 1;                                    //Set the wait flag. Cleared in the example_cb_handler
     pstorage_load(userid_buff, &Flash_User_ID, 4, 0);          //读取块内数据	 
		 while(pstorage_wait_flag) { power_manage(); }              //Sleep until load operation is finished.
		 
	   user_id_send[0] = Nap_App_send_userid;
	   memcpy(user_id_send+1,userid_buff, 4);

		 if(RTT_PRINT)
		 {		 		 		 
					SEGGER_RTT_printf(0,"<<[FLASH]: delete_User_id \r\n\n"); 
		 }
}

#include "HC_factory_test.h"

static bool Into_System_test_mode = false;              //是否进入系统测试模式

bool APP_restart = false;                               //APP软复位标志
bool Into_factory_test_mode = false;                    //是否进入工厂测试模式
bool StoryDeviceID =false;                              //是否存储deviceID
bool StorySN =false;                                    //是否存储SN
bool deleteUserid =false;                               //是否删除UserID

extern bool ads1291_is_init;                            //ADS1291是否初始化
extern uint8_t device_id_receive[16];                   //缓存主机端发送过来的deviceID-16位
extern uint8_t device_sn_receive[16];                   //缓存主机端发送过来的SN-16位
extern bool Global_connected_state;                     //连接+握手成功标志

extern void sleep_mode_enter(void);
//启动Check，判断是否为软复位启动，是否进入工厂测试模式
void bootup_check(void)
{	  
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0," NRF_POWER->GPREGRET:%x\r\n\n",NRF_POWER->GPREGRET);
		}
		APP_restart = (NRF_POWER->GPREGRET == 0x55);    //如果软件复位，复位前会将寄存器NRF_POWER->GPREGRET设置为0x55
		if(APP_restart)                                
		{
			 NRF_POWER->GPREGRET = 0;
		}
		
		if(nrf_gpio_pin_read(FACTORY_TEST) == 0)        //判断是否进入工厂测试模式
		{
				if(RTT_PRINT)
				{
						SEGGER_RTT_printf(0,"\r Into_factory_test_mode \r\n");
				}
				Into_factory_test_mode = true;
				Uart_init();
				app_uart_put(Nap_Tool_Gotofactorytest);	    //Nap通知Tool--单板成功进入工厂测试
		}
		else
		{
			  Into_factory_test_mode = false;
		}
}
//LED测试，循环闪烁
void led_test(void)
{
	    uint32_t err_code;
	    pwm_uint();                                   //进入led_test之前，单板蓝牙连接手机，所以PWM已经初始化
	
	    nrf_gpio_cfg_output(LED_GPIO_BLUE);
      nrf_gpio_cfg_output(LED_GPIO_RED);
	    nrf_gpio_cfg_output(LED_GPIO_GREEN);
	  
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_RED;
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_GREEN;
      
	 		err_code = bsp_led_indication(BSP_INDICATE_FACTORY_LED_TEST);   //LED状态设置
      APP_ERROR_CHECK(err_code);
}
//手机端数据解析，com服务接收到的数据
void App_Nap_data_Analysis(uint8_t *pdata)
{
    uint32_t err_code;
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0,"commmand: %x \r\n",*pdata);
		}
		switch(*pdata)
		{
			 case App_Nap_SystemTest:             
				    Into_System_test_mode = true;							 
						break;
			 
			 case App_Nap_Start1291:             
				    if(ads1291_is_init == false) 
						{
								ads1291_init();
						}							 
						break;
									
			 case App_Nap_Stop1291: 
				    if(ads1291_is_init == true) 
						{
								ADS1291_disable();
						}							 
						break;	
																
			 case App_Nap_write_deviceid : 
				    if(Into_System_test_mode)
						{
								memcpy(device_id_receive,pdata+1, 16);  //去除第一个字节
								StoryDeviceID = true;                   //flash操作在main函数进入
						}
						break;
			 
			 case App_Nap_write_SN: 			 
				    if(Into_System_test_mode)
						{
								memcpy(device_sn_receive,pdata+1, 16);  //去除第一个字节
								StorySN = true;                         //flash操作在main函数进入
						}
						break;
			 
			 case App_Nap_useriddelete: 
					  deleteUserid = true;                        //flash操作在main函数进入                   
						break;
			 
			 case App_Nap_Gotoledtest: 
           if(Into_factory_test_mode)                   //进入工厂测试模式
	         {						 
							led_test(); 
					 }						 
					 break;
					 
				case App_Nap_Poweroff:                          //关机指令  
					  if(ads1291_is_init == true)
						{			
					   	 ADS1291_disable();
						}
						Global_connected_state = false;
					  err_code = bsp_led_indication(BSP_INDICATE_IDLE);
					  APP_ERROR_CHECK(err_code);
					  sleep_mode_enter();
					  break;

			 default:
						break;
		}
}


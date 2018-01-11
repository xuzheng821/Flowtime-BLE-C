#include "HC_factory_test.h"

ble_com_t                                m_com;                                      /**< Structure to identify the Nordic UART Service. */

bool APP_restart = false;
bool Into_factory_test_mode = false;
bool StoryDeviceID =false;
bool StorySN =false;
bool deleteUserid =false;

extern bool ads1291_is_init;
extern uint8_t device_id_receive[16];
extern uint8_t device_sn_receive[16];

extern void sleep_mode_enter(void);

void bootup_check(void)
{	  
	  SEGGER_RTT_printf(0," NRF_POWER->GPREGRET:%x\r\n\n",NRF_POWER->GPREGRET);
		APP_restart = (NRF_POWER->GPREGRET == 0x55);    //如果软件复位，复位前会将寄存器NRF_POWER->GPREGRET设置为0x55
		if(APP_restart)                                
		{
			 NRF_POWER->GPREGRET = 0;
		}
		
    if(nrf_gpio_pin_read(FACTORY_TEST) == 0)        //判断是否进入工厂测试模式
		{
			  SEGGER_RTT_printf(0,"\r Into_factory_test_mode \r\n");
			  Into_factory_test_mode = true;
        Uart_init();
        app_uart_put(Nap_Tool_Gotofactorytest);	    //Nap通知Tool--串口接收到单板成功进入工厂测试
		}
		else
		{
			  Into_factory_test_mode = false;
		}
}

void led_test(void)
{
	    uint32_t err_code;
	    PWM_uint();                          //进入led_test之前，单板蓝牙连接手机，所以PWM已经初始化
	
	    nrf_gpio_cfg_output(LED_GPIO_BLUE);
      nrf_gpio_cfg_output(LED_GPIO_RED);
	    nrf_gpio_cfg_output(LED_GPIO_GREEN);
	  
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_RED;
	    NRF_GPIO->OUTCLR = 1<<LED_GPIO_GREEN;
      
	 		err_code = bsp_led_indication(BSP_INDICATE_factory_led_test);   //LED状态设置
      APP_ERROR_CHECK(err_code);
}

void button_test(void)
{
//	    uint32_t err_code;

//	    uint8_t button_state_test[1] = {Nap_App_keyPress};
//	    err_code = ble_com_string_send(&m_com, button_state_test , 1);
//			APP_ERROR_CHECK(err_code);
}

void App_Nap_data_Analysis(uint8_t *pdata)
{
    uint32_t err_code;
		switch(*pdata)
		{
			 case App_Nap_Start1291: 
				    if(ads1291_is_init == 0) 
						{
								ads1291_init();
						}							 
						break;
									
			 case App_Nap_Stop1291: 
				    if(ads1291_is_init == 1) 
						{
								ADS1291_disable();
						}							 
						break;	
																
			 case App_Nap_write_deviceid : 
						memcpy(device_id_receive,pdata+1, 16);
						StoryDeviceID = true;            
						break;
			 
			 case App_Nap_write_SN: 			 
						memcpy(device_sn_receive,pdata+1, 16);
						StorySN = true;            
						break;
			 
			 case App_Nap_useriddelete: 
					  deleteUserid = true;                        
						break;
			 
			 case App_Nap_Gotoledtest:
           if(Into_factory_test_mode)        //进入工厂测试模式
	         {						 
							led_test(); 
					 }						 
					 break;
					 
				case App_Nap_Poweroff:
             err_code = bsp_led_indication(BSP_INDICATE_POWER_OFF);
             APP_ERROR_CHECK(err_code);
					   sleep_mode_enter();
					 break;

			 default:
						break;
		}
}


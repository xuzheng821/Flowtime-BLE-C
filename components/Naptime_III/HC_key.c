#include "HC_key.h"

bsp_button_event_cfg_t m_buttin_events;

static uint16_t key_tigger_num = 0;          //按键按下超时计数
static uint8_t Key_detection_interval = 50;  //定时器50ms检测一次

extern bool APP_restart;              //软复位判断
extern bool Into_factory_test_mode;   //是否进入工厂测试模式

extern void button_event_handler(button_event_t event);  //按键事件处理函数

void button_power_on(void)
{
		if(!Into_factory_test_mode && !APP_restart)          //工厂测试模式或者软复位直接开机，否则需要按键开机
	  {
	     buttons_configure_init();
			 key_tigger_num = 0;
	     button_timer_start();
       while(nrf_gpio_pin_read(BUTTON) == 0);			
       nrf_delay_ms(500);	                               //如果进入休眠，500ms内完成关机操作	
		}
}

void buttons_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	  nrf_delay_ms(20);
	  if(nrf_gpio_pin_read(BUTTON) == 0)
		{
			key_tigger_num = 0;
      button_timer_start();
		}
}

void button_gpiote_init(void)
{
	ret_code_t err_code;

	if(!nrf_drv_gpiote_is_init())
	{
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
	}

  nrf_drv_gpiote_in_config_t in_config_button = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config_button.pull = NRF_GPIO_PIN_PULLUP;
	
  err_code = nrf_drv_gpiote_in_init(BUTTON, &in_config_button, buttons_event_handler);
  APP_ERROR_CHECK(err_code);
	
  nrf_drv_gpiote_in_event_enable(BUTTON, true);	
}


void buttons_state_update(void)
{
		button_event_t button_event;
	  if(nrf_gpio_pin_read(BUTTON) == 0)     
		{
			   key_tigger_num ++;
			   if(RTT_PRINT)
				 {
						SEGGER_RTT_printf(0," %d \r\n",key_tigger_num);
				 }
         if(key_tigger_num == 2000/Key_detection_interval)
				 {
					  if(RTT_PRINT)
						{
								SEGGER_RTT_printf(0," push_event \r\n");
						}
					  button_event = m_buttin_events.push_event;
						button_event_handler(button_event);
						return;
				 }
         else if(key_tigger_num == 4000/Key_detection_interval)	
				 {
					  if(RTT_PRINT)
						{
								SEGGER_RTT_printf(0," long_push_event \r\n");
						}
	          button_timer_stop();
					  button_event = m_buttin_events.long_push_event;
						button_event_handler(button_event);
						return;
				 }					 
		}
		else  //松开 
		{
	       button_timer_stop();
			   if( key_tigger_num < 2000/Key_detection_interval )
				 {
					  if(RTT_PRINT)
						{
								SEGGER_RTT_printf(0," tigger_event \r\n");
						}
					  button_event = m_buttin_events.tigger_event;
						button_event_handler(button_event);
						return;
				 }
		}
}

void bsp_event_to_button_action_assign(bsp_button_action_t action, button_event_t event)
{
    switch (action)
    {
        case BUTTON_ACTION_TIGGER:
             m_buttin_events.tigger_event = event;
             break;
        case BUTTON_ACTION_PUSH:
             m_buttin_events.push_event = event;
             break;
        case BUTTON_ACTION_LONG_PUSH:
             m_buttin_events.long_push_event = event;
             break;
        default:
             break;
    }
}

void buttons_configure_init(void)      //按键初始化后按键功能       
{
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0,"\rbuttons_configure_init \r\n");
		}
	  bsp_event_to_button_action_assign(BUTTON_ACTION_TIGGER,
                                      BUTTON_EVENT_SLEEP);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_PUSH,
                                      BUTTON_EVENT_POWER_ON);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_LONG_PUSH,
                                      BUTTON_EVENT_IDLE);
}

void pairing_buttons_configure(void)     //快速广播下按键功能
{
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0,"\rpairing_buttons_configure \r\n");
		}
	  bsp_event_to_button_action_assign(BUTTON_ACTION_TIGGER,
                                      BUTTON_EVENT_IDLE);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_PUSH,
                                      BUTTON_EVENT_IDLE);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_LONG_PUSH,
                                      BUTTON_EVENT_POWER_OFF);
}

void advertising_buttons_configure(void)  //白名单广播下按键功能
{
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0,"\radvertising_buttons_configure \r\n");
		}
	  bsp_event_to_button_action_assign(BUTTON_ACTION_TIGGER,
                                      BUTTON_EVENT_LEDSTATE);  

	  bsp_event_to_button_action_assign(BUTTON_ACTION_PUSH,
                                      BUTTON_EVENT_WHITELIST_OFF);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_LONG_PUSH,
                                      BUTTON_EVENT_POWER_OFF);
}

void connection_buttons_configure(void)   //已连接状态按键功能
{
	  if(RTT_PRINT)
		{
				SEGGER_RTT_printf(0,"\rconnection_buttons_configure \r\n");
		}
	  bsp_event_to_button_action_assign(BUTTON_ACTION_TIGGER,
                                      BUTTON_EVENT_LEDSTATE);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_PUSH,
                                      BUTTON_EVENT_DISCONNECT);

	  bsp_event_to_button_action_assign(BUTTON_ACTION_LONG_PUSH,
                                      BUTTON_EVENT_POWER_OFF);
}

uint32_t bsp_wakeup_buttons_set(void)        //休眠唤醒按键配置
{
    uint32_t new_sense = GPIO_PIN_CNF_SENSE_Low;
	
    uint32_t new_cnf_button = NRF_GPIO->PIN_CNF[BUTTON];   
    new_cnf_button &= ~GPIO_PIN_CNF_SENSE_Msk;
    new_cnf_button |= (new_sense << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->PIN_CNF[BUTTON] = new_cnf_button;
	
	  uint32_t new_cnf_PG = NRF_GPIO->PIN_CNF[BQ_PG];
    new_cnf_PG &= ~GPIO_PIN_CNF_SENSE_Msk;
    new_cnf_PG |= (new_sense << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->PIN_CNF[BQ_PG] = new_cnf_PG;

	  uint32_t new_cnf_TEST = NRF_GPIO->PIN_CNF[FACTORY_TEST];
    new_cnf_TEST &= ~GPIO_PIN_CNF_SENSE_Msk;
    new_cnf_TEST |= (new_sense << GPIO_PIN_CNF_SENSE_Pos);
    NRF_GPIO->PIN_CNF[FACTORY_TEST] = new_cnf_TEST;

	  return NRF_SUCCESS;
}


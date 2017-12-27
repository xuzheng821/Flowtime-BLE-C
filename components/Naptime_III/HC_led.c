#include "HC_led.h"

led_indication_t m_stable_state = BSP_INDICATE_IDLE;

extern bool pwm_is_init;
extern bool Is_device_bond;  
extern bool Goto_factory_test_mode;
extern bool Is_led_timer_start;

uint8_t LED_Red_pro = 50;
uint8_t LED_Green_pro = 25;
uint8_t LED_Blue_pro = 40;

uint8_t led_blue_timerout = 0;
uint8_t led_red_timerout = 0;

uint32_t bsp_led_indication(led_indication_t indicate)
{
    uint32_t err_code = NRF_SUCCESS;
		if(pwm_is_init == false)
		{
				led_pwm_init();
		}

    switch (indicate)
    {
			case  BSP_INDICATE_IDLE:
				    if(Is_led_timer_start)
						{
							  led_timer_stop();
						}
						PWM_uint();
            m_stable_state = indicate;
            break;

      case  BSP_INDICATE_POWER_ON:        //À¶µÆÉÁË¸2´Î£¬ÆµÂÊ5HZ	
            LED_BLUE(5,LED_Blue_pro);
						nrf_delay_ms(500);
			      LED_BLUE(1,0);
            m_stable_state = indicate;
				    break;

      case  BSP_INDICATE_POWER_OFF:       //À¶µÆÉÁË¸2´Î£¬ÆµÂÊ5HZ
				    if(Is_led_timer_start)
						{
							  led_timer_stop();
						}
						LED_BLUE(1,0);
			      nrf_delay_ms(500);
            LED_BLUE(5,LED_Blue_pro);
            nrf_delay_ms(500);
			      LED_BLUE(1,0);
            m_stable_state = indicate;
				    break;

    	case  BSP_INDICATE_CONNECTED:       //À¶µÆ³£ÁÁ
      			if(Is_led_timer_start)
						{
							  led_timer_stop();
						}
      			LED_BLUE(250,LED_Blue_pro);
						if(!Goto_factory_test_mode)
						{
						    led_timer_start();
						}
			      m_stable_state = indicate;	
				    break;

	    case  BLE_INDICATE_WITH_WHITELIST:  //´ý»ú£¬À¶µÆÂýËÙÉÁË¸£¬ÆµÂÊ1HZ  2min
				    if(Is_led_timer_start)
						{
							  led_timer_stop();
						}
						led_timer_start();
			      LED_BLUE(1,LED_Blue_pro);
            m_stable_state = indicate;
 			      break;

    	case  BLE_INDICATE_WITHOUT_WHITELIST:   //À¶µÆ¿ìËÙÉÁË¸£¬ÆµÂÊ5HZ  2min     ok
				    nrf_delay_ms(100);
						if(Is_device_bond && (!Goto_factory_test_mode))
						{
						    led_timer_start();
						}
      			LED_BLUE(5,LED_Blue_pro);
            m_stable_state = indicate;			      
				    break;

	    case  BSP_INDICATE_Battery_LOW:         //ºìµÆÂýËÙÉÁË¸£¬ÆµÂÊ1HZ  2min
				    LED_BLUE(1,0);
			      if(Is_led_timer_start)
						{
							  led_timer_stop();
						}
						led_timer_start();
      			LED_RED(1,LED_Red_pro);
            m_stable_state = indicate;
				    break;

	    case  BSP_INDICATE_Battery_CHARGING:    //ºìµÆ³£ÁÁ
            LED_RED(250,LED_Red_pro);
            m_stable_state = indicate;
				    break;

    	case  BSP_INDICATE_Battery_CHARGEOVER:  //ÂÌµÆ³£ÁÁ
						LED_GREEN(LED_Green_pro);
            m_stable_state = indicate;
				    break;

    	case  BSP_INDICATE_factory_led_test:    //ÂÌµÆ³£ÁÁ
			      led_test_timer_start();	      
						NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
						NRF_GPIO->OUTSET = 1<<LED_GPIO_RED;
						nrf_delay_ms(300);
						NRF_GPIO->OUTCLR = 1<<LED_GPIO_RED;
						NRF_GPIO->OUTSET = 1<<LED_GPIO_GREEN;
						nrf_delay_ms(300);
						NRF_GPIO->OUTCLR = 1<<LED_GPIO_GREEN;
						NRF_GPIO->OUTSET = 1<<LED_GPIO_BLUE;		
            m_stable_state = indicate;
				    break;

			default:
            break;
    }
    return err_code;
}

void leds_state_update(void)
{
	  if(m_stable_state == BSP_INDICATE_CONNECTED || 
			 m_stable_state == BLE_INDICATE_WITH_WHITELIST || 
		   m_stable_state == BLE_INDICATE_WITHOUT_WHITELIST)
		{
				LED_BLUE(1,0);
				led_blue_timerout = 1;	
		}
	  if(m_stable_state == BSP_INDICATE_Battery_LOW)
		{
				LED_RED(1,0);
				led_red_timerout = 1;	
		}
		PWM_uint();
}

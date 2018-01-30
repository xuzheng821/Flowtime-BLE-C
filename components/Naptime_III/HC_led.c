#include "HC_led.h"

led_indication_t m_stable_state = BSP_INDICATE_IDLE;

extern bool Is_pwm_init;             //打开LED才进行PWM配置，降低功耗
extern bool Is_device_bond;          //设备是否绑定   
extern bool Is_led_timer_start;      //LED亮灯时间计时判断，如果亮灯中途切换状态，重新计时
extern bool Into_factory_test_mode;  //是否进入工厂测试模式
extern bool Is_red_on;
extern bool Is_green_on;
extern bool Is_blue_on;

uint8_t LED_Red_pro = 50;            //LED占空比调节
uint8_t LED_Green_pro = 25;
uint8_t LED_Blue_pro = 40;

bool led_blue_timerout = false;       //蓝灯亮灯时间超时标志
bool led_red_timerout = false;        //红灯亮灯时间超时标志

void LED_timeout_start(void)
{
	  PWM_uint();
	  if(Is_led_timer_start)
		{
			  led_timer_stop();
		}
		led_timer_start();
}

uint32_t bsp_led_indication(led_indication_t indicate)
{
		if(Is_pwm_init == false)       
		{
				led_pwm_init();
		}
		
    switch (indicate)
    {
			case  BSP_INDICATE_IDLE:                //关闭LED和超时定时器，PWM去初始化，LED口输出低电平
						led_timer_stop();
			      if(Is_pwm_init == true)       
						{
								PWM_uint();
						}
			      m_stable_state = indicate;  
            break;

      case  BSP_INDICATE_POWER_ON:            //蓝灯闪烁频率5HZ，闪烁2次后关闭
						if(Is_pwm_init == true)       
						{
								PWM_uint();
						} 
	          nrf_gpio_cfg_output(LED_GPIO_BLUE);
	          NRF_GPIO->OUTSET = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTSET = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
            m_stable_state = indicate;
				    break;

      case  BSP_INDICATE_POWER_OFF:           //蓝灯闪烁频率5HZ，闪烁2次后关闭	
//            SEGGER_RTT_printf(0," BSP_INDICATE_POWER_OFF \n");
						if(Is_pwm_init == true)       
						{
								PWM_uint();
						}  
	          nrf_gpio_cfg_output(LED_GPIO_BLUE);
	          NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(500);	  
	          NRF_GPIO->OUTSET = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTSET = 1<<LED_GPIO_BLUE;
						nrf_delay_ms(100);	  
	          NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;
            m_stable_state = BSP_INDICATE_IDLE;
				    break;

    	case  BSP_INDICATE_CONNECTED:           //连接亮灯状态，开启超时定时器
//            SEGGER_RTT_printf(0," BSP_INDICATE_CONNECTED \n");
      			LED_ON_duty(0,0,40);
			      m_stable_state = indicate;	      
				    break;

	    case  BSP_INDICATE_WITH_WHITELIST:      //待机亮灯状态，蓝灯1HZ频率闪烁，开启超时定时器
//            SEGGER_RTT_printf(0," BSP_INDICATE_WITH_WHITELIST \n");
			      if(Is_blue_on)
						{  
			          LED_ON_duty(0,0,0);
						}
						else
						{
			          LED_ON_duty(0,0,40);  
						}
						ledFlips_timer_start(500);
            m_stable_state = indicate;        //记录当前led状态，便于超时且按键按下后恢复当前状态
 			      break;

    	case  BSP_INDICATE_WITHOUT_WHITELIST:   //蓝灯快速闪烁，频率5HZ，如果绑定过则打开超时定时器
//            SEGGER_RTT_printf(0," BSP_INDICATE_WITHOUT_WHITELIST \n");
			      if(Is_blue_on)
						{
			          LED_ON_duty(0,0,0);
						}
						else
						{
			          LED_ON_duty(0,0,40);  
						}
						ledFlips_timer_start(100);
            m_stable_state = indicate;			      
				    break;

	    case  BSP_INDICATE_Battery_LOW:         //红灯慢速闪烁，频率1HZ，开启超时定时器，led切换先关闭所有灯
//            SEGGER_RTT_printf(0," BSP_INDICATE_Battery_LOW \n");
			      if(Is_red_on)
						{
			          LED_ON_duty(0,0,0);
						}
						else
						{
			          LED_ON_duty(40,0,0);  
						}
						ledFlips_timer_start(100);
            m_stable_state = indicate;        //记录当前led状态，便于超时且按键按下后恢复当前状态
				    break;

	    case  BSP_INDICATE_Battery_CHARGING:    //充电状态，红灯常亮，led切换先关闭所有灯
			      if(Is_red_on)
						{
			          LED_ON_duty(0,0,0);
						}
						else
						{
			          LED_ON_duty(60,35,0);  
						}
            m_stable_state = indicate;
				    break;

    	case  BSP_INDICATE_Battery_CHARGEOVER:  //充电完成状态，绿灯常亮，led切换先关闭所有灯
            LED_ON_duty(0,60,0);
            m_stable_state = indicate;
				    break;

    	case  BSP_INDICATE_factory_led_test:    //LED工厂测试状态，定时器1s进入一次，蓝红绿灯依次点亮
						if(Is_pwm_init == true)       
						{
								PWM_uint();
						}  
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
    return NRF_SUCCESS;
}

void leds_state_update(void)                             //LED超时定时器回调函数
{
    ledFlips_timer_stop();
	  SEGGER_RTT_printf(0," leds_state_update \n");
	  if(m_stable_state == BSP_INDICATE_CONNECTED ||       //该3种状态下，超时后标志位置1，关闭蓝灯，如果按键按下，LED恢复状态。
			 m_stable_state == BSP_INDICATE_WITH_WHITELIST || 
		   m_stable_state == BSP_INDICATE_WITHOUT_WHITELIST)
		{
				led_blue_timerout = true;	
		}
	  if(m_stable_state == BSP_INDICATE_Battery_LOW)       //低电量状态连接后，超时定时器打开，并在超时后红灯超时标志置1
		{
				led_red_timerout = true;	
		}
		PWM_uint();                                          //LED已经熄灭，将PWM去初始化
}

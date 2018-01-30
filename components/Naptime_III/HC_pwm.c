#include "HC_pwm.h"

APP_PWM_INSTANCE(PWM1,1);                   // 创建一个使用定时器1产生PWM波的实例
APP_PWM_INSTANCE(PWM2,2);                   // 创建一个使用定时器2产生PWM波的实例
APP_PWM_INSTANCE(PWM3,3);                   // 创建一个使用定时器3产生PWM波的实例

bool Is_pwm_init = false;
bool Is_red_on = false;
bool Is_green_on = false;
bool Is_blue_on = false;

void led_pwm_init(void)
{
    ret_code_t err_code;
    
    /* 3路PWM */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000L/500, LED_GPIO_RED);         //红灯PWM初始化1HZ    
    app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000L/500, LED_GPIO_GREEN);       //绿灯PWM初始化250HZ
    app_pwm_config_t pwm3_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000000L/500, LED_GPIO_BLUE);        //蓝灯PWM初始化5HZ
	
	  pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
	  pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
	  pwm3_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
	
  	/* 初始化和使能PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,NULL);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);//使能PWM1
	
    err_code = app_pwm_init(&PWM2,&pwm2_cfg,NULL);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM2);//使能PWM2

    err_code = app_pwm_init(&PWM3,&pwm3_cfg,NULL);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM3);//使能PWM3	
		
    Is_pwm_init = true;                             //PWM初始化标志
  	LED_OFF();	                                    //灯不打开
}

void LED_ON_duty(uint8_t RED_duty,uint8_t GREEN_duty,uint8_t BLUE_duty)
{
	  ret_code_t err_code;
	  err_code = app_pwm_channel_duty_set(&PWM1, 0, RED_duty);  
	  APP_ERROR_CHECK(err_code);
	  err_code = app_pwm_channel_duty_set(&PWM2, 0, GREEN_duty);
    APP_ERROR_CHECK(err_code);  
	  err_code = app_pwm_channel_duty_set(&PWM3, 0, BLUE_duty); 
	  APP_ERROR_CHECK(err_code);	 
    if(RED_duty > 0)
			 Is_red_on = true;
		else
			 Is_red_on = false;
    if(GREEN_duty > 0)
			 Is_green_on = true;
		else
			 Is_green_on = false;
    if(BLUE_duty > 0)
			 Is_blue_on = true;
		else
			 Is_blue_on = false;		
}

void LED_OFF(void)
{
	  ret_code_t err_code;
	  err_code = app_pwm_channel_duty_set(&PWM1, 0, 0);  
	  APP_ERROR_CHECK(err_code);
	  err_code = app_pwm_channel_duty_set(&PWM2, 0, 0);
    APP_ERROR_CHECK(err_code);  
	  err_code = app_pwm_channel_duty_set(&PWM3, 0, 0); 
	  APP_ERROR_CHECK(err_code);
    Is_red_on = false;
    Is_green_on = false;
    Is_blue_on = false; 
}

void PWM_uint(void)
{
	  app_pwm_uninit(&PWM1);                             //PWM去初始化
	  app_pwm_uninit(&PWM2);
	  app_pwm_uninit(&PWM3);
	
	  Is_pwm_init = false;                               //PWM初始化标志位置0
	  nrf_gpio_cfg_output(LED_GPIO_BLUE);                //设置LED的IO口为普通输出模式
    nrf_gpio_cfg_output(LED_GPIO_RED);
	  nrf_gpio_cfg_output(LED_GPIO_GREEN);

	  NRF_GPIO->OUTCLR = 1<<LED_GPIO_BLUE;               //IO口输出低电平
	  NRF_GPIO->OUTCLR = 1<<LED_GPIO_RED;
	  NRF_GPIO->OUTCLR = 1<<LED_GPIO_GREEN;
	
	  Is_red_on = false;
    Is_green_on = false;
    Is_blue_on = false; 
}



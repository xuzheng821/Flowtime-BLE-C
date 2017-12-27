#include "HC_battery.h"
#include "HC_timer.h"

ble_bas_t                    m_bas;                     /**< Structure used to identify the battery service. */

//全局变量
double bat_vol;                         //实测电量
double saft_vol = 3.1;                  //安全电压
double min_work_vol = 3.2;              //低电量

bool saadc_is_init = false;
extern bool Goto_factory_test_mode;

extern void sleep_mode_enter(void);

double min(double a, double b)
{
    return a<b?a:b;
}

/*******************************************************************************
* Battery Service init
********************************************************************************/
void ble_battory_serv_init(void)
{
	  uint32_t        err_code;
	  ble_bas_init_t  bas_init;

    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = false;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

}																	 

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);   //30
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
	  saadc_is_init = true;
	
	  nrf_gpio_cfg_output(TPS_CTRL);
	  NRF_GPIO->OUTSET = 1<<TPS_CTRL;	
	  battery_timer_start();
}

void saadc_unit(void)
{
    nrf_drv_saadc_uninit();
	  saadc_is_init = false;
	  NRF_GPIO->OUTCLR = 1<<TPS_CTRL;	
}


void battery_level_update(void)
{
    uint32_t err_code;
	  double Electricity_percentage;                                      //电量百分比
	  static nrf_saadc_value_t  ADC_value = 0;	                          //ADC读取数据
	  static double  bat_vol_old = 4.10;	                                
	
  	nrf_drv_saadc_sample_convert(0,&ADC_value);
	  bat_vol = ADC_value * 3.6 / 1024.0 * 6;                             //电池电压实际电压

  	bat_vol_old = min(bat_vol_old,bat_vol);
	  Electricity_percentage = ( min(bat_vol_old,bat_vol) - 3.10 ) * 100; //电量百分比
 	  err_code = ble_bas_battery_level_update(&m_bas, Electricity_percentage);
    APP_ERROR_CHECK(err_code);
}

void Power_Check(void)
{
	  static nrf_saadc_value_t  ADC_value = 0;	                          //ADC读取数据
	
  	nrf_drv_saadc_sample_convert(0,&ADC_value);
	  bat_vol = ADC_value * 3.6 / 1024.0 * 6;                             //电池电压实际电压
	
		if(bat_vol < saft_vol)                          //低于安全电压,关机
		{
			  SEGGER_RTT_printf(0,"\r Voltage is lower than 3.1V \r\n");
			  sleep_mode_enter();
		}
		else
			  SEGGER_RTT_printf(0,"\r Voltage is higher than 3.1V \r\n");
}

uint8_t connected_power_check(void)
{
		float bat_vol;
		nrf_saadc_value_t  ADC_value = 0;	         //电量检测,先初始化ADC

  	nrf_drv_saadc_sample_convert(0,&ADC_value);
	  bat_vol = ADC_value * 3.6 / 1024 * 6;           //电池电压
	
	  if(bat_vol < min_work_vol)            //低于使用电压
	  {
			 SEGGER_RTT_printf(0,"\r Voltage is lower than 3.2V \r\n");
			 return true;
	  }
		else
		{
			 SEGGER_RTT_printf(0,"\r Voltage is higher than 3.2V \r\n");
			 return false;
		}
}

void charging_check(void)
{
		uint32_t err_code;	   
		while(nrf_gpio_pin_read(BQ_PG) == 0 && !Goto_factory_test_mode)     //input vol is above battery vol
		{
			 nrf_delay_ms(500);
			 if(nrf_gpio_pin_read(BQ_CHG) == 0)   //charging
			 {
				  err_code = bsp_led_indication(BSP_INDICATE_Battery_CHARGING);
           APP_ERROR_CHECK(err_code);
			 }
			 else                                 //charging_over
			 {
				  err_code = bsp_led_indication(BSP_INDICATE_Battery_CHARGEOVER);
           APP_ERROR_CHECK(err_code);
			 }
		}
		err_code = bsp_led_indication(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}

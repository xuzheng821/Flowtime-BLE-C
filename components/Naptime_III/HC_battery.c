#include "HC_battery.h"

ble_bas_t                    m_bas;     /**< Structure used to identify the battery service. */

#define VOLTAGE_AVG_NUM  3              //电池电压滤波数组大小

//全局变量
double bat_vol;                         //实测电量
uint8_t bat_vol_pre;                    //电量百分比
uint8_t bat_vol_pre_work = 20;          //电量百分比

extern bool Into_factory_test_mode;     //是否进入工厂测试模式
extern bool Global_connected_state;     //连接+握手成功标志

extern void sleep_mode_enter(void);
//取两个数中较小的数
uint8_t min(uint8_t a, uint8_t b)
{
	return a > b ? b : a;
}
//电池服务初始化
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
    bas_init.initial_batt_level   = bat_vol_pre;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

//SAADC回调函数
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

}																	 

//SAADC初始化-PIN30-ADC第六通道-采集电池电压
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);   //PIN30，adc第六通道，
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
	
	  nrf_gpio_cfg_output(TPS_CTRL);                 //打开TPS62746内部开关才能采集到电池电压
	  NRF_GPIO->OUTSET = 1<<TPS_CTRL;	 
}


//电池电量更新到bat_vol，每个1s更新一次，然后取10次的数据的平均值发送
void battery_level_update(void)                   
{
    uint32_t err_code;
	  static uint8_t bat_vol_pre_old = 100;                //上一次电量百分比
    static uint8_t bat_vol_arrary_index = 0;
    static double bat_vol_arrary[VOLTAGE_AVG_NUM] = {0};
    
	  nrf_saadc_value_t  ADC_value = 0;	                   //ADC读取数据
  	nrf_drv_saadc_sample_convert(0,&ADC_value);
  	bat_vol = ADC_value * 3.60 / 1024.0 * 2;             //电池测量电压
		
		if( bat_vol_arrary[0] == 0 )                         //第一次进入，数组为空             
		{
				for( int i = 0; i < VOLTAGE_AVG_NUM; i++ )
			  {
						bat_vol_arrary[i] = bat_vol;
				}
		}
		else
		{
				bat_vol_arrary[bat_vol_arrary_index] = bat_vol;
				bat_vol_arrary_index = ( bat_vol_arrary_index + 1 )%VOLTAGE_AVG_NUM;
				if(bat_vol_arrary_index == 0)                    //10个数据后计算一次平均数
				{
						bat_vol = 0;
						for( int i = 0; i < VOLTAGE_AVG_NUM; i++ )
						{
								bat_vol += bat_vol_arrary[i];
						}
						bat_vol = bat_vol / VOLTAGE_AVG_NUM;
						bat_vol_pre = (uint8_t)((bat_vol - 3.10 ) * 100);     //电量百分比
						
						bat_vol_pre = min(bat_vol_pre,bat_vol_pre_old);       //取与上次计算数据两者中较小的数，避免手机端数据会增大
						bat_vol_pre_old = bat_vol_pre;
						
						if(bat_vol_pre > 100)                                 //最大显示电量100%
							 bat_vol_pre = 100;
	          SEGGER_RTT_printf(0,"\r Voltage %d \r\n",bat_vol_pre);
						
						do{
							 err_code = ble_bas_battery_level_update(&m_bas, bat_vol_pre);
		          }while(err_code == BLE_ERROR_NO_TX_PACKETS && Global_connected_state);
						
						if(bat_vol_pre < 5)                                //低于3.15V,关机
						{
//								SEGGER_RTT_printf(0,"\r Voltage is lower than 3.1V \r\n");
							  Global_connected_state = false;
								sleep_mode_enter();
						}
			 }
		}
}

//开机电池电压Check，低电压不能开机
void Power_Check(void)
{
	  nrf_saadc_value_t  ADC_value = 0;	              //ADC读取数据
  	nrf_drv_saadc_sample_convert(0,&ADC_value);
	  bat_vol = ADC_value * 3.6 / 1024.0 * 2;         //电池电压实际电压
	
		if(bat_vol < 3.1)                          //低于3.1V无法开机
		{
//			  SEGGER_RTT_printf(0,"\r Voltage is lower than 3.1V \r\n");
			  Global_connected_state = false;
			  sleep_mode_enter();
		}
}

//主机发起连接后进行电量Check，电量低（小于3.3V）则提示低电量
uint8_t connected_power_check(void)
{
//	  return bat_vol < min_work_vol ? true : false;   //电量过低返回true
}
//USB插入且为非工厂测试模式时进入，充电不执行其他操作
void charging_check(void)
{
		uint32_t err_code;	   
		while(nrf_gpio_pin_read(BQ_PG) == 0 && !Into_factory_test_mode)    
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
		
		battery_timer_start();   //开启定时采集电压
}

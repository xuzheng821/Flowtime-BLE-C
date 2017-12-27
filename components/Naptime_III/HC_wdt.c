#include "HC_wdt.h"
#include "HC_timer.h"

nrf_drv_wdt_channel_id m_channel_id;
extern void sleep_mode_enter(void);

/**
 * @brief WDT events handler.
 */
void wdt_events_handler(void)
{
		uint32_t err_code;
    SEGGER_RTT_printf(0," wdt_events_handler \n");
		err_code = sd_power_gpregret_set(0x55);
		APP_ERROR_CHECK(err_code);
	  //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void wdt_Init(void)
{
		uint32_t err_code;
	  nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_events_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
	  wdt_timer_start();
}





/*
 * pps960.c
 *
 *  Created on: 2016年9月8日
 *      Author: cole
 */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h" 
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "afe4404_hw.h"
#include "agc_V3_1_19.h"
#include "hqerror.h"
#include "pps960.h"
#include "Naptime_III.h"
#include "ble_hrs.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"
#include "HC_uart.h"
#include "HC_timer.h"

extern ble_hrs_t                         m_hrs;                                      /**< Structure used to identify the heart rate service. */

bool pps964_is_init = false;   //1291是否初始化完成标志位
uint8_t PPS960_readReg_faile = 0;   

static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);//twi0

uint16_t lifeQhrm = 0;
int8_t snrValue = 0,skin = 0,sample = 0;
 
uint8_t Hrs_data_is_ok = 0;

int8_t HR_HRV_enable=0;//0=>HR;1=>HRV;2=>HR+HRV;

void PPS_DELAY_MS(uint32_t ms)
{
		nrf_delay_ms(ms);
}

void pps960_Rest_SW(void)
{
    PPS960_writeReg(0,0x8);
    PPS_DELAY_MS(50);
}

void pps960_disable(void)
{
	pps964_is_init = false;
	pps960_rd_raw_timer_stop();
	pps960_alg_timer_stop();
	nrf_gpio_cfg_output(PPS_EN_PIN);
	nrf_gpio_pin_write(PPS_EN_PIN, 0);
	PPS_DELAY_MS(200);
//	SEGGER_RTT_printf(0," pps964_is_init:%d \n",__FPU_USED);
	#if (__FPU_USED == 1)
  __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
 (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
  #endif
}

void pps960_init(void)
{
	  nrf_gpio_cfg_output(PPS_EN_PIN);
	  nrf_gpio_pin_write(PPS_EN_PIN, 1);
	  PPS_DELAY_MS(200);
	  nrf_gpio_pin_write(PPS_REST_PIN, 0);//low
    PPS_DELAY_MS(30);
		nrf_gpio_pin_write(PPS_REST_PIN, 1);//high
    PPS_DELAY_MS(30);
	
    init_PPS960_register();
    PPS960_init();
	
	  pps964_is_init = true;
		pps960_rd_raw_timer_start();
		pps960_alg_timer_start();
	  SEGGER_RTT_printf(0," pps964_is_init:%d \n",pps964_is_init);
}

void pps960_sensor_task(void *params)
{
		if(pps964_is_init){
			ALGSH_retrieveSamplesAndPushToQueue();//read pps raw data
			//move ALGSH_dataToAlg(); to message queue loop. and then send message at here.
			ALGSH_dataToAlg();
		}
}

void pps960_sensor_task2(void *params)
{
	  uint32_t err_code;
    static uint8_t cnt=0;
	 
		if(pps964_is_init) {
						sample=GetHRSampleCount();
						ClrHRSampleCount();
						cnt++;if(cnt>255)cnt=0;
						lifeQhrm = pps_getHR();
						snrValue=PP_GetHRConfidence();//for snr check
						skin = PPS_get_skin_detect();
			
						if(skin == 0  && PPS960_readReg_faile == 0)
						{
							lifeQhrm = 0;
						}
						Hrs_data_is_ok = 1;
						SEGGER_RTT_printf(0,"%d HR=%d snr=%d spl=%d skin=%d\r\n",cnt,lifeQhrm,snrValue,sample,skin);

						err_code = ble_HRS_DATA_send(&m_hrs, lifeQhrm , 1);
						if (err_code == BLE_ERROR_NO_TX_PACKETS ||
							err_code == NRF_ERROR_INVALID_STATE || 
							err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
							{
								 return;
							}
						else if (err_code == NRF_SUCCESS) 
						{
							Hrs_data_is_ok = 0;
						}
						else 
						{
							APP_ERROR_CHECK(err_code);
						}
		}
}

//=============================================================================

#define PPS960_ADDR (0x58U)

/**
 * @brief TWI master instance
 *
 * eeprom memory.
 */
void PPS960_writeReg(uint8_t regaddr, uint32_t wdata)
{
	  uint8_t temp[4];
	
    uint32_t wd=wdata;
    temp[0]=regaddr;
    temp[1]=(wd>>16) & 0xff;
    temp[2]=(wd>>8) & 0xff;
    temp[3]=wd & 0xff;
    ret_code_t ret;
    do
    {
        ret = nrf_drv_twi_tx(&m_twi_master, PPS960_ADDR, temp, 4, false);
    }while (0);

		if(NRF_SUCCESS != ret)
		{
			if(0x8201==ret)
				{SEGGER_RTT_printf(0,"i2c Ack error!");}
			else if(0x8202==ret)
				{SEGGER_RTT_printf(0,"i2c Nack error!");}
				
//			SEGGER_RTT_printf(0,"PPS960_writeReg faile!!! %x\r\n",ret);
			PPS960_readReg_faile = 1;
		}
}



uint32_t PPS960_readReg(uint8_t regaddr)
{
    ret_code_t ret;
		uint8_t temp[4];
    uint32_t rdtemp;
	
    do
    {
       uint8_t addr8 = (uint8_t)regaddr;
       ret = nrf_drv_twi_tx(&m_twi_master, PPS960_ADDR, &addr8, 1, true);
       if (NRF_SUCCESS != ret)
       {
				 	if(0x8201==ret)
						{SEGGER_RTT_printf(0,"i2c Ack error!");}
					else if(0x8202==ret)
						{SEGGER_RTT_printf(0,"i2c Nack error!");}
						
//					SEGGER_RTT_printf(0,"PPS960_readReg faile!!! %x\r\n",ret);
					PPS960_readReg_faile = 1;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, PPS960_ADDR, temp, 3);
    }while (0);
		if(NRF_SUCCESS != ret)
		{
			if(0x8201==ret)
				{SEGGER_RTT_printf(0,"i2c Ack error!");}
			else if(0x8202==ret)
				{SEGGER_RTT_printf(0,"i2c Nack error!");}
				
//			SEGGER_RTT_printf(0,"PPS960_readReg faile!!! %x\r\n",ret);
			PPS960_readReg_faile = 1;
		}

    rdtemp = temp[0]<<16 | temp[1]<<8 | temp[2];
    //printf("rdtemp : %d\r\n ",rdtemp);

    return rdtemp;
}

/**
 * @brief Initialize the master TWI
 * @return NRF_SUCCESS or the reason of failure
 */

ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}
/** @} */ /* End of group twi_master_with_twis_slave_example */

//#endif



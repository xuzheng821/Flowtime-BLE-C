/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef NAPTIME_III_H
#define NAPTIME_III_H

#define AEF_RDRDY           2
#define SPI_CS_PIN          6 /**< SPI CS Pin.*/
#define AEF_MAIN_CLK        7
#define AEF_START           8
#define AEF_RESET           9
#define AEF_MAIN_CLKSEL     10

#define BQ_CHG              15
#define BQ_PG               16
#define AEF_PM_EN           27
#define BUTTON              18
#define RX_PIN_NUMBER       20
#define TX_PIN_NUMBER       19

#define TPS_CTRL            25
#define FACTORY_TEST        26 
#define LED_GPIO_RED        17 
#define LED_GPIO_GREEN      18
#define LED_GPIO_BLUE       19 
#define ADC_VBAT            30

#define CTS_PIN_NUMBER      7  //11
#define RTS_PIN_NUMBER      8  //12
#define HWFC                true

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif // PCA10040_H

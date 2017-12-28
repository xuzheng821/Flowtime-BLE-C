#ifndef HC_RANDOM_NUMBER_H__
#define HC_RANDOM_NUMBER_H__

#include <stdio.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_drv_rng.h"
#include "nrf_assert.h"
#include "softdevice_handler.h"

uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size);
uint8_t get_random_number(void);

#endif

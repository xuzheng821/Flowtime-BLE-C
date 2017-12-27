#include "HC_random_number.h"

#define RANDOM_BUFF_SIZE 1                     /**< Random numbers buffer size. */

uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint8_t available;
    uint32_t err_code;
    err_code = nrf_drv_rng_bytes_available(&available);
    APP_ERROR_CHECK(err_code);
    uint8_t length = (size<available) ? size : available;
    err_code = nrf_drv_rng_rand(p_buff,length);
    APP_ERROR_CHECK(err_code);
    return length;
}

uint8_t get_random_number(void)
{
	  uint8_t p_buff[1] = {0};
    uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
	  return p_buff[0];
}

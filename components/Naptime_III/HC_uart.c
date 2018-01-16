#include "HC_uart.h"

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//工装发过来的串口数据解析，主要进行充电过程的测试
void uart_data_Analysis(uint8_t *pdata)
{
	 switch (*pdata)
	 {
		 case Tool_Nap_charging:                       //Tool通知Nap--充电中
	   {
			 if(nrf_gpio_pin_read(BQ_PG) == 0 && nrf_gpio_pin_read(BQ_CHG) == 0)     //充电中
			 {
					 app_uart_put(Nap_Tool_chargingnormal);  //状态正确
			 }
			 else
			 { 
					 app_uart_put(Nap_Tool_chargingerror);   //状态错误
			 }
	   }
		 break;
		 
		 case Tool_Nap_chargeover:                     //Tool通知Nap--充电完成状态
	   {
			 if(nrf_gpio_pin_read(BQ_PG) == 0 && nrf_gpio_pin_read(BQ_CHG) == 1)     //充电完成
			 {
					 app_uart_put(Nap_Tool_chargeovernormal);  //状态正确
			 }
			 else
			 {
					 app_uart_put(Nap_Tool_chargeovererror);   //状态错误
			 }
	   }
		 break;
	
		 default:
          break;
   }
}

//串口服务函数
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t uart_data[1];
	
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
             app_uart_get(&uart_data[0]);
             uart_data_Analysis(uart_data);
             break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

//串口初始化函数-19-20-9600
void Uart_init(void)
{
    uint32_t  err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,          
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud9600
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

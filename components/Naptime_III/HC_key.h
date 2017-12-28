#ifndef HC_KEY_H_
#define HC_KEY_H_

#include "app_error.h"
#include "Naptime_III.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "ble.h"
#include "HC_timer.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

#define BUTTON_ACTION_TIGGER     1
#define BUTTON_ACTION_PUSH       2      
#define BUTTON_ACTION_LONG_PUSH  3
  
typedef uint8_t      bsp_button_action_t;                        /**< The different actions that can be performed on a button. */

typedef enum
{
	  BUTTON_EVENT_IDLE,
	  BUTTON_EVENT_POWER_ON,         //开机状态
	  BUTTON_EVENT_POWER_OFF,        //关机状态  
    BUTTON_EVENT_LEDSTATE,         //LED状态             
    BUTTON_EVENT_SLEEP,            //睡眠状态  
    BUTTON_EVENT_WHITELIST_OFF,    //配对状态 
	  BUTTON_EVENT_POWER_OFF_LED,
	  BUTTON_EVENT_FACTORY_TEST
} button_event_t;

typedef struct
{
    button_event_t tigger_event;      /**< The event to fire on regular button press. */
    button_event_t push_event;        /**< The event to fire on long button press. */
    button_event_t long_push_event;   /**< The event to fire on button release. */
} bsp_button_event_cfg_t;

#define init_buttons           11
#define pairing_buttons        12
#define advertising_buttons    13
#define connection_buttons     14
#define factory_buttons        15

void button_power_on(void);
void button_gpiote_init(void);
void buttons_state_update(void);
void bsp_event_to_button_action_assign(bsp_button_action_t action, button_event_t event);

void buttons_configure_init(void);          //按键初始化后按键功能     
void advertising_buttons_configure(void);   //白名单广播下按键功能
void pairing_buttons_configure(void);       //快速广播下按键功能
void connection_buttons_configure(void);    //已连接状态按键功能
void factory_buttons_configure(void);       //工厂测试模式按键功能    

uint32_t bsp_wakeup_buttons_set(void);      //休眠唤醒按键配置

#endif

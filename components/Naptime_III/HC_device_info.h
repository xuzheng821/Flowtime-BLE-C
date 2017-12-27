#ifndef HC_DEVICE_INFO_H_
#define HC_DEVICE_INFO_H_

// Device Information

#define DEVICE_NAME       "Naptime123"       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Huiche Inc"        /**< Manufacturer. Will be passed to Device Information Service. */
#define FW_REV_STR        "0.2.0"
#define HW_REV_STR        "1.0.0"             /**< Hardware Revision String. */

void ble_devinfo_serv_init(void);
void mac_get(void);

#endif

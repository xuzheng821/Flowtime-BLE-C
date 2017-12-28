#include "HC_device_info.h"

ble_gap_addr_t address;

extern uint8_t device_id_sn[32];

/*******************************************************************************
* Device information init
********************************************************************************/
void ble_devinfo_serv_init(void)
{
    uint32_t        err_code;
    ble_dis_init_t  dis_init;

	  mac_get();
	
    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);  //设备名称
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FW_REV_STR);                //软件版本
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,HW_REV_STR);                 //硬件版本
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,(char *)address.addr);    //MAC地址

	  Read_device_id_sn();
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str,(char *)device_id_sn);
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

void mac_get(void)
{
	  uint32_t err_code = sd_ble_gap_address_get(&address);
	  APP_ERROR_CHECK(err_code);
}







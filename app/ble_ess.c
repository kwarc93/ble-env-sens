/*
 * ble_ess.c
 *
 *  Created on: 14 lut 2022
 *      Author: kwarc
 */

#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_ess.h"

#include <string.h>

#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"


uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    VERIFY_PARAM_NOT_NULL(p_ess);
    VERIFY_PARAM_NOT_NULL(p_ess_init);

    uint32_t              err_code;
//    ble_hts_meas_t        initial_htm;
    ble_uuid_t            ble_uuid;
//    ble_add_char_params_t add_char_params;

    // Initialize service structure
//    p_ess->evt_handler   = p_ess_init->evt_handler;
//    p_ess->p_gatt_queue  = p_ess_init->p_gatt_queue;
//    p_ess->error_handler = p_ess_init->error_handler;
    p_ess->conn_handle   = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ess->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return err_code;
}

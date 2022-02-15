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

#define OPCODE_LENGTH 1                                                           /**< Length of opcode inside ES Measurement packet. */
#define HANDLE_LENGTH 2                                                           /**< Length of handle inside ES Measurement packet. */
#define MAX_ESS_LEN   (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted ES Measurement. */

/**
 * @brief Function for handling the Connect event.
 *
 * @param[in]   p_hts       ESS structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    p_ess->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**
 * @brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hts       ESS structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**
 * @brief Function for handling write events to the ESS all characteristic.
 *
 * @param[in]   p_hts         ESS structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_cccd_write(ble_ess_t * p_ess, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_ess->evt_handler != NULL)
        {
            ble_ess_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_DISABLED;
            }

            p_ess->evt_handler(p_ess, &evt);
        }
    }
}


/**
 * @brief Function for handling the Write event.
 *
 * @param[in]   p_hts       ESS structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ess->temperature_handles.cccd_handle ||
        p_evt_write->handle == p_ess->humidity_handles.cccd_handle ||
        p_evt_write->handle == p_ess->pressure_handles.cccd_handle)
    {
        on_cccd_write(p_ess, p_evt_write);
    }
}

uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    VERIFY_PARAM_NOT_NULL(p_ess);
    VERIFY_PARAM_NOT_NULL(p_ess_init);

    uint32_t                err_code;
    ble_uuid_t              ble_uuid;
    ble_add_char_params_t   add_char_params;
    ble_add_descr_params_t  add_descr_params;
    uint8_t                 init_value_encoded[MAX_ESS_LEN];
    ble_ess_meas_t          initial_meas = {0};

    // Initialize service structure
    p_ess->conn_handle   = BLE_CONN_HANDLE_INVALID;
    p_ess->evt_handler   = p_ess_init->evt_handler;
//    p_ess->p_gatt_queue  = p_ess_init->p_gatt_queue;
//    p_ess->error_handler = p_ess_init->error_handler;
    p_ess->feature       = p_ess_init->feature;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ess->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_ess->feature & BLE_ESS_FEATURE_TEMPERATURE_BIT)
    {
        // Add temperature measurement characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));

        add_char_params.uuid                = 0x2A6E;
        add_char_params.init_len            = uint16_encode(initial_meas.temperature, init_value_encoded);
        add_char_params.max_len             = sizeof(initial_meas.temperature);
        add_char_params.p_init_value        = init_value_encoded;
        add_char_params.char_props.read     = 1;
        add_char_params.char_props.notify   = 1;
        add_char_params.read_access         = SEC_OPEN;
        add_char_params.cccd_write_access   = SEC_OPEN;

        err_code = characteristic_add(p_ess->service_handle, &add_char_params, &p_ess->temperature_handles);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // TODO: Add temperature measurement descriptor
        memset(&add_descr_params, 0, sizeof(add_descr_params));

//        add_descr_params.uuid = 0x290C;
//        add_descr_params.read_access = SEC_OPEN;
//
//        err_code = descriptor_add(p_ess->temperature_handles.value_handle, &add_descr_params, &p_ess->temperature_handles.sccd_handle);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
    }

    if (p_ess->feature & BLE_ESS_FEATURE_HUMIDITY_BIT)
    {
        // Add humidity measurement characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));

        add_char_params.uuid                = 0x2A6F;
        add_char_params.init_len            = uint16_encode(initial_meas.humidity, init_value_encoded);
        add_char_params.max_len             = sizeof(initial_meas.humidity);
        add_char_params.p_init_value        = init_value_encoded;
        add_char_params.char_props.read     = 1;
        add_char_params.char_props.notify   = 1;
        add_char_params.read_access         = SEC_OPEN;
        add_char_params.cccd_write_access   = SEC_OPEN;

        err_code = characteristic_add(p_ess->service_handle, &add_char_params, &p_ess->humidity_handles);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    if (p_ess->feature & BLE_ESS_FEATURE_PRESSURE_BIT)
    {
        // Add pressure measurement characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));

        add_char_params.uuid                = 0x2A6D;
        add_char_params.init_len            = uint32_encode(initial_meas.pressure, init_value_encoded);
        add_char_params.max_len             = sizeof(initial_meas.pressure);
        add_char_params.p_init_value        = init_value_encoded;
        add_char_params.char_props.read     = 1;
        add_char_params.char_props.notify   = 1;
        add_char_params.read_access         = SEC_OPEN;
        add_char_params.cccd_write_access   = SEC_OPEN;

        err_code = characteristic_add(p_ess->service_handle, &add_char_params, &p_ess->pressure_handles);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return err_code;
}

void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_ess_t * p_ess = (ble_ess_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ess, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ess, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ess, p_ble_evt);
            break;

//        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
//            on_read_rsp(p_ess, p_ble_evt);
//            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_ess_measurement_send(ble_ess_t * p_ess, ble_ess_meas_t * p_measurement)
{
    if (p_ess == NULL || p_measurement == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    // Send value if connected and notifying
    if (p_ess->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_meas[MAX_ESS_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        if ((p_ess->feature & BLE_ESS_FEATURE_TEMPERATURE_BIT) && p_measurement->is_temperature_data_present)
        {
            // Send temperature measurement characteristic
            len     = uint16_encode(p_measurement->temperature, encoded_meas);
            hvx_len = len;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_ess->temperature_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvx_len;
            hvx_params.p_data = encoded_meas;

            err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
            if ((err_code == NRF_SUCCESS) && (hvx_len != len))
            {
                err_code = NRF_ERROR_DATA_SIZE;
                return err_code;
            }
        }

        if ((p_ess->feature & BLE_ESS_FEATURE_HUMIDITY_BIT) && p_measurement->is_humidity_data_present)
        {
            // Send humidity measurement characteristic
            len     = uint16_encode(p_measurement->humidity, encoded_meas);
            hvx_len = len;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_ess->humidity_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvx_len;
            hvx_params.p_data = encoded_meas;

            err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
            if ((err_code == NRF_SUCCESS) && (hvx_len != len))
            {
                err_code = NRF_ERROR_DATA_SIZE;
                return err_code;
            }
        }

        if ((p_ess->feature & BLE_ESS_FEATURE_PRESSURE_BIT) && p_measurement->is_pressure_data_present)
        {
            // Send pressure measurement characteristic
            len     = uint32_encode(p_measurement->pressure, encoded_meas);
            hvx_len = len;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_ess->pressure_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvx_len;
            hvx_params.p_data = encoded_meas;

            err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
            if ((err_code == NRF_SUCCESS) && (hvx_len != len))
            {
                err_code = NRF_ERROR_DATA_SIZE;
                return err_code;
            }
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_ess_measurement_update(ble_ess_t * p_ess, ble_ess_meas_t * p_measurement)
{
    if (p_ess == NULL || p_measurement == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;

    uint8_t                encoded_meas[MAX_ESS_LEN];
    ble_gatts_value_t      gatts_value;

    if ((p_ess->feature & BLE_ESS_FEATURE_TEMPERATURE_BIT) && p_measurement->is_temperature_data_present)
    {
        // Update temperature measurement characteristic

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = uint16_encode(p_measurement->temperature, encoded_meas);
        gatts_value.offset  = 0;
        gatts_value.p_value = encoded_meas;

        err_code = sd_ble_gatts_value_set(p_ess->conn_handle,
                                          p_ess->temperature_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    if ((p_ess->feature & BLE_ESS_FEATURE_HUMIDITY_BIT) && p_measurement->is_humidity_data_present)
    {
        // Update humidity measurement characteristic

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = uint16_encode(p_measurement->humidity, encoded_meas);
        gatts_value.offset  = 0;
        gatts_value.p_value = encoded_meas;

        err_code = sd_ble_gatts_value_set(p_ess->conn_handle,
                                          p_ess->humidity_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    if ((p_ess->feature & BLE_ESS_FEATURE_PRESSURE_BIT) && p_measurement->is_pressure_data_present)
    {
        // Update pressure measurement characteristic

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = uint32_encode(p_measurement->pressure, encoded_meas);
        gatts_value.offset  = 0;
        gatts_value.p_value = encoded_meas;

        err_code = sd_ble_gatts_value_set(p_ess->conn_handle,
                                          p_ess->pressure_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return err_code;
}

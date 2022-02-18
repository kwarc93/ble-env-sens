/*
 * ble_ess.h
 *
 *  Created on: 14 lut 2022
 *      Author: kwarc
 */

/* @brief   Environmental Sensing Service (ESS) module. */

#ifndef APP_BLE_ESS_H_
#define APP_BLE_ESS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/**
 * @brief   Macro for defining a ble_ess instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ESS_DEF(_name) \
static ble_ess_t _name;     \
NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                     2,                         \
                     ble_ess_on_ble_evt, &_name)

#define BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE  0x181A

/**
 * @brief BLE_ESS_FEATURES ESS feature bits
 */
#define BLE_ESS_FEATURE_TEMPERATURE_BIT          (0x01 << 0)     /**< Temperature Data Supported bit. */
#define BLE_ESS_FEATURE_HUMIDITY_BIT             (0x01 << 1)     /**< Humidity Data Supported bit. */
#define BLE_ESS_FEATURE_PRESSURE_BIT             (0x01 << 2)     /**< Pressure Data Supported bit. */


/** @brief ESS event type. */
typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED,                       /**< ESS value notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED                       /**< ESS value notification disabled event. */
} ble_ess_evt_type_t;

/** @brief ESS event. */
typedef struct
{
    ble_ess_evt_type_t evt_type;                            /**< Type of event. */
} ble_ess_evt_t;

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

/** @brief ESS event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_cscs, ble_ess_evt_t * p_evt);

/**@brief ESS init structure. This contains all options and data
*         needed for initialization of the service. */
typedef struct
{
    ble_ess_evt_handler_t       evt_handler;                /**< Event handler to be called for handling events in the ESS. */
    uint16_t                    feature;                    /**< Initial value for features of sensor @ref BLE_ESS_FEATURES. */
} ble_ess_init_t;

typedef struct ble_ess_meas_s
{
    bool        is_temperature_data_present;                /**< True if Temperature is present in the measurement. */
    bool        is_humidity_data_present;                   /**< True if Humidity is present in the measurement. */
    bool        is_pressure_data_present;                   /**< True if Pressure is present in the measurement. */
    int16_t    temperature;                                 /**< Temperature in Celsius. */
    uint16_t    humidity;                                   /**< Humidity in %. */
    uint32_t    pressure;                                   /**< Pressure in hPa. */
} ble_ess_meas_t;

/**
 * @brief ESS structure. This contains various status information for the service.
 */
struct ble_ess_s
{
    ble_ess_evt_handler_t        evt_handler;               /**< Event handler to be called for handling events in the ESS. */
    ble_srv_error_handler_t      error_handler;             /**< Function to be called in case of an error. */
    uint16_t                     service_handle;            /**< Handle of ESS (as provided by the BLE stack). */
    ble_gatts_char_handles_t     temperature_handles;       /**< Handles related to the ESS temperature characteristic. */
    ble_gatts_char_handles_t     humidity_handles;          /**< Handles related to the ESS humidity characteristic. */
    ble_gatts_char_handles_t     pressure_handles;          /**< Handles related to the ESS pressure characteristic. */
    uint16_t                     conn_handle;               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     feature;                   /**< Bit mask of features available on sensor. */
};

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

/**
 * @brief Function for initializing the Custom Service.
 *
 * @param[out]  p_ess       ESS structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ess_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init);

/**
 * @brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ESS.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   ESS structure.
 */
void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**
 * @brief Function for sending ESS measurements if notification has been enabled.
 *
 * @details The application calls this function after having performed a ESS measurement.
 *          If notification has been enabled, the measurement data is encoded
 *          and sent to the client.
 *
 * @param[in]   p_cscs         ESS structure.
 * @param[in]   p_measurement  Pointer to new ess measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_measurement_send(ble_ess_t * p_cscs, ble_ess_meas_t * p_measurement);

/**
 * @brief Function for updating ESS measurements.
 *
 * @details The application calls this function after having performed a ESS measurement.
 *          If notification has been enabled, the measurement data is encoded
 *          and sent to the client.
 *
 * @param[in]   p_cscs         ESS structure.
 * @param[in]   p_measurement  Pointer to new ess measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_measurement_update(ble_ess_t * p_ess, ble_ess_meas_t * p_measurement);

#endif /* APP_BLE_ESS_H_ */

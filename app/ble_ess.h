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

/**
 * @brief   Macro for defining a ble_ess instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ESS_DEF(_name) \
static ble_ess_t _name;

#define BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE 0x181A

/** @brief ESS event type. */
typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED,                                  /**< ESS value notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED                                  /**< ESS value notification disabled event. */
} ble_ess_evt_type_t;

/** @brief ESS event. */
typedef struct
{
    ble_ess_evt_type_t evt_type;                                       /**< Type of event. */
} ble_ess_evt_t;

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

/** @brief ESS event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_cscs, ble_ess_evt_t * p_evt);

/**@brief Cycling Speed and Cadence Service init structure. This contains all options and data
*         needed for initialization of the service. */
typedef struct
{
    ble_ess_evt_handler_t       evt_handler;                           /**< Event handler to be called for handling events in the ESS. */
} ble_ess_init_t;

/**
 * @brief ESS structure. This contains various status information for the service.
 */
struct ble_ess_s
{
    ble_ess_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the ESS. */
    ble_srv_error_handler_t      error_handler;                             /**< Function to be called in case of an error. */
    uint16_t                     service_handle;                            /**< Handle of ESS (as provided by the BLE stack). */
    ble_gatts_char_handles_t     temp_handles;                              /**< Handles related to the ESS temperature characteristic. */
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
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

#endif /* APP_BLE_ESS_H_ */

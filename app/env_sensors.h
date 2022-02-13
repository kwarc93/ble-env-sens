/*
 * env_sensors.h
 *
 *  Created on: 13 lut 2022
 *      Author: kwarc
 */

#ifndef APP_ENV_SENSORS_H_
#define APP_ENV_SENSORS_H_

typedef enum
{
    ENV_SENS_OK = 0,
    ENV_SENS_ERROR,
    ENV_SENS_TIMEOUT,
} env_sens_stat_t;

typedef struct
{
    float temperature;
    float humidity;
    float pressure;
} env_sens_data_t;

typedef void (*env_sens_drdy_cb_t)(const env_sens_data_t *data);

env_sens_stat_t env_sensors_init(void);
env_sens_stat_t env_sensors_deinit(void);
env_sens_stat_t env_sensors_pwr_on(void);
env_sens_stat_t env_sensors_pwr_off(void);
env_sens_stat_t env_sensors_trigger_measurement(env_sens_drdy_cb_t drdy_cb);
env_sens_stat_t env_sensors_get_data(env_sens_data_t *data);

#endif /* APP_ENV_SENSORS_H_ */

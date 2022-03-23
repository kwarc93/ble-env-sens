# Bluetooth LE Environmental sensor

BLE sensor (thermometer, hygrometer, pressure) running on 'Arduino Nano 33 BLE Sense board' (ABX00031)

## Features:

 - Temperature measurement
 - Relative Humidity measurement
 - Barometric pressure measurement
 - (TODO) Luminous intensity/flux measurement
 - Low power consumption: ~8uA at advertising, ~16uA at connection

## Overview

Low power Bluetooth Low Energy sensor. Uses standard BLE Environmental Sensing Service to share sensor data.
Sensors used for measurements: HTS221, LPS22HB, APDS-9960. Battery Service is used to show supply voltage (3V CR2032 cell). 

## Dependencies

- NRF5 SDK (v17.1.0)

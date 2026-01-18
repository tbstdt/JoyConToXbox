/**
 * @file ble_uuid.h
 * @brief Определения BLE UUID для сервисов и характеристик
 * 
 * Содержит стандартные UUID для BLE сервисов и характеристик:
 * - HID Service (0x1812)
 * - Battery Service (0x180F)
 * - Характеристики HID (Report, Control Point, Information, Report Map)
 * - Дескрипторы (CCCD для notifications)
 */

#ifndef BLE_UUID_H
#define BLE_UUID_H

#include <stdint.h>

// UUID сервисов BLE
#define BLE_UUID_HID_SERVICE           0x1812  // Human Interface Device Service
#define BLE_UUID_BATTERY_SERVICE       0x180F  // Battery Service

// UUID характеристик HID
#define BLE_UUID_HID_INFORMATION       0x2A4A  // HID Information
#define BLE_UUID_HID_REPORT_MAP        0x2A4B  // HID Report Map
#define BLE_UUID_HID_CONTROL_POINT     0x2A4C  // HID Control Point
#define BLE_UUID_HID_REPORT            0x2A4D  // HID Report
#define BLE_UUID_HID_PROTOCOL_MODE     0x2A4E  // HID Protocol Mode

// UUID характеристик Battery Service
#define BLE_UUID_BATTERY_LEVEL         0x2A19  // Battery Level

// UUID дескрипторов
#define BLE_UUID_CCCD                  0x2902  // Client Characteristic Configuration Descriptor

// Совместимость со старыми определениями (если используются разные имена)
#define BLE_UUID_HID_SVC               BLE_UUID_HID_SERVICE

#endif // BLE_UUID_H

/**
 * @file ble_hid.c
 * @brief Модуль BLE HID устройства (эмуляция Xbox контроллера)
 * 
 * Реализует BLE HID устройство, которое эмулирует Xbox контроллер:
 * - Регистрация HID сервиса и характеристик GATT
 * - Отправка HID отчетов хосту (компьютер/планшет)
 * - Получение команд вибрации от хоста
 * - Реклама устройства как "Xbox Controller"
 * 
 * Архитектура:
 * - Работает в периферийной роли BLE (SLAVE)
 * - Использует HID Report Descriptor для описания формата данных
 * - Поддерживает notifications для отправки отчетов
 * - Обрабатывает команды вибрации через HID Output характеристику
 * 
 * Взаимодействие:
 * - Получает HID отчеты от xbox_emulator.c
 * - Отправляет команды вибрации в joycon_manager.c через callback
 * - Использует Bluedroid BLE GATT сервер для работы в BTDM режиме
 */

#include "ble_hid.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "common/ble_uuid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BLE_HID";

// Совместимость со старыми именами
#define BLE_UUID_HID_SVC               BLE_UUID_HID_SERVICE

/**
 * @brief HID Report Descriptor для Xbox контроллера
 * 
 * Описывает структуру HID отчета для эмуляции Xbox контроллера через BLE HID.
 * Дескриптор определяет формат данных, которые отправляются хосту (компьютеру/планшету).
 * 
 * Структура отчета (20 байт):
 * - Byte 0-1: Заголовок (Report ID, размер)
 * - Byte 2-3: Кнопки (14 кнопок + 2 бита padding) и D-Pad (4 бита + 4 бита padding)
 * - Byte 4-5: Триггеры LT и RT (по 1 байту каждый)
 * - Byte 6-9: Левый стик X и Y (по 2 байта, signed 16-bit)
 * - Byte 10-13: Правый стик X и Y (по 2 байта, signed 16-bit)
 * - Byte 14-19: Зарезервировано
 * 
 * Формат HID дескриптора следует стандарту HID 1.11.
 */
static const uint8_t xbox_hid_report_descriptor[] = {
    // Блок 1: Начало коллекции Application (Game Pad)
    // Usage Page: Generic Desktop (0x01) - стандартная страница для игровых контроллеров
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad) - тип устройства: игровой контроллер
    0xa1, 0x01,        // Collection (Application) - начало коллекции приложения
    
    // Блок 2: Кнопки (14 кнопок: A, B, X, Y, LB, RB, Back, Start, Left Stick, Right Stick, View, Menu, и 2 зарезервированы)
    // Каждая кнопка - 1 бит (нажата/не нажата)
    0x05, 0x09,        //   Usage Page (Button) - страница кнопок
    0x19, 0x01,        //   Usage Minimum (0x01) - минимальный номер кнопки
    0x29, 0x0e,        //   Usage Maximum (0x0e) - максимальный номер кнопки (14 кнопок)
    0x15, 0x00,        //   Logical Minimum (0) - логический минимум (не нажата)
    0x25, 0x01,        //   Logical Maximum (1) - логический максимум (нажата)
    0x75, 0x01,        //   Report Size (1) - размер каждого поля в битах
    0x95, 0x0e,        //   Report Count (14) - количество полей (14 кнопок)
    0x81, 0x02,        //   Input (Data,Var,Abs,...) - входные данные, переменные, абсолютные значения
    
    // Блок 3: Padding для кнопок (2 бита) - заполнение до байта
    // Нужно для выравнивания: 14 кнопок = 14 бит, добавляем 2 бита = 16 бит = 2 байта
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2) - 2 бита padding
    0x81, 0x01,        //   Input (Const,...) - константные данные (не используются)
    
    // Блок 4: D-Pad (Hat switch) - 4 бита для 8 направлений
    // Значения: 0=Up, 1=Up-Right, 2=Right, 3=Down-Right, 4=Down, 5=Down-Left, 6=Left, 7=Up-Left, 8=Neutral
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x39,        //   Usage (Hat switch) - переключатель направлений
    0x15, 0x00,        //   Logical Minimum (0) - минимальное значение (Up)
    0x25, 0x07,        //   Logical Maximum (7) - максимальное значение (Up-Left)
    0x35, 0x00,        //   Physical Minimum (0) - физический минимум
    0x46, 0x3b, 0x01,  //   Physical Maximum (315) - физический максимум в градусах (8 направлений * 45°)
    0x65, 0x14,        //   Unit (Degrees) - единица измерения: градусы
    0x75, 0x04,        //   Report Size (4) - 4 бита для D-Pad
    0x95, 0x01,        //   Report Count (1) - одно поле
    0x81, 0x42,        //   Input (Data,Var,Abs,Null State) - входные данные с нулевым состоянием
    
    // Блок 5: Padding для D-Pad (4 бита) - заполнение до байта
    // D-Pad занимает 4 бита, добавляем 4 бита = 8 бит = 1 байт
    0x65, 0x00,        //   Unit (None) - без единиц измерения
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1) - 4 бита padding
    0x81, 0x01,        //   Input (Const,...) - константные данные
    
    // Блок 6: Триггеры (LT, RT) - 2 байта (по 1 байту на триггер)
    // Значения: 0-255 (0 = не нажат, 255 = полностью нажат)
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x32,        //   Usage (Z) - Left Trigger (ось Z)
    0x09, 0x35,        //   Usage (Rz) - Right Trigger (ось Rz)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xff, 0x00,  //   Logical Maximum (255) - little-endian формат
    0x75, 0x08,        //   Report Size (8) - 8 бит = 1 байт на триггер
    0x95, 0x02,        //   Report Count (2) - два триггера
    0x81, 0x02,        //   Input (Data,Var,Abs,...) - входные данные
    
    // Блок 7: Стики (4 оси: LX, LY, RX, RY) - 8 байт (по 2 байта на ось)
    // Значения: signed 16-bit (-32767 до +32767)
    // Центр: 0, отрицательные значения: влево/вверх, положительные: вправо/вниз
    0x09, 0x30,        //   Usage (X) - Left Stick X
    0x09, 0x31,        //   Usage (Y) - Left Stick Y
    0x09, 0x33,        //   Usage (Rx) - Right Stick X
    0x09, 0x34,        //   Usage (Ry) - Right Stick Y
    0x16, 0x01, 0x80,  //   Logical Minimum (-32767) - little-endian, signed
    0x26, 0xff, 0x7f,  //   Logical Maximum (32767) - little-endian, signed
    0x75, 0x10,        //   Report Size (16) - 16 бит = 2 байта на ось
    0x95, 0x04,        //   Report Count (4) - четыре оси
    0x81, 0x02,        //   Input (Data,Var,Abs,...) - входные данные
    
    // Блок 8: Output для вибрации (1 байт)
    // Используется для получения команд вибрации от хоста
    // Значение: 0-255 (сила вибрации)
    0x06, 0x00, 0xff,  //   Usage Page (Vendor Defined 0xff00) - пользовательская страница
    0x09, 0x20,        //   Usage (0x20) - пользовательское использование
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xff, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8) - 8 бит = 1 байт
    0x95, 0x01,        //   Report Count (1) - одно поле
    0x91, 0x02,        //   Output (Data,Var,Abs,Non-volatile) - выходные данные (команды от хоста)
    
    // Конец коллекции
    0xc0,              // End Collection - конец коллекции Application
};

// HID Information (версия, страна, флаги)
static const uint8_t hid_information[] = {
    0x01, 0x01,  // bcdHID (версия 1.01)
    0x00,        // bCountryCode (0 = не поддерживается)
    0x01,        // Flags (Remote Wake = 1, Normally Connectable = 0)
};

// Структура для хранения GATT handles
typedef struct {
    uint16_t service_handle;
    uint16_t char_hid_info_handle;
    uint16_t char_hid_info_val_handle;
    uint16_t char_hid_report_map_handle;
    uint16_t char_hid_report_map_val_handle;
    uint16_t char_hid_control_point_handle;
    uint16_t char_hid_control_point_val_handle;
    uint16_t char_hid_protocol_mode_handle;     // Protocol Mode (обязательна для HID)
    uint16_t char_hid_protocol_mode_val_handle;
    uint16_t char_hid_report_input_handle;      // Input Report (notify)
    uint16_t char_hid_report_input_val_handle;
    uint16_t char_hid_report_output_handle;     // Output Report (write)
    uint16_t char_hid_report_output_val_handle;
    uint16_t descr_cccd_handle;                 // CCCD для notifications
} hid_gatt_handles_t;

static ble_hid_vibration_callback_t vibration_callback = NULL;
static bool ble_hid_ready = false;
static bool ble_hid_advertising = false;
static bool ble_hid_services_registered = false;
static hid_gatt_handles_t gatt_handles = {0};
static uint16_t gatt_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = ESP_GATT_IF_NONE;
static bool cccd_subscribed = false;  // Флаг подписки на notifications

// Буфер для хранения последнего HID отчета
static uint8_t last_hid_report[XBOX_HID_REPORT_SIZE] = {0};
// Время последней отправки HID отчета (для throttling)
static uint32_t last_report_time = 0;

// UUID определения
static const uint16_t hid_service_uuid = BLE_UUID_HID_SERVICE;
static const uint16_t hid_info_char_uuid = BLE_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_char_uuid = BLE_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_char_uuid = BLE_UUID_HID_CONTROL_POINT;
static const uint16_t hid_protocol_mode_char_uuid = BLE_UUID_HID_PROTOCOL_MODE;
static const uint16_t hid_report_char_uuid = BLE_UUID_HID_REPORT;
static const uint16_t cccd_uuid = BLE_UUID_CCCD;

// HID Protocol Mode значение (0x00 = Boot Mode, 0x01 = Report Mode)
// Android и большинство устройств требуют Report Mode (0x01)
static uint8_t hid_protocol_mode = 0x01;

// GATTS callback
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            if (param->reg.status == ESP_GATT_OK) {
                gatt_if = gatts_if;
                esp_gatt_srvc_id_t srvc_id = {
                    .id = {
                        .uuid = {
                            .len = ESP_UUID_LEN_16,
                            .uuid = {.uuid16 = hid_service_uuid}
                        },
                        .inst_id = 0
                    },
                    .is_primary = true
                };
                esp_ble_gatts_create_service(gatts_if, &srvc_id, 30);  // 30 handles должно быть достаточно для HID сервиса
                ESP_LOGI(TAG, "GATTS app registered, gatt_if=%d", gatts_if);
            } else {
                ESP_LOGE(TAG, "GATTS app registration failed: %d", param->reg.status);
            }
            break;
        }
        
        case ESP_GATTS_CREATE_EVT: {
            if (param->create.status == ESP_GATT_OK) {
                gatt_handles.service_handle = param->create.service_handle;
                ESP_LOGI(TAG, "HID service created, handle=%d", gatt_handles.service_handle);
                
                // Добавляем характеристики после создания сервиса
                // 1. HID Information
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_info_char_uuid}
                }, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, NULL, NULL);
                
                // 2. HID Report Map
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_report_map_char_uuid}
                }, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ, NULL, NULL);
                
                // 3. HID Control Point
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_control_point_char_uuid}
                }, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE_NR, NULL, NULL);
                
                // 4. HID Protocol Mode (обязательна для HID, должна быть перед Report)
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_protocol_mode_char_uuid}
                }, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, NULL, NULL);
                
                // 5. HID Report Input (для notifications)
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_report_char_uuid}
                }, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, NULL, NULL);
                
                // 6. HID Report Output (для записи вибрации)
                esp_ble_gatts_add_char(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = hid_report_char_uuid}
                }, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Service creation failed: %d", param->create.status);
            }
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_EVT: {
            if (param->add_char.status == ESP_GATT_OK) {
                uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;
                
                if (char_uuid == hid_info_char_uuid) {
                    gatt_handles.char_hid_info_handle = param->add_char.attr_handle;
                    gatt_handles.char_hid_info_val_handle = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "HID Information char added, handle=%d, val_handle=%d", 
                             gatt_handles.char_hid_info_handle, gatt_handles.char_hid_info_val_handle);
                } else if (char_uuid == hid_report_map_char_uuid) {
                    gatt_handles.char_hid_report_map_handle = param->add_char.attr_handle;
                    gatt_handles.char_hid_report_map_val_handle = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "HID Report Map char added, handle=%d, val_handle=%d", 
                             gatt_handles.char_hid_report_map_handle, gatt_handles.char_hid_report_map_val_handle);
                } else if (char_uuid == hid_control_point_char_uuid) {
                    gatt_handles.char_hid_control_point_handle = param->add_char.attr_handle;
                    gatt_handles.char_hid_control_point_val_handle = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "HID Control Point char added, handle=%d, val_handle=%d", 
                             gatt_handles.char_hid_control_point_handle, gatt_handles.char_hid_control_point_val_handle);
                } else if (char_uuid == hid_protocol_mode_char_uuid) {
                    gatt_handles.char_hid_protocol_mode_handle = param->add_char.attr_handle;
                    gatt_handles.char_hid_protocol_mode_val_handle = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "HID Protocol Mode char added, handle=%d, val_handle=%d", 
                             gatt_handles.char_hid_protocol_mode_handle, gatt_handles.char_hid_protocol_mode_val_handle);
                } else if (char_uuid == hid_report_char_uuid) {
                    // Может быть Input или Output Report
                    if (gatt_handles.char_hid_report_input_val_handle == 0) {
                        // Это Input Report
                        gatt_handles.char_hid_report_input_handle = param->add_char.attr_handle;
                        gatt_handles.char_hid_report_input_val_handle = param->add_char.attr_handle;
                        ESP_LOGI(TAG, "HID Report Input char added, handle=%d, val_handle=%d", 
                                 gatt_handles.char_hid_report_input_handle, gatt_handles.char_hid_report_input_val_handle);
                        
                        // Добавляем CCCD дескриптор для Input Report
                        esp_ble_gatts_add_char_descr(gatt_handles.service_handle, &(esp_bt_uuid_t) {
                            .len = ESP_UUID_LEN_16,
                            .uuid = {.uuid16 = cccd_uuid}
                        }, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
                    } else {
                        // Это Output Report
                        gatt_handles.char_hid_report_output_handle = param->add_char.attr_handle;
                        gatt_handles.char_hid_report_output_val_handle = param->add_char.attr_handle;
                        ESP_LOGI(TAG, "HID Report Output char added, handle=%d, val_handle=%d", 
                                 gatt_handles.char_hid_report_output_handle, gatt_handles.char_hid_report_output_val_handle);
                        
                        // Все характеристики добавлены, можно запускать сервис
                        esp_ble_gatts_start_service(gatt_handles.service_handle);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Characteristic addition failed: %d", param->add_char.status);
            }
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            if (param->add_char_descr.status == ESP_GATT_OK) {
                gatt_handles.descr_cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(TAG, "CCCD descriptor added, handle=%d", gatt_handles.descr_cccd_handle);
            }
            break;
        }
        
        case ESP_GATTS_START_EVT: {
            if (param->start.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "HID service started successfully");
                ble_hid_services_registered = true;
                // Запускаем рекламу после успешного запуска сервиса
                ble_hid_start_advertising();
            } else {
                ESP_LOGE(TAG, "Service start failed: %d", param->start.status);
            }
            break;
        }
        
        case ESP_GATTS_CONNECT_EVT: {
            conn_id = param->connect.conn_id;
            ESP_LOGI(TAG, "BLE HID connection attempt from %02x:%02x:%02x:%02x:%02x:%02x, conn_id=%d",
                     param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                     param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5], conn_id);
            
            // Параметры соединения - оптимизированы для Android
            // Android требует достаточно большие интервалы для стабильности
            // Для HID рекомендуется: min 12-20ms, max 40-50ms
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;  // Нет задержки для низкой латентности
            conn_params.max_int = 0x28;  // 50ms (40 * 1.25ms) - оптимально для Android HID
            conn_params.min_int = 0x0C;  // 15ms (12 * 1.25ms) - минимальная задержка для HID
            conn_params.timeout = 600;   // 6 seconds (600 * 10ms) - увеличен для Android стабильности
            
            esp_err_t conn_update_ret = esp_ble_gap_update_conn_params(&conn_params);
            if (conn_update_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to update connection parameters: %d (0x%x)", conn_update_ret, conn_update_ret);
            } else {
                ESP_LOGI(TAG, "Connection parameters updated: min_int=%d, max_int=%d, latency=%d, timeout=%d",
                         conn_params.min_int, conn_params.max_int, conn_params.latency, conn_params.timeout);
            }
            
            ESP_LOGI(TAG, "BLE HID connected, conn_id=%d", conn_id);
            ble_hid_ready = true;
            break;
        }
        
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "BLE HID disconnected, conn_id=%d", conn_id);
            conn_id = ESP_GATT_IF_NONE;
            ble_hid_ready = false;
            cccd_subscribed = false;
            // Перезапускаем рекламу после отключения
            if (ble_hid_advertising) {
                ble_hid_advertising = false;
                vTaskDelay(pdMS_TO_TICKS(100));
                ble_hid_start_advertising();
            }
            break;
        }
        
        case ESP_GATTS_READ_EVT: {
            uint16_t handle = param->read.handle;
            
            if (handle == gatt_handles.char_hid_info_val_handle) {
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.len = sizeof(hid_information);
                memcpy(rsp.attr_value.value, hid_information, sizeof(hid_information));
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                           ESP_GATT_OK, &rsp);
            } else if (handle == gatt_handles.char_hid_report_map_val_handle) {
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.len = sizeof(xbox_hid_report_descriptor);
                memcpy(rsp.attr_value.value, xbox_hid_report_descriptor, sizeof(xbox_hid_report_descriptor));
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                           ESP_GATT_OK, &rsp);
            } else if (handle == gatt_handles.char_hid_report_input_val_handle) {
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.len = XBOX_HID_REPORT_SIZE;
                memcpy(rsp.attr_value.value, last_hid_report, XBOX_HID_REPORT_SIZE);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                           ESP_GATT_OK, &rsp);
            } else if (handle == gatt_handles.char_hid_protocol_mode_val_handle) {
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.len = 1;
                rsp.attr_value.value[0] = hid_protocol_mode;
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                           ESP_GATT_OK, &rsp);
            }
            break;
        }
        
        case ESP_GATTS_WRITE_EVT: {
            uint16_t handle = param->write.handle;
            
            if (handle == gatt_handles.descr_cccd_handle) {
                // CCCD descriptor - проверяем подписку на notifications
                if (param->write.len == 2) {
                    uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                    cccd_subscribed = (cccd_value & 0x0001) != 0;
                    ESP_LOGI(TAG, "CCCD write: %04x, subscribed=%d", cccd_value, cccd_subscribed);
                }
            } else if (handle == gatt_handles.char_hid_report_output_val_handle) {
                // Output Report - команда вибрации
                if (param->write.len >= 2) {
                    uint8_t left_motor = param->write.value[1];
                    uint8_t right_motor = (param->write.len > 2) ? param->write.value[2] : 0;
                    
                    ESP_LOGD(TAG, "Received vibration command: Left=%d, Right=%d", left_motor, right_motor);
                    
                    if (vibration_callback != NULL) {
                        vibration_callback(left_motor, right_motor);
                    }
                }
            } else if (handle == gatt_handles.char_hid_control_point_val_handle) {
                // Control Point - обычно игнорируем (Suspend/Resume)
                ESP_LOGD(TAG, "HID Control Point write received");
            } else if (handle == gatt_handles.char_hid_protocol_mode_val_handle) {
                // Protocol Mode - должен быть 0x00 (Boot) или 0x01 (Report)
                if (param->write.len == 1) {
                    uint8_t new_mode = param->write.value[0];
                    if (new_mode == 0x00 || new_mode == 0x01) {
                        hid_protocol_mode = new_mode;
                        ESP_LOGI(TAG, "HID Protocol Mode set to: %s (0x%02x)", 
                                new_mode == 0x01 ? "Report Mode" : "Boot Mode", new_mode);
                    } else {
                        ESP_LOGW(TAG, "Invalid Protocol Mode value: 0x%02x (must be 0x00 or 0x01)", 
                                param->write.value[0]);
                    }
                }
            }
            break;
        }
        
        default:
            break;
    }
}

esp_err_t ble_hid_init(void)
{
    if (ble_hid_services_registered) {
        ESP_LOGW(TAG, "BLE HID services already registered, skipping");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE HID...");
    
    // Устанавливаем имя устройства для BLE заранее, до регистрации сервисов
    // Это важно для BTDM режима, чтобы имя было установлено до начала рекламы
    const char *device_name = "JoyConXbox";
    esp_err_t name_ret = esp_ble_gap_set_device_name(device_name);
    if (name_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BLE device name: %d (0x%x)", name_ret, name_ret);
        // Продолжаем, но имя может не установиться правильно
    } else {
        ESP_LOGI(TAG, "BLE device name set to '%s'", device_name);
    }
    
    // Регистрируем GATTS приложение
    esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTS callback: %d", ret);
        return ret;
    }
    
    // Регистрируем приложение
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTS app: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE HID initialization started, waiting for service registration...");
    
    return ESP_OK;
}

void ble_hid_start_advertising(void)
{
    if (ble_hid_advertising) {
        ESP_LOGD(TAG, "HID advertising already started");
        return;
    }
    
    if (!ble_hid_services_registered) {
        ESP_LOGW(TAG, "Services not registered yet, cannot start advertising");
        return;
    }
    
    const char *device_name = "JoyConXbox";
    
    // Имя устройства уже должно быть установлено в ble_hid_init(),
    // но убедимся, что оно установлено перед началом рекламы
    // (на случай, если было перезаписано)
    esp_err_t name_ret = esp_ble_gap_set_device_name(device_name);
    if (name_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set BLE device name (may already be set): %d (0x%x)", name_ret, name_ret);
    }
    
    // Настройка данных рекламы с UUID сервиса для лучшей видимости на Android
    // Android лучше видит устройства с UUID сервиса в основной рекламе
    uint8_t adv_uuid[2] = {(uint8_t)(hid_service_uuid & 0xFF), (uint8_t)((hid_service_uuid >> 8) & 0xFF)};
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,  // Отключаем txpower для уменьшения размера
        .min_interval = 0x30,  // Синхронизируем с параметрами рекламы
        .max_interval = 0x60,
        .appearance = ESP_BLE_APPEARANCE_GENERIC_HID,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 2,  // Включаем UUID в основную рекламу для Android
        .p_service_uuid = adv_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
    };
    
    esp_err_t adv_data_ret = esp_ble_gap_config_adv_data(&adv_data);
    if (adv_data_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure advertising data: %d (0x%x) - retrying without flags", adv_data_ret, adv_data_ret);
        // Пробуем без флагов
        adv_data.flag = 0;
        adv_data_ret = esp_ble_gap_config_adv_data(&adv_data);
        if (adv_data_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure advertising data (retry): %d (0x%x)", adv_data_ret, adv_data_ret);
            return;  // Не продолжаем, если настройка не удалась
        }
    }
    
    // Настройка scan response с UUID сервиса
    // UUID должен быть в little-endian формате для BLE (LSB первый)
    uint8_t scan_rsp_uuid[2] = {(uint8_t)(hid_service_uuid & 0xFF), (uint8_t)((hid_service_uuid >> 8) & 0xFF)};
    esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,  // Включаем имя в scan response для лучшей видимости
        .include_txpower = false,
        .min_interval = 0,
        .max_interval = 0,
        .appearance = ESP_BLE_APPEARANCE_GENERIC_HID,  // Указываем appearance и в scan response
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 2,
        .p_service_uuid = scan_rsp_uuid,
        .flag = 0
    };
    
    esp_err_t scan_rsp_ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (scan_rsp_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure scan response data: %d (0x%x) - continuing anyway", scan_rsp_ret, scan_rsp_ret);
    }
    
    // Параметры рекламы - оптимизированы для Android совместимости
    // Android лучше работает с более медленной, но стабильной рекламой
    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x30,  // 48 * 0.625ms = 30ms (более стабильно для Android)
        .adv_int_max = 0x60,  // 96 * 0.625ms = 60ms
        .adv_type = ADV_TYPE_IND,  // Indicate (connectable undirected) - требуется для HID
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,  // Публичный адрес для лучшей совместимости
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,  // Принимаем все подключения
    };
    
    esp_err_t adv_start_ret = esp_ble_gap_start_advertising(&adv_params);
    if (adv_start_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %d (0x%x)", adv_start_ret, adv_start_ret);
        return;
    }
    
    ESP_LOGI(TAG, "BLE HID advertising started as '%s'", device_name);
    ESP_LOGI(TAG, "Advertising details: name='%s', appearance=0x%04x, service_uuid=0x%04x (in scan response)",
             device_name, adv_data.appearance, hid_service_uuid);
    ble_hid_advertising = true;
}

void ble_hid_stop_advertising(void)
{
    if (!ble_hid_advertising) {
        return;
    }
    
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "BLE HID advertising stopped");
    ble_hid_advertising = false;
}

esp_err_t ble_hid_send_report(const uint8_t *report, size_t len)
{
    if (report == NULL) {
        ESP_LOGE(TAG, "Report buffer pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (len < XBOX_HID_REPORT_SIZE) {
        ESP_LOGE(TAG, "Invalid report size: %d bytes (minimum %d bytes)", len, XBOX_HID_REPORT_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    if (len > XBOX_HID_REPORT_SIZE * 2) {
        ESP_LOGE(TAG, "Report size too large: %d bytes (maximum %d bytes)", len, XBOX_HID_REPORT_SIZE * 2);
        return ESP_ERR_INVALID_SIZE;
    }
    
    if (!ble_hid_ready || conn_id == ESP_GATT_IF_NONE) {
        ESP_LOGD(TAG, "BLE HID not ready or not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (gatt_handles.char_hid_report_input_val_handle == 0) {
        ESP_LOGW(TAG, "HID report input characteristic handle not set");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!cccd_subscribed) {
        ESP_LOGD(TAG, "Client not subscribed to notifications");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Throttling
    uint32_t now = xTaskGetTickCount();
    if (last_report_time != 0) {
        uint32_t elapsed_ticks = now - last_report_time;
        uint32_t min_interval_ticks = pdMS_TO_TICKS(MIN_REPORT_INTERVAL_MS);
        
        if (elapsed_ticks < min_interval_ticks) {
            ESP_LOGD(TAG, "HID report throttled (elapsed=%lu ms, min=%d ms)", 
                     elapsed_ticks * portTICK_PERIOD_MS, MIN_REPORT_INTERVAL_MS);
            return ESP_ERR_INVALID_STATE;
        }
    }
    last_report_time = now;
    
    // Сохраняем отчет
    memcpy(last_hid_report, report, XBOX_HID_REPORT_SIZE);
    
    // Отправка через notify
    esp_err_t ret = esp_ble_gatts_send_indicate(gatt_if, conn_id,
                                                 gatt_handles.char_hid_report_input_val_handle,
                                                 XBOX_HID_REPORT_SIZE, last_hid_report, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send HID report notify: %d", ret);
        return ret;
    }
    
    ESP_LOGD(TAG, "HID report sent successfully (conn_id=%d)", conn_id);
    return ESP_OK;
}

void ble_hid_set_vibration_callback(ble_hid_vibration_callback_t callback)
{
    vibration_callback = callback;
}

bool ble_hid_is_ready(void)
{
    return ble_hid_ready;
}

// Пустая функция для совместимости с API (GAP events обрабатываются через GATTS callback)
void ble_hid_gap_event_handler(void *event)
{
    // GAP события обрабатываются через esp_ble_gap_cb_t callback, если нужен
}

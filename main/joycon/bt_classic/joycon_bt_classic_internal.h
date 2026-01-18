/**
 * @file joycon_bt_classic_internal.h
 * @brief Внутренние структуры и функции для модуля Bluetooth Classic подключения Joy-Con
 * 
 * Этот файл содержит определения внутренних структур, типов и констант,
 * используемых между модулями bt_classic. Не должен включаться из внешних модулей.
 */

#ifndef JOYCON_BT_CLASSIC_INTERNAL_H
#define JOYCON_BT_CLASSIC_INTERNAL_H

#include "joycon_bt_classic.h"
#include "../joycon_constants.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_gap_bt_api.h"

// Макрос для безопасного определения размера массива
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

// Константы для Bluetooth Classic сканирования
#define BT_CLASSIC_INQUIRY_LENGTH 8  // 8 * 1.28s = ~10 секунд
#define BT_CLASSIC_INQUIRY_MAX_RSP 0  // Без ограничения

// Константы для подключения
#define BT_CLASSIC_CONNECTION_TIMEOUT_MS 10000
#define BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS 100  // Увеличено с 50ms для предотвращения потери данных при высокой нагрузке
#define BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS 50

// Макрос для форматирования MAC-адреса
// В ESP-IDF MAC-адреса хранятся в стандартном порядке (big-endian): addr[0] - первый байт
#define MAC_STR(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

// UUID для HID сервиса (Bluetooth Classic)
#define BT_CLASSIC_HID_SERVICE_UUID 0x1124

/**
 * @brief Структура для хранения информации о подключенном Joy-Con через Classic
 */
typedef struct {
    uint8_t addr[6];
    joycon_state_t state;
    joycon_type_t type;
    bool connected;
    bool connecting;
    bool sdp_complete;
    uint16_t hid_psm;  // PSM для HID L2CAP канала
    uint16_t hid_interrupt_psm;  // PSM для HID Interrupt канала
    uint16_t hid_control_psm;  // PSM для HID Control канала
    uint32_t connecting_start_tick;  // Время начала попытки подключения (для таймаута)
    // L2CAP каналы
    bool l2cap_interrupt_connected;  // Флаг подключения Interrupt канала
    bool l2cap_control_connected;    // Флаг подключения Control канала
    uint32_t l2cap_interrupt_handle; // Handle Interrupt канала
    uint32_t l2cap_control_handle;   // Handle Control канала
    int l2cap_interrupt_fd;          // File descriptor Interrupt канала (для отправки/получения данных)
    int l2cap_control_fd;            // File descriptor Control канала (для отправки команд)
} joycon_bt_classic_connection_t;

/**
 * @brief Структура для накопления данных об обнаруженных Classic устройствах
 */
typedef struct {
    uint8_t addr[6];
    char name[32];
    bool has_name;
    bool has_hid_service;
    int8_t rssi;
    uint32_t last_seen_tick;
    bool logged_as_new_device;
    bool logged_as_joycon;
    bool sdp_requested;
    bool sdp_complete;
} discovered_bt_classic_device_t;

// Максимальное количество устройств в кэше
#define MAX_DISCOVERED_BT_CLASSIC_DEVICES 10
#define BT_CLASSIC_DEVICE_CACHE_TIMEOUT_MS 30000

// Внешние объявления для использования между модулями
// ВНИМАНИЕ: left_joycon и right_joycon теперь инкапсулированы в joycon_bt_classic_state.c
// Используйте joycon_bt_classic_state_get_connection() для доступа

extern SemaphoreHandle_t connection_mutex;
extern SemaphoreHandle_t device_cache_mutex;

// Внешние функции для обратной совместимости
// ВНИМАНИЕ: get_connection() теперь делегирует работу модулю state
extern joycon_bt_classic_connection_t* get_connection(joycon_type_t type);
extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_connection(joycon_type_t type);
extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);

// Макросы для обратной совместимости (используются внутри мьютексов)
// ВНИМАНИЕ: Эти макросы возвращают указатели на структуры.
// Мьютекс connection_mutex ДОЛЖЕН быть захвачен перед использованием!
#define left_joycon (*joycon_bt_classic_state_get_left_joycon())
#define right_joycon (*joycon_bt_classic_state_get_right_joycon())
extern bool is_mac_addr_zero(const uint8_t *addr);
extern bool is_valid_mac_address(const uint8_t *addr);

// update_state() теперь в joycon_bt_classic_state.c
// Используйте joycon_bt_classic_state_set() вместо update_state()
// Объявление update_state() оставлено для обратной совместимости в joycon_bt_classic.c
extern void update_state(joycon_type_t type, joycon_state_t new_state);
extern esp_err_t joycon_bt_classic_state_set(joycon_type_t type, joycon_state_t new_state);

// Макросы для безопасного захвата/освобождения мьютекса
#define TAKE_CONN_MUTEX(timeout_ms) \
    (connection_mutex != NULL && xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)

#define RELEASE_CONN_MUTEX() \
    do { if (connection_mutex != NULL) xSemaphoreGive(connection_mutex); } while(0)

#define SAFE_CALL_CALLBACK(cb, ...) \
    do { if (cb) cb(__VA_ARGS__); } while(0)

#endif // JOYCON_BT_CLASSIC_INTERNAL_H

/**
 * @file joycon_bt_classic.c
 * @brief Базовый модуль Bluetooth Classic подключения Joy-Con - инициализация и общие функции
 * 
 * Содержит:
 * - Инициализацию Bluedroid стека
 * - Общие функции для работы с подключениями
 * - Работу с NVS для сохранения MAC-адресов
 * - Callbacks для состояния и данных
 * - Синхронизацию Bluedroid стека
 */

#include "joycon_bt_classic.h"
#include "joycon_bt_classic_internal.h"
#include "../joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "JOYCON_BT_CLASSIC";

// ВНИМАНИЕ: left_joycon и right_joycon теперь инкапсулированы в joycon_bt_classic_state.c
// Используйте joycon_bt_classic_state_get_connection() для доступа
// ВНИМАНИЕ: state_callback теперь инкапсулирован в joycon_bt_classic_state.c

// Callbacks
joycon_bt_classic_data_callback_t data_callback = NULL;

// Состояние Bluedroid стека
bool bt_classic_initialized = false;
bool bt_classic_enabled = false;

// Синхронизация
SemaphoreHandle_t connection_mutex = NULL;
SemaphoreHandle_t device_cache_mutex = NULL;
SemaphoreHandle_t bt_init_semaphore = NULL;

// Forward declarations
static esp_err_t save_address_to_nvs(joycon_type_t type, const uint8_t *addr);
static esp_err_t load_address_from_nvs(joycon_type_t type, uint8_t *addr);

/**
 * @brief Получить структуру подключения по типу
 * 
 * ВНИМАНИЕ: Использует модуль state для получения структур подключения
 * Мьютекс connection_mutex должен быть захвачен вызывающей стороной
 */
joycon_bt_classic_connection_t* get_connection(joycon_type_t type)
{
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_connection(joycon_type_t type);
    return joycon_bt_classic_state_get_connection(type);
}

/**
 * @brief Проверить, является ли MAC адрес нулевым
 */
bool is_mac_addr_zero(const uint8_t *addr)
{
    static const uint8_t zero_addr[6] = {0, 0, 0, 0, 0, 0};
    return memcmp(addr, zero_addr, 6) == 0;
}

/**
 * @brief Проверить валидность MAC-адреса
 */
bool is_valid_mac_address(const uint8_t *addr)
{
    if (!addr) {
        return false;
    }
    
    if (is_mac_addr_zero(addr)) {
        return false;
    }
    
    // Проверка на multicast: младший бит первого байта не должен быть установлен
    if ((addr[0] & 0x01) != 0) {
        return false;
    }
    
    // Проверка на broadcast адрес (все байты = 0xFF)
    bool is_broadcast = true;
    for (int i = 0; i < 6; i++) {
        if (addr[i] != 0xFF) {
            is_broadcast = false;
            break;
        }
    }
    if (is_broadcast) {
        return false;
    }
    
    // Joy-Con обычно имеют OUI начинающиеся с 00:9F или 98:B6
    // Также Joy-Con могут использовать LAA (Locally Administered Address) адреса
    // Проверяем известные паттерны OUI или разрешаем любые валидные MAC адреса
    bool looks_like_joycon = (addr[5] == 0x00 && addr[4] == 0x9f) ||
                            (addr[5] == 0x98 && addr[4] == 0xb6);
    
    if (looks_like_joycon) {
        return true;
    }
    
    // LAA (Locally Administered Address) адреса валидны для Bluetooth устройств
    // Joy-Con могут использовать LAA адреса, особенно если они настроены локально
    // Разрешаем любые адреса, которые прошли базовые проверки (не multicast, не broadcast)
    return true;
}

/**
 * @brief Обновить состояние подключения и вызвать callback
 * 
 * ВНИМАНИЕ: Эта функция теперь делегирует работу модулю state
 * Оставлена для обратной совместимости
 * Возвращает void для совместимости со старым кодом, но игнорирует код возврата
 */
void update_state(joycon_type_t type, joycon_state_t new_state)
{
    extern esp_err_t joycon_bt_classic_state_set(joycon_type_t type, joycon_state_t new_state);
    // Игнорируем код возврата для обратной совместимости (старая функция была void)
    (void)joycon_bt_classic_state_set(type, new_state);
}

/**
 * @brief Callback для событий GAP
 */
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    // Обработка событий GAP будет в joycon_bt_classic_scan.c
    extern void joycon_bt_classic_handle_gap_event(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    joycon_bt_classic_handle_gap_event(event, param);
}

/**
 * @brief Сохранение MAC адреса в NVS
 */
static esp_err_t save_address_to_nvs(joycon_type_t type, const uint8_t *addr)
{
    if (!is_valid_mac_address(addr)) {
        ESP_LOGW(TAG, "Invalid MAC address, not saving to NVS");
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("joycon_bt", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", err);
        return err;
    }
    
    const char *key = (type == JOYCON_TYPE_LEFT) ? "left_addr" : "right_addr";
    err = nvs_set_blob(nvs_handle, key, addr, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save address to NVS: %d", err);
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved %s Joy-Con address to NVS: %02x:%02x:%02x:%02x:%02x:%02x",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right",
                 MAC_STR(addr));
    }
    
    return err;
}

/**
 * @brief Загрузка MAC адреса из NVS
 */
static esp_err_t load_address_from_nvs(joycon_type_t type, uint8_t *addr)
{
    if (!addr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("joycon_bt", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Failed to open NVS (may be first run): %d", err);
        return err;
    }
    
    const char *key = (type == JOYCON_TYPE_LEFT) ? "left_addr" : "right_addr";
    size_t required_size = 6;
    err = nvs_get_blob(nvs_handle, key, addr, &required_size);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK && required_size == 6 && is_valid_mac_address(addr)) {
        ESP_LOGI(TAG, "Loaded %s Joy-Con address from NVS: %02x:%02x:%02x:%02x:%02x:%02x",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right",
                 MAC_STR(addr));
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Инициализация Bluedroid стека
 */
esp_err_t joycon_bt_classic_init(void)
{
    if (bt_classic_initialized) {
        ESP_LOGW(TAG, "Bluedroid already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing Bluedroid stack...");
    
    // В BTDM режиме контроллер уже инициализирован для BLE
    // Проверяем, инициализирован ли контроллер
    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
        // Контроллер не инициализирован - инициализируем в BTDM режиме
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BT controller: %d", ret);
            return ret;
        }
        
        ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable BT controller in BTDM mode: %d", ret);
            esp_bt_controller_deinit();
            return ret;
        }
        ESP_LOGI(TAG, "BT controller enabled in BTDM mode");
    } else if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_LOGI(TAG, "BT controller already enabled (BTDM mode)");
    } else {
        ESP_LOGW(TAG, "BT controller status: %d", status);
    }
    
    // Инициализация Bluedroid
    esp_err_t ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %d", ret);
        // В BTDM режиме не деинициализируем контроллер, так как он используется для BLE
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %d", ret);
        esp_bluedroid_deinit();
        return ret;
    }
    
    // Регистрация GAP callback
    ret = esp_bt_gap_register_callback(bt_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %d", ret);
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        return ret;
    }
    
    // Инициализация L2CAP модуля для работы с Joy-Con HID каналами
    // Должна вызываться после esp_bluedroid_enable()
    extern esp_err_t joycon_bt_classic_l2cap_init(void);
    ret = joycon_bt_classic_l2cap_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP: %d", ret);
        // Не критично - возможно L2CAP API недоступен, продолжим без него
        // но это может привести к проблемам с получением данных от Joy-Con
        ESP_LOGW(TAG, "Continuing without L2CAP - Joy-Con data may not be received");
    }
    
    // Создание мьютексов
    connection_mutex = xSemaphoreCreateMutex();
    device_cache_mutex = xSemaphoreCreateMutex();
    bt_init_semaphore = xSemaphoreCreateBinary();
    
    if (!connection_mutex || !device_cache_mutex || !bt_init_semaphore) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return ESP_ERR_NO_MEM;
    }
    
    // Инициализация модуля управления состоянием (после создания мьютексов)
    extern esp_err_t joycon_bt_classic_state_init(void);
    ret = joycon_bt_classic_state_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize state module: %d", ret);
        vSemaphoreDelete(connection_mutex);
        vSemaphoreDelete(device_cache_mutex);
        vSemaphoreDelete(bt_init_semaphore);
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return ret;
    }
    
    // Загрузка сохраненных адресов в структуры подключения через state модуль
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_connection(joycon_type_t type);
    uint8_t addr[6];
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (load_address_from_nvs(JOYCON_TYPE_LEFT, addr) == ESP_OK) {
            joycon_bt_classic_connection_t *conn = joycon_bt_classic_state_get_connection(JOYCON_TYPE_LEFT);
            if (conn) {
                memcpy(conn->addr, addr, 6);
            }
        }
        if (load_address_from_nvs(JOYCON_TYPE_RIGHT, addr) == ESP_OK) {
            joycon_bt_classic_connection_t *conn = joycon_bt_classic_state_get_connection(JOYCON_TYPE_RIGHT);
            if (conn) {
                memcpy(conn->addr, addr, 6);
            }
        }
        xSemaphoreGive(connection_mutex);
    }
    
    bt_classic_initialized = true;
    bt_classic_enabled = true;
    xSemaphoreGive(bt_init_semaphore);
    
    ESP_LOGI(TAG, "Bluedroid stack initialized successfully");
    return ESP_OK;
}

/**
 * @brief Деинициализация Bluedroid стека
 */
esp_err_t joycon_bt_classic_deinit(void)
{
    if (!bt_classic_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing Bluedroid stack...");
    
    // Отключение всех подключений
    joycon_bt_classic_disconnect(JOYCON_TYPE_LEFT);
    joycon_bt_classic_disconnect(JOYCON_TYPE_RIGHT);
    
    // Деинициализация L2CAP модуля
    extern esp_err_t joycon_bt_classic_l2cap_deinit(void);
    joycon_bt_classic_l2cap_deinit();
    
    // Отключение Bluedroid (но не контроллера, так как он используется для BLE тоже)
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    // НЕ отключаем контроллер, так как он используется для BLE в BTDM режиме
    // Контроллер будет деинициализирован при деинициализации BLE стека
    
    // Освобождение мьютексов
    if (connection_mutex) {
        vSemaphoreDelete(connection_mutex);
        connection_mutex = NULL;
    }
    if (device_cache_mutex) {
        vSemaphoreDelete(device_cache_mutex);
        device_cache_mutex = NULL;
    }
    if (bt_init_semaphore) {
        vSemaphoreDelete(bt_init_semaphore);
        bt_init_semaphore = NULL;
    }
    
    bt_classic_initialized = false;
    bt_classic_enabled = false;
    
    ESP_LOGI(TAG, "Bluedroid stack deinitialized");
    return ESP_OK;
}

/**
 * @brief Получить состояние подключения
 */
joycon_state_t joycon_bt_classic_get_state(joycon_type_t type)
{
    extern joycon_state_t joycon_bt_classic_state_get(joycon_type_t type);
    return joycon_bt_classic_state_get(type);
}

/**
 * @brief Установить callback для получения данных
 */
void joycon_bt_classic_set_data_callback(joycon_bt_classic_data_callback_t callback)
{
    data_callback = callback;
}

/**
 * @brief Установить callback для изменения состояния
 */
void joycon_bt_classic_set_state_callback(joycon_bt_classic_state_callback_t callback)
{
    extern void joycon_bt_classic_state_subscribe(joycon_bt_classic_state_callback_t callback);
    joycon_bt_classic_state_subscribe(callback);
}

/**
 * @brief Получить MAC адрес подключенного Joy-Con
 */
esp_err_t joycon_bt_classic_get_address(joycon_type_t type, uint8_t *addr)
{
    if (!addr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    joycon_bt_classic_connection_t *conn = get_connection(type);
    if (!conn) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (connection_mutex && xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(addr, conn->addr, 6);
        xSemaphoreGive(connection_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Очистить сохраненные MAC адреса Joy-Con
 */
esp_err_t joycon_bt_classic_clear_saved_addresses(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("joycon_bt", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", err);
        return err;
    }
    
    nvs_erase_key(nvs_handle, "left_addr");
    nvs_erase_key(nvs_handle, "right_addr");
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    // Очистка в памяти
    if (connection_mutex && xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memset(left_joycon.addr, 0, 6);
        memset(right_joycon.addr, 0, 6);
        xSemaphoreGive(connection_mutex);
    }
    
    ESP_LOGI(TAG, "Cleared saved Joy-Con addresses");
    return ESP_OK;
}

/**
 * @brief Ожидать инициализации Bluedroid стека
 */
esp_err_t joycon_bt_classic_wait_for_init(uint32_t timeout_ms)
{
    if (!bt_init_semaphore) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (timeout_ms == 0) {
        // Ждать бесконечно
        xSemaphoreTake(bt_init_semaphore, portMAX_DELAY);
        return ESP_OK;
    } else {
        TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
        if (xSemaphoreTake(bt_init_semaphore, timeout_ticks) == pdTRUE) {
            return ESP_OK;
        } else {
            return ESP_ERR_TIMEOUT;
        }
    }
}

/**
 * @brief Сохранить адрес Joy-Con в NVS
 */
esp_err_t joycon_bt_classic_save_address(joycon_type_t type, const uint8_t *addr)
{
    return save_address_to_nvs(type, addr);
}

// Экспорт функций для использования в других модулях
joycon_bt_classic_connection_t* joycon_bt_classic_get_connection(joycon_type_t type)
{
    return get_connection(type);
}

SemaphoreHandle_t joycon_bt_classic_get_connection_mutex(void)
{
    return connection_mutex;
}

bool joycon_bt_classic_is_initialized(void)
{
    return bt_classic_initialized;
}

/**
 * @file joycon_bt_classic_connection.c
 * @brief Модуль управления подключениями Joy-Con через Bluetooth Classic
 * 
 * Содержит:
 * - Подключение и отключение от Joy-Con через Classic HID
 * - Обработку L2CAP каналов для HID данных
 * - Управление состоянием подключений
 * - Отправку команд вибрации
 */

#include "joycon_bt_classic_connection.h"
#include "joycon_bt_classic_internal.h"
#include "joycon_bt_classic_discovery.h"
#include "joycon_bt_classic_l2cap.h"
#include "../joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "JOYCON_BT_CLASSIC";

// Внешние объявления
extern joycon_bt_classic_data_callback_t data_callback;
extern bool bt_classic_initialized;

// PSM для HID каналов (стандартные значения)
#define HID_INTERRUPT_PSM 0x13
#define HID_CONTROL_PSM 0x11

// L2CAP callback удален - в Bluedroid L2CAP управляется через SDP/ACL автоматически

/**
 * @brief Подключиться к Joy-Con через Bluetooth Classic
 */
esp_err_t joycon_bt_classic_connect(joycon_type_t type, uint8_t *addr)
{
    if (!bt_classic_initialized) {
        ESP_LOGE(TAG, "Bluedroid stack not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (type != JOYCON_TYPE_LEFT && type != JOYCON_TYPE_RIGHT) {
        ESP_LOGE(TAG, "Invalid Joy-Con type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (addr != NULL && !is_valid_mac_address(addr)) {
        ESP_LOGE(TAG, "Invalid MAC address provided");
        return ESP_ERR_INVALID_ARG;
    }
    
    joycon_bt_classic_connection_t *conn = get_connection(type);
    if (!conn) {
        ESP_LOGE(TAG, "Failed to get connection structure for type %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    bool is_connected = false;
    uint8_t addr_copy[6] = {0};
    bool has_addr = false;
    
    if (connection_mutex == NULL) {
        ESP_LOGE(TAG, "Connection mutex not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        is_connected = conn->connected;
        if (!is_connected) {
            if (addr) {
                memcpy(conn->addr, addr, 6);
                memcpy(addr_copy, addr, 6);
                has_addr = true;
            } else {
                memcpy(addr_copy, conn->addr, 6);
                has_addr = !is_mac_addr_zero(conn->addr);
            }
        }
        xSemaphoreGive(connection_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take connection mutex for joycon_bt_classic_connect");
        return ESP_ERR_TIMEOUT;
    }
    
    if (is_connected) {
        return ESP_OK;
    }
    
    if (!has_addr) {
        ESP_LOGW(TAG, "No address provided for Joy-Con %s, starting scan",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        extern esp_err_t joycon_bt_classic_start_scan(void);
        return joycon_bt_classic_start_scan();
    }
    
    // Подключение через ACL
    // Устанавливаем режим сканирования
    esp_err_t ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set scan mode: %d", ret);
    }
    
    // Устанавливаем PSM для HID каналов
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        conn->hid_interrupt_psm = HID_INTERRUPT_PSM;
        conn->hid_control_psm = HID_CONTROL_PSM;
        xSemaphoreGive(connection_mutex);
    }
    
    // Сохраняем время начала подключения для таймаута
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        conn->connecting_start_tick = xTaskGetTickCount();
        xSemaphoreGive(connection_mutex);
    }
    
    update_state(type, JOYCON_STATE_CONNECTING);
    ESP_LOGI(TAG, "Connecting to Joy-Con %s at %02x:%02x:%02x:%02x:%02x:%02x",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right",
             MAC_STR(addr_copy));
    
    // Явно инициируем ACL соединение через SDP запрос
    // esp_bt_gap_get_remote_services() устанавливает ACL соединение в ESP-IDF
    // Это необходимо для последующей установки L2CAP каналов
    ret = esp_bt_gap_get_remote_services(addr_copy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to request remote services (SDP) for ACL connection: %d", ret);
        // Не возвращаем ошибку - SDP мог быть уже запрошен ранее
        // ACL соединение может установиться при обработке предыдущих SDP запросов
    } else {
        ESP_LOGI(TAG, "SDP request sent for ACL connection to Joy-Con %s",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    }
    
    // Для Joy-Con Classic HID нужно будет установить L2CAP каналы после ACL соединения
    // ACL соединение будет установлено при получении события ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT
    // которое обработается в joycon_bt_classic_scan.c и вызовет joycon_bt_classic_handle_acl_connected()
    
    return ESP_OK;
}

/**
 * @brief Задача для перезапуска сканирования после подключения первого джойкона
 */
static void scan_restart_task(void *pvParameters)
{
    // Задержка для установки L2CAP каналов первого джойкона
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    extern esp_err_t joycon_bt_classic_start_scan(void);
    esp_err_t ret = joycon_bt_classic_start_scan();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restart scan after first Joy-Con connected: %d", ret);
    } else {
        ESP_LOGI(TAG, "Scan restarted to find second Joy-Con");
    }
    vTaskDelete(NULL);
}

/**
 * @brief Задача для отложенного открытия L2CAP каналов после ACL подключения
 */
static void l2cap_open_delayed_task(void *pvParameters)
{
    joycon_type_t type = (joycon_type_t)(uintptr_t)pvParameters;
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    
    ESP_LOGI(TAG, "[L2CAP] Starting delayed L2CAP open task for %s Joy-Con, waiting 300ms after ACL", type_str);
    
    // ВАЖНО: Увеличиваем задержку для полного установления ACL соединения
    // и готовности L2CAP/VFS стека к регистрации файловых дескрипторов.
    // Ошибка "l2cap_malloc_slot unable to register fd" часто возникает
    // из-за недостаточной задержки - стек еще не готов к открытию каналов.
    // В примерах ESP-IDF обычно используют 200-500мс после ACL.
    vTaskDelay(pdMS_TO_TICKS(300));
    
    ESP_LOGI(TAG, "[L2CAP] Delay completed, calling joycon_bt_classic_open_l2cap_channels for %s Joy-Con", type_str);
    
    esp_err_t l2cap_ret = joycon_bt_classic_open_l2cap_channels(type);
    if (l2cap_ret != ESP_OK) {
        ESP_LOGW(TAG, "[L2CAP] Failed to open L2CAP channels for %s Joy-Con: %d (0x%x)",
                 type_str, l2cap_ret, l2cap_ret);
    } else {
        ESP_LOGI(TAG, "[L2CAP] L2CAP channels open request sent successfully for %s Joy-Con", type_str);
    }
    vTaskDelete(NULL);
}

/**
 * @brief Обработка события подключения ACL
 */
void joycon_bt_classic_handle_acl_connected(uint8_t *bda)
{
    if (!bda) {
        return;
    }
    
    joycon_bt_classic_connection_t *conn = NULL;
    joycon_type_t conn_type = JOYCON_TYPE_UNKNOWN;
    
    if (connection_mutex == NULL) {
        ESP_LOGE(TAG, "Connection mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (memcmp(bda, left_joycon.addr, 6) == 0) {
            conn = &left_joycon;
            conn_type = JOYCON_TYPE_LEFT;
        } else if (memcmp(bda, right_joycon.addr, 6) == 0) {
            conn = &right_joycon;
            conn_type = JOYCON_TYPE_RIGHT;
        }
        
        xSemaphoreGive(connection_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take connection mutex for ACL connected");
        return;
    }
    
    if (conn) {
        // update_state синхронизирует connected и connecting с state
        update_state(conn_type, JOYCON_STATE_CONNECTED);
        ESP_LOGI(TAG, "Joy-Con %s ACL connected", 
                 conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        
        // Проверяем, подключен ли второй джойкон, и если нет - перезапускаем сканирование
        bool both_connected = false;
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            both_connected = left_joycon.connected && right_joycon.connected;
            xSemaphoreGive(connection_mutex);
        }
        
        if (!both_connected) {
            // Второй джойкон еще не подключен - перезапускаем сканирование после задержки
            // Задержка нужна, чтобы не прерывать установку L2CAP каналов
            // Используем задачу, чтобы не блокировать callback
            BaseType_t scan_task_created = xTaskCreate(
                scan_restart_task,
                "restart_scan",
                2048,
                NULL,
                3,
                NULL
            );
            if (scan_task_created != pdPASS) {
                ESP_LOGW(TAG, "Failed to create scan restart task");
            }
        }
        
        // Открываем L2CAP каналы для HID протокола с задержкой после ACL подключения
        // Это позволяет ACL соединению полностью установиться перед открытием L2CAP каналов
        // Задержка предотвращает ошибку "l2cap_malloc_slot unable to register fd"
        // Создаем задачу с задержкой для открытия L2CAP каналов, чтобы не блокировать callback
        BaseType_t task_created = xTaskCreate(
            l2cap_open_delayed_task,
            "l2cap_open",
            3072,
            (void*)(uintptr_t)conn_type,
            5,
            NULL
        );
        
        if (task_created == pdPASS) {
            ESP_LOGI(TAG, "L2CAP channels opening initiated for %s Joy-Con (delayed 300ms after ACL)",
                     conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        } else {
            ESP_LOGE(TAG, "Failed to create L2CAP open task for %s Joy-Con",
                     conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        }
    }
}

/**
 * @brief Обработка события отключения ACL
 */
void joycon_bt_classic_handle_acl_disconnected(uint8_t *bda)
{
    if (!bda) {
        return;
    }
    
    joycon_bt_classic_connection_t *conn = NULL;
    joycon_type_t conn_type = JOYCON_TYPE_UNKNOWN;
    
    if (connection_mutex == NULL) {
        ESP_LOGE(TAG, "Connection mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (memcmp(bda, left_joycon.addr, 6) == 0) {
            conn = &left_joycon;
            conn_type = JOYCON_TYPE_LEFT;
        } else if (memcmp(bda, right_joycon.addr, 6) == 0) {
            conn = &right_joycon;
            conn_type = JOYCON_TYPE_RIGHT;
        }
        
        if (conn) {
            conn->sdp_complete = false;
            conn->hid_psm = 0;
            conn->hid_interrupt_psm = 0;
            conn->hid_control_psm = 0;
        }
        xSemaphoreGive(connection_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take connection mutex for ACL disconnected");
        return;
    }
    
    if (conn) {
        // Закрываем L2CAP каналы при отключении ACL
        joycon_bt_classic_close_l2cap_channels(conn_type);
        
        // update_state синхронизирует connected и connecting с state
        update_state(conn_type, JOYCON_STATE_DISCONNECTED);
        ESP_LOGI(TAG, "Joy-Con %s ACL disconnected", 
                 conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    }
}

/**
 * @brief Отключиться от Joy-Con
 */
esp_err_t joycon_bt_classic_disconnect(joycon_type_t type)
{
    if (type != JOYCON_TYPE_LEFT && type != JOYCON_TYPE_RIGHT) {
        ESP_LOGE(TAG, "Invalid Joy-Con type: %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    joycon_bt_classic_connection_t *conn = get_connection(type);
    if (!conn) {
        ESP_LOGE(TAG, "Failed to get connection structure for type %d", type);
        return ESP_ERR_INVALID_ARG;
    }
    
    bool is_connected = false;
    uint8_t addr[6] = {0};
    
    if (connection_mutex == NULL) {
        ESP_LOGE(TAG, "Connection mutex not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        is_connected = conn->connected;
        if (is_connected) {
            memcpy(addr, conn->addr, 6);
        }
        xSemaphoreGive(connection_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take connection mutex for joycon_bt_classic_disconnect");
        return ESP_ERR_TIMEOUT;
    }
    
    if (!is_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Закрываем L2CAP каналы перед отключением ACL
    joycon_bt_classic_close_l2cap_channels(type);
    
    ESP_LOGI(TAG, "Disconnecting Joy-Con %s (L2CAP channels closed)",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    
    // ACL соединение закрывается автоматически при закрытии всех L2CAP каналов
    // или при отключении устройства. Явное закрытие через esp_gap_bt_api не требуется.
    // Обработка отключения ACL выполняется в joycon_bt_classic_handle_acl_disconnected()
    
    return ESP_OK;
}

/**
 * @brief Проверить таймауты подключения и сбросить зависшие подключения
 */
void joycon_bt_classic_check_connection_timeouts(void)
{
    if (connection_mutex == NULL) {
        return;
    }
    
    uint32_t current_tick = xTaskGetTickCount();
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Проверяем левый Joy-Con
        if (left_joycon.connecting && !left_joycon.connected) {
            uint32_t elapsed_ms = (current_tick - left_joycon.connecting_start_tick) * portTICK_PERIOD_MS;
            if (elapsed_ms >= BT_CLASSIC_CONNECTION_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Connection timeout for Left Joy-Con (%lu ms), resetting state", elapsed_ms);
                left_joycon.connecting = false;
                left_joycon.connecting_start_tick = 0;
                xSemaphoreGive(connection_mutex);
                update_state(JOYCON_TYPE_LEFT, JOYCON_STATE_DISCONNECTED);
                return;
            }
        }
        
        // Проверяем правый Joy-Con
        if (right_joycon.connecting && !right_joycon.connected) {
            uint32_t elapsed_ms = (current_tick - right_joycon.connecting_start_tick) * portTICK_PERIOD_MS;
            if (elapsed_ms >= BT_CLASSIC_CONNECTION_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Connection timeout for Right Joy-Con (%lu ms), resetting state", elapsed_ms);
                right_joycon.connecting = false;
                right_joycon.connecting_start_tick = 0;
                xSemaphoreGive(connection_mutex);
                update_state(JOYCON_TYPE_RIGHT, JOYCON_STATE_DISCONNECTED);
                return;
            }
        }
        
        xSemaphoreGive(connection_mutex);
    }
}

/**
 * @brief Отправить команду вибрации Joy-Con
 */
esp_err_t joycon_bt_classic_send_vibration(joycon_type_t type, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    joycon_bt_classic_connection_t *conn = get_connection(type);
    if (!conn) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (connection_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    bool is_connected = false;
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        is_connected = conn->connected;
        xSemaphoreGive(connection_mutex);
    }
    
    if (!is_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Отправка данных через HID Control канал L2CAP
    // Формируем команду вибрации в формате Output Report
    // Format: [0x01, 0x00, 0x01, 0x10, ...rumble_data]
    uint8_t output_report[64] = {0};
    output_report[0] = 0x01; // Output Report ID
    output_report[1] = 0x00; // Rumble/Timestamp байт 1
    output_report[2] = 0x01; // Rumble/Timestamp байт 2
    output_report[3] = 0x10; // Subcommand ID для HD Rumble
    
    // Копируем данные вибрации
    size_t copy_len = (len < sizeof(output_report) - 4) ? len : sizeof(output_report) - 4;
    if (copy_len > 0) {
        memcpy(&output_report[4], data, copy_len);
    }
    
    size_t report_len = 4 + copy_len;
    
    esp_err_t ret = joycon_bt_classic_send_output_report(type, output_report, report_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send vibration command: %d", ret);
        return ret;
    }
    
    ESP_LOGD(TAG, "Vibration command sent to %s Joy-Con",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    
    return ESP_OK;
}

/**
 * @brief Проверка условий для попытки подключения к Joy-Con
 */
bool joycon_bt_classic_connection_should_attempt(const discovered_bt_classic_device_t *cached_device,
                                                 bool is_left, bool is_right)
{
    if (!cached_device) {
        return false;
    }
    
    // Ужесточенная проверка: подключаемся только к устройствам, которые точно являются Joy-Con
    // Требуем либо явное определение типа по имени (is_left/is_right),
    // либо комбинацию MAC-адреса Joy-Con И HID сервиса И имени с "joy-con" (проверка без учета регистра)
    
    bool has_hid_service = cached_device->has_hid_service;
    
    bool mac_looks_like_joycon = (cached_device->addr[5] == 0x00 && cached_device->addr[4] == 0x9f) ||
                                (cached_device->addr[5] == 0x98 && cached_device->addr[4] == 0xb6);
    
    // Проверяем имя на наличие "joy-con" (без учета регистра - имя приводится к нижнему регистру)
    bool name_contains_joycon = false;
    if (cached_device->has_name && strlen(cached_device->name) > 0) {
        char name_lower[32];
        size_t name_len = strlen(cached_device->name);
        size_t copy_len = (name_len < sizeof(name_lower) - 1) ? name_len : sizeof(name_lower) - 1;
        memcpy(name_lower, cached_device->name, copy_len);
        name_lower[copy_len] = '\0';
        // Приводим к нижнему регистру для проверки
        for (size_t i = 0; i < copy_len; i++) {
            if (name_lower[i] >= 'A' && name_lower[i] <= 'Z') {
                name_lower[i] = name_lower[i] - 'A' + 'a';
            }
        }
        name_contains_joycon = (strstr(name_lower, "joy-con") != NULL);
    }
    
    // Подключаемся только если:
    // 1. Устройство определено как Joy-Con по имени (is_left или is_right), ИЛИ
    // 2. (MAC похож на Joy-Con И есть HID сервис И имя содержит "joy-con" без учета регистра)
    bool should_connect = (is_left || is_right) || 
                         (mac_looks_like_joycon && has_hid_service && name_contains_joycon);
    
    return should_connect;
}

/**
 * @brief Проверка, найдены ли оба Joy-Con (внутренняя функция)
 */
static bool both_joycons_found_internal(void)
{
    extern bool joycon_bt_classic_discovery_is_left_discovered_in_scan(void);
    extern bool joycon_bt_classic_discovery_is_right_discovered_in_scan(void);
    
    bool left_discovered = joycon_bt_classic_discovery_is_left_discovered_in_scan();
    bool right_discovered = joycon_bt_classic_discovery_is_right_discovered_in_scan();
    
    bool left_has_address = false;
    bool right_has_address = false;
    bool left_connected_or_connecting = false;
    bool right_connected_or_connecting = false;
    
    if (connection_mutex != NULL) {
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            left_has_address = !is_mac_addr_zero(left_joycon.addr);
            right_has_address = !is_mac_addr_zero(right_joycon.addr);
            left_connected_or_connecting = left_joycon.connected || left_joycon.connecting;
            right_connected_or_connecting = right_joycon.connected || right_joycon.connecting;
            xSemaphoreGive(connection_mutex);
        }
    }
    
    // Оба найдены в текущем сканировании
    if (left_discovered && right_discovered) {
        return true;
    }
    
    // Один подключен/подключается с адресом, другой найден в текущем сканировании
    if (left_connected_or_connecting && left_has_address && right_discovered) {
        return true;
    }
    if (right_connected_or_connecting && right_has_address && left_discovered) {
        return true;
    }
    
    // Оба подключены/подключаются с адресами (но не найдены в текущем сканировании - переподключение)
    if (left_connected_or_connecting && left_has_address && 
        right_connected_or_connecting && right_has_address) {
        return true;
    }
    
    return false;
}

/**
 * @brief Попытка подключения к обнаруженному Joy-Con
 */
int joycon_bt_classic_connection_attempt(const uint8_t *addr,
                                         const discovered_bt_classic_device_t *cached_device,
                                         bool is_left, bool is_right)
{
    if (!addr || !cached_device) {
        return -1;
    }
    
    joycon_type_t type;
    
    if (is_left) {
        type = JOYCON_TYPE_LEFT;
    } else if (is_right) {
        type = JOYCON_TYPE_RIGHT;
    } else {
        // Определяем по доступным слотам
        bool left_free = false;
        bool right_free = false;
        
        if (connection_mutex == NULL) {
            ESP_LOGE(TAG, "Connection mutex not initialized");
            return -1;
        }
        
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            left_free = !left_joycon.connected && is_mac_addr_zero(left_joycon.addr);
            right_free = !right_joycon.connected && is_mac_addr_zero(right_joycon.addr);
            xSemaphoreGive(connection_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take connection mutex for type detection");
            return -1;
        }
        
        if (left_free) {
            type = JOYCON_TYPE_LEFT;
            ESP_LOGI(TAG, "HID device without name detected, assigning to Left slot");
        } else if (right_free) {
            type = JOYCON_TYPE_RIGHT;
            ESP_LOGI(TAG, "HID device without name detected, assigning to Right slot");
        } else {
            ESP_LOGW(TAG, "HID device detected but both slots are in use, skipping");
            return -1;
        }
    }
    
    uint8_t addr_copy[6];
    memcpy(addr_copy, addr, 6);
    bool is_connected = false;
    bool is_connecting = false;
    bool slot_reserved = false;
    
    if (connection_mutex == NULL) {
        ESP_LOGE(TAG, "Connection mutex not initialized");
        return -1;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection(type);
        if (conn) {
            is_connected = conn->connected;
            is_connecting = conn->connecting;
            if (!is_connected && !is_connecting) {
                conn->connecting = true;
                memcpy(conn->addr, addr_copy, 6);
                slot_reserved = true;
            }
        }
        xSemaphoreGive(connection_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take connection mutex for connection attempt");
        return -1;
    }
    
    if (slot_reserved) {
        const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
        bool has_name = cached_device->has_name;
        bool has_hid_service = cached_device->has_hid_service;
        
        // Логируем только если это первый раз (не через callback из discovery)
        ESP_LOGI(TAG, "Found Joy-Con %s at %02x:%02x:%02x:%02x:%02x:%02x (name='%s', HID=%s)",
                 type_str,
                 MAC_STR(addr_copy),
                 has_name ? cached_device->name : "no name",
                 has_hid_service ? "yes" : "no");
        
        // Помечаем джойкон как обнаруженный в текущем сканировании через discovery модуль
        extern void joycon_bt_classic_discovery_set_left_discovered(bool discovered);
        extern void joycon_bt_classic_discovery_set_right_discovered(bool discovered);
        if (type == JOYCON_TYPE_LEFT) {
            joycon_bt_classic_discovery_set_left_discovered(true);
        } else {
            joycon_bt_classic_discovery_set_right_discovered(true);
        }
        
        // Сохраняем в NVS
        extern esp_err_t joycon_bt_classic_save_address(joycon_type_t type, uint8_t *addr);
        joycon_bt_classic_save_address(type, addr_copy);
        
        // Останавливаем discovery перед началом подключения, чтобы освободить ресурсы
        // и позволить ACL соединению установиться. Discovery может помешать установке ACL.
        esp_bt_gap_cancel_discovery();
        
        // Проверяем, найдены ли оба джойкона в текущем сканировании
        if (both_joycons_found_internal()) {
            ESP_LOGI(TAG, "Both Joy-Con found, stopping scan before connection");
        } else {
            ESP_LOGI(TAG, "Stopping discovery to allow connection, will resume scan later if needed");
        }
        
        esp_err_t ret = joycon_bt_classic_connect(type, addr_copy);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to Joy-Con %s: %d", type_str, ret);
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                joycon_bt_classic_connection_t *conn = get_connection(type);
                if (conn) {
                    conn->connecting = false;
                    memset(conn->addr, 0, 6);
                }
                xSemaphoreGive(connection_mutex);
            }
            return -1;
        }
    }
    
    return 0;
}

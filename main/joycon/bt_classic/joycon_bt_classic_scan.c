/**
 * @file joycon_bt_classic_scan.c
 * @brief Модуль сканирования Bluetooth Classic устройств
 * 
 * Содержит:
 * - GAP inquiry для обнаружения устройств
 * - SDP запросы для проверки HID сервиса
 * - Кэширование обнаруженных устройств
 * - Определение типа Joy-Con
 * - Автоматическое подключение к Joy-Con
 */

#include "joycon_bt_classic_internal.h"
#include "joycon_bt_classic_discovery.h"
#include "joycon_bt_classic_connection.h"
#include "../joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
// SDP API интегрирован в GAP API в Bluedroid
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "JOYCON_BT_CLASSIC";

// Внешние объявления
extern bool bt_classic_initialized;

// Внешние объявления
extern esp_err_t joycon_bt_classic_connect(joycon_type_t type, uint8_t *addr);
extern esp_err_t joycon_bt_classic_save_address(joycon_type_t type, const uint8_t *addr);
extern joycon_bt_classic_connection_t left_joycon;
extern joycon_bt_classic_connection_t right_joycon;
extern SemaphoreHandle_t connection_mutex;

/**
 * @brief Проверить, найдены ли оба Joy-Con (в текущем сканировании или уже подключены)
 * 
 * @return true если оба джойкона найдены:
 *   - Оба найдены в текущем сканировании, ИЛИ
 *   - Один подключен/подключается с адресом, а второй найден в текущем сканировании, ИЛИ
 *   - Оба подключены/подключаются с адресами
 * 
 * Примечание: Проверяет комбинацию обнаружения в текущем сканировании и состояния подключения.
 * Это позволяет остановить сканирование, если:
 * - Оба джойкона найдены в текущем сканировании (обычный случай)
 * - Один уже подключен, а второй найден (переподключение)
 * Но продолжает сканирование, если джойконы только загружены из NVS, но не обнаружены и не подключены.
 */
static bool both_joycons_found(void)
{
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
    
    // Получаем флаги обнаружения из discovery модуля
    extern bool joycon_bt_classic_discovery_is_left_discovered_in_scan(void);
    extern bool joycon_bt_classic_discovery_is_right_discovered_in_scan(void);
    bool left_discovered = joycon_bt_classic_discovery_is_left_discovered_in_scan();
    bool right_discovered = joycon_bt_classic_discovery_is_right_discovered_in_scan();
    
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
 * @brief Найти или создать запись в кэше обнаруженных устройств
 * 
 * @deprecated Используйте joycon_bt_classic_discovery_get_device() вместо этого
 * Оставлена для обратной совместимости в процессе миграции
 */
__attribute__((unused))
static discovered_bt_classic_device_t* find_or_create_device_cache(const uint8_t *addr)
{
    // Делегируем работу discovery модулю
    return joycon_bt_classic_discovery_get_device(addr);
}

/**
 * @brief Определение типа Joy-Con по имени и MAC-адресу
 * 
 * @deprecated Используйте joycon_bt_classic_discovery_identify_joycon_type() вместо этого
 * Оставлена для обратной совместимости в процессе миграции
 */
__attribute__((unused))
static void identify_joycon_type(discovered_bt_classic_device_t *cached_device,
                                 const uint8_t *mac_addr,
                                 bool *is_left, bool *is_right)
{
    // Делегируем работу discovery модулю
    joycon_bt_classic_discovery_identify_joycon_type(cached_device, mac_addr, is_left, is_right);
}


// Callback для обработки найденных устройств (вызывается из discovery модуля)
static void on_device_found_callback(const discovered_bt_classic_device_t *device, bool is_left, bool is_right)
{
    if (!device) {
        return;
    }
    
    // Проверяем, нужно ли подключаться
    if (joycon_bt_classic_connection_should_attempt(device, is_left, is_right)) {
        // Проверяем, подключены ли оба джойкона перед остановкой discovery
        bool both_connected = false;
        if (connection_mutex != NULL) {
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                both_connected = left_joycon.connected && right_joycon.connected;
                xSemaphoreGive(connection_mutex);
            }
        }
        
        // Останавливаем discovery только если оба джойкона уже подключены
        // Или если это точно тот джойкон, который мы ищем (по сохраненному адресу)
        // Это позволяет продолжать поиск второго джойкона после подключения первого
        if (both_connected) {
            extern esp_err_t joycon_bt_classic_discovery_stop(void);
            joycon_bt_classic_discovery_stop();
            ESP_LOGI(TAG, "Both Joy-Con connected, stopping discovery");
        } else {
            // Не останавливаем discovery полностью - продолжаем поиск второго джойкона
            // Но временно останавливаем, чтобы не мешать текущему подключению
            extern esp_err_t joycon_bt_classic_discovery_stop(void);
            joycon_bt_classic_discovery_stop();
            ESP_LOGI(TAG, "Temporarily stopping discovery for connection, will resume for second Joy-Con");
        }
        
        // Делегируем работу connection модулю
        joycon_bt_classic_connection_attempt(device->addr, device, is_left, is_right);
    }
}

// Callback для обработки завершения discovery
static void on_discovery_complete_callback(void)
{
    // Проверяем, идет ли процесс подключения к какому-либо джойкону
    // Если идет подключение, НЕ перезапускаем сканирование, чтобы не прервать его
    bool is_connecting = false;
    if (connection_mutex != NULL) {
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            is_connecting = left_joycon.connecting || right_joycon.connecting;
            xSemaphoreGive(connection_mutex);
        }
    }
    
    if (is_connecting) {
        ESP_LOGI(TAG, "Joy-Con connection in progress, not restarting scan to avoid interruption");
    } else {
        // Проверяем, найдены ли оба джойкона
        bool both_found = both_joycons_found();
        
        // Проверяем количество активных подключений
        bool has_active_connections = false;
        bool both_connected = false;
        if (connection_mutex != NULL) {
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                has_active_connections = left_joycon.connected || right_joycon.connected;
                both_connected = left_joycon.connected && right_joycon.connected;
                xSemaphoreGive(connection_mutex);
            }
        }
        
        if (both_found || both_connected) {
            ESP_LOGI(TAG, "Both Joy-Con found/connected, scanning complete");
        } else {
            // Не все джойконы найдены/подключены - перезапускаем сканирование
            // даже если есть активные подключения, чтобы найти второй джойкон
            ESP_LOGI(TAG, "Not all Joy-Con found yet (active connections: %s), restarting scan...",
                     has_active_connections ? "yes" : "no");
            // Небольшая задержка перед перезапуском сканирования
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_err_t ret = joycon_bt_classic_start_scan();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to restart scan: %d", ret);
            }
        }
    }
}

/**
 * @brief Обработчик событий GAP
 * 
 * Маршрутизирует события между модулями:
 * - Discovery события → discovery модуль
 * - ACL события → connection модуль (через функции, объявленные в connection.c)
 * - Аутентификация → обработка здесь
 */
void joycon_bt_classic_handle_gap_event(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    // События discovery маршрутизируем в discovery модуль
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
            joycon_bt_classic_discovery_handle_gap_event(event, param);
            return; // Discovery модуль обработал событие
        }
        
        default:
            // Продолжаем обработку других событий
            break;
    }
    
    // Обрабатываем остальные события (ACL, аутентификация и т.д.)
    switch (event) {
        
        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: {
            // ACL соединение завершено
            extern void joycon_bt_classic_handle_acl_connected(uint8_t *bda);
            if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "ACL connection completed for %02x:%02x:%02x:%02x:%02x:%02x",
                         MAC_STR(param->acl_conn_cmpl_stat.bda));
                joycon_bt_classic_handle_acl_connected(param->acl_conn_cmpl_stat.bda);
            } else {
                ESP_LOGW(TAG, "ACL connection failed for %02x:%02x:%02x:%02x:%02x:%02x: status=%d",
                         MAC_STR(param->acl_conn_cmpl_stat.bda), param->acl_conn_cmpl_stat.stat);
            }
            break;
        }
        
        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
            // ACL соединение разорвано
            extern void joycon_bt_classic_handle_acl_disconnected(uint8_t *bda);
            ESP_LOGI(TAG, "ACL disconnected for %02x:%02x:%02x:%02x:%02x:%02x",
                     MAC_STR(param->acl_disconn_cmpl_stat.bda));
            joycon_bt_classic_handle_acl_disconnected(param->acl_disconn_cmpl_stat.bda);
            break;
        }
        
        case ESP_BT_GAP_PIN_REQ_EVT: {
            // Запрос PIN кода для аутентификации
            // Joy-Con обычно не требуют PIN, но обрабатываем событие
            ESP_LOGI(TAG, "PIN request for %02x:%02x:%02x:%02x:%02x:%02x",
                     MAC_STR(param->pin_req.bda));
            // Для Joy-Con обычно используется Just-Works аутентификация (без PIN)
            // Отвечаем отказом на PIN, если требуется
            esp_bt_pin_code_t pin_code = {0};
            esp_err_t ret = esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to reply to PIN request: %d", ret);
            }
            break;
        }
        
        case ESP_BT_GAP_CFM_REQ_EVT: {
            // Запрос подтверждения для SSP (Secure Simple Pairing)
            ESP_LOGI(TAG, "SSP confirmation request for %02x:%02x:%02x:%02x:%02x:%02x (num_val=%d)",
                     MAC_STR(param->cfm_req.bda), param->cfm_req.num_val);
            // Для Joy-Con обычно используем Just-Works (автоматическое подтверждение)
            esp_err_t ret = esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to reply to SSP confirmation: %d", ret);
            }
            break;
        }
        
        case ESP_BT_GAP_KEY_NOTIF_EVT: {
            // Уведомление о ключе для аутентификации
            ESP_LOGD(TAG, "Key notification for %02x:%02x:%02x:%02x:%02x:%02x",
                     MAC_STR(param->key_notif.bda));
            break;
        }
        
        case ESP_BT_GAP_KEY_REQ_EVT: {
            // Запрос ключа для аутентификации
            ESP_LOGD(TAG, "Key request for %02x:%02x:%02x:%02x:%02x:%02x",
                     MAC_STR(param->key_req.bda));
            break;
        }
        
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            // Аутентификация завершена
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication completed successfully for %02x:%02x:%02x:%02x:%02x:%02x",
                         MAC_STR(param->auth_cmpl.bda));
            } else {
                ESP_LOGW(TAG, "Authentication failed for %02x:%02x:%02x:%02x:%02x:%02x: status=%d",
                         MAC_STR(param->auth_cmpl.bda), param->auth_cmpl.stat);
            }
            break;
        }
        
        default:
            break;
    }
}

// Флаг подписки на callbacks (инициализируется один раз)
static bool discovery_callbacks_subscribed = false;

/**
 * @brief Начать GAP inquiry для сканирования Joy-Con устройств
 */
esp_err_t joycon_bt_classic_start_scan(void)
{
    if (!bt_classic_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Подписываемся на callbacks discovery модуля (один раз)
    if (!discovery_callbacks_subscribed) {
        joycon_bt_classic_discovery_callbacks_t callbacks = {
            .on_device_found = on_device_found_callback,
            .on_discovery_complete = on_discovery_complete_callback,
            .on_discovery_started = NULL
        };
        joycon_bt_classic_discovery_subscribe(&callbacks);
        discovery_callbacks_subscribed = true;
        ESP_LOGI(TAG, "Subscribed to discovery callbacks");
    }
    
    ESP_LOGI(TAG, "Starting Bluetooth Classic scan (GAP inquiry)...");
    
    // Используем discovery модуль для запуска сканирования
    esp_err_t ret = joycon_bt_classic_discovery_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start discovery: %d", ret);
        return ret;
    }
    
    // Обновляем состояние на SCANNING только для джойконов, которые не подключены
    // и не находятся в процессе подключения. Это предотвращает прерывание активных подключений.
    if (connection_mutex != NULL) {
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            // Проверяем левый джойкон
            if (!left_joycon.connected && !left_joycon.connecting) {
                xSemaphoreGive(connection_mutex);
                update_state(JOYCON_TYPE_LEFT, JOYCON_STATE_SCANNING);
            } else {
                xSemaphoreGive(connection_mutex);
            }
            
            // Проверяем правый джойкон
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                if (!right_joycon.connected && !right_joycon.connecting) {
                    xSemaphoreGive(connection_mutex);
                    update_state(JOYCON_TYPE_RIGHT, JOYCON_STATE_SCANNING);
                } else {
                    xSemaphoreGive(connection_mutex);
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "Bluetooth Classic scan started");
    return ESP_OK;
}

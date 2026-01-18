/**
 * @file joycon_bt_classic_discovery.c
 * @brief Реализация модуля обнаружения Bluetooth Classic устройств
 * 
 * Модуль отвечает только за обнаружение и идентификацию устройств:
 * - GAP inquiry (сканирование)
 * - Кэширование обнаруженных устройств
 * - Определение типа Joy-Con
 * 
 * ВНИМАНИЕ: Подключение к устройствам НЕ входит в ответственность этого модуля.
 */

#include "joycon_bt_classic_discovery.h"
#include "joycon_bt_classic_internal.h"
#include "../joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "JOYCON_BT_CLASSIC_DISCOVERY";

// Кэш обнаруженных устройств (инкапсулирован в модуле)
static discovered_bt_classic_device_t device_cache[MAX_DISCOVERED_BT_CLASSIC_DEVICES];

// Callbacks для уведомлений о найденных устройствах
static joycon_bt_classic_discovery_callbacks_t discovery_callbacks = {0};

// Флаги обнаружения джойконов в текущем сканировании
// Используются для проверки, что джойконы реально найдены, а не только загружены из NVS
static bool left_discovered_in_scan = false;
static bool right_discovered_in_scan = false;

/**
 * @brief Проверить, соответствует ли MAC-адрес сохраненным джойконам
 * 
 * @param addr MAC-адрес для проверки
 * @param is_left_out Указатель для вывода, является ли это левым джойконом
 * @param is_right_out Указатель для вывода, является ли это правым джойконом
 * @return true если адрес соответствует сохраненному джойкону
 */
static bool is_saved_joycon_address(const uint8_t *addr, bool *is_left_out, bool *is_right_out)
{
    if (!addr || !is_left_out || !is_right_out) {
        return false;
    }
    
    *is_left_out = false;
    *is_right_out = false;
    
    extern SemaphoreHandle_t connection_mutex;
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
    
    if (connection_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }
    
    joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
    joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
    
    bool is_left = (left && !is_mac_addr_zero(left->addr) && memcmp(addr, left->addr, 6) == 0);
    bool is_right = (right && !is_mac_addr_zero(right->addr) && memcmp(addr, right->addr, 6) == 0);
    
    xSemaphoreGive(connection_mutex);
    
    *is_left_out = is_left;
    *is_right_out = is_right;
    
    return is_left || is_right;
}

/**
 * @brief Проверить, есть ли сохраненные MAC-адреса джойконов
 * 
 * @return true если есть хотя бы один сохраненный адрес
 */
static bool has_saved_joycon_addresses(void)
{
    extern SemaphoreHandle_t connection_mutex;
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
    
    if (connection_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }
    
    joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
    joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
    
    bool has_left = (left && !is_mac_addr_zero(left->addr));
    bool has_right = (right && !is_mac_addr_zero(right->addr));
    
    xSemaphoreGive(connection_mutex);
    
    return has_left || has_right;
}

/**
 * @brief Проверить, похоже ли устройство на Joy-Con по признакам (MAC OUI или имя)
 * 
 * @param cached_device Обнаруженное устройство
 * @param addr MAC-адрес устройства
 * @return true если устройство похоже на Joy-Con
 */
__attribute__((unused))
static bool looks_like_joycon(const discovered_bt_classic_device_t *cached_device, const uint8_t *addr)
{
    if (!addr) {
        return false;
    }
    
    // Проверяем по MAC адресу (Joy-Con имеют характерные OUI)
    // Nintendo OUI: 00:09:9F или 98:B6:XX
    bool mac_looks_like_joycon = (addr[5] == JOYCON_OUI_1_BYTE_5 && addr[4] == JOYCON_OUI_1_BYTE_4) ||
                                 (addr[5] == JOYCON_OUI_2_BYTE_5 && addr[4] == JOYCON_OUI_2_BYTE_4);
    
    // Проверяем по имени (если получено)
    bool name_looks_like_joycon = false;
    if (cached_device && cached_device->has_name && strlen(cached_device->name) > 0) {
        char name_lower[BT_DEVICE_NAME_BUFFER_SIZE];
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
        name_looks_like_joycon = (strstr(name_lower, "joy-con") != NULL);
    }
    
    return mac_looks_like_joycon || name_looks_like_joycon;
}

/**
 * @brief Найти или создать запись в кэше обнаруженных устройств
 */
static discovered_bt_classic_device_t* find_or_create_device_cache_internal(const uint8_t *addr)
{
    if (!addr || device_cache_mutex == NULL) {
        return NULL;
    }
    
    uint32_t current_tick = xTaskGetTickCount();
    discovered_bt_classic_device_t *empty_slot = NULL;
    discovered_bt_classic_device_t *oldest_slot = NULL;
    uint32_t oldest_tick = UINT32_MAX;
    
    if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take device cache mutex");
        return NULL;
    }
    
    // Ищем существующую запись или свободный слот
    for (int i = 0; i < ARRAY_SIZE(device_cache); i++) {
        if (memcmp(device_cache[i].addr, addr, 6) == 0 && 
            !is_mac_addr_zero(device_cache[i].addr)) {
            device_cache[i].last_seen_tick = current_tick;
            xSemaphoreGive(device_cache_mutex);
            return &device_cache[i];
        }
        
        if (is_mac_addr_zero(device_cache[i].addr)) {
            if (empty_slot == NULL) {
                empty_slot = &device_cache[i];
            }
        } else {
            if (device_cache[i].last_seen_tick < oldest_tick) {
                oldest_tick = device_cache[i].last_seen_tick;
                oldest_slot = &device_cache[i];
            }
        }
    }
    
    discovered_bt_classic_device_t *slot = empty_slot ? empty_slot : oldest_slot;
    
    if (slot) {
        memset(slot, 0, sizeof(discovered_bt_classic_device_t));
        memcpy(slot->addr, addr, 6);
        slot->last_seen_tick = current_tick;
        slot->logged_as_new_device = false;
        slot->logged_as_joycon = false;
    }
    
    xSemaphoreGive(device_cache_mutex);
    return slot;
}

esp_err_t joycon_bt_classic_discovery_init(void)
{
    // Очищаем кэш при инициализации
    memset(device_cache, 0, sizeof(device_cache));
    left_discovered_in_scan = false;
    right_discovered_in_scan = false;
    
    ESP_LOGI(TAG, "Discovery module initialized");
    return ESP_OK;
}

void joycon_bt_classic_discovery_deinit(void)
{
    memset(&discovery_callbacks, 0, sizeof(discovery_callbacks));
    memset(device_cache, 0, sizeof(device_cache));
    left_discovered_in_scan = false;
    right_discovered_in_scan = false;
    
    ESP_LOGI(TAG, "Discovery module deinitialized");
}

esp_err_t joycon_bt_classic_discovery_start(void)
{
    // Проверяем, что Bluedroid инициализирован
    extern bool bt_classic_initialized;
    if (!bt_classic_initialized) {
        ESP_LOGE(TAG, "Bluedroid not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Сбрасываем флаги обнаружения для нового сканирования
    left_discovered_in_scan = false;
    right_discovered_in_scan = false;
    
    // Очищаем кэш устройств
    joycon_bt_classic_discovery_clear_cache();
    
    // Запускаем GAP inquiry
    uint8_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
    uint8_t inq_len = BT_CLASSIC_INQUIRY_LENGTH;
    uint8_t inq_num_rsps = BT_CLASSIC_INQUIRY_MAX_RSP;
    
    esp_err_t ret = esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start discovery: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "Discovery started");
    
    if (discovery_callbacks.on_discovery_started) {
        discovery_callbacks.on_discovery_started();
    }
    
    return ESP_OK;
}

esp_err_t joycon_bt_classic_discovery_stop(void)
{
    esp_err_t ret = esp_bt_gap_cancel_discovery();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to cancel discovery: %d", ret);
    }
    
    return ret;
}

discovered_bt_classic_device_t* joycon_bt_classic_discovery_get_device(const uint8_t *addr)
{
    return find_or_create_device_cache_internal(addr);
}

void joycon_bt_classic_discovery_identify_joycon_type(const discovered_bt_classic_device_t *device,
                                                       const uint8_t *mac_addr,
                                                       bool *is_left, bool *is_right)
{
    if (!is_left || !is_right) {
        return;
    }
    
    *is_left = false;
    *is_right = false;
    
    if (!device || !mac_addr) {
        return;
    }
    
    if (device->has_name) {
        char name_buf[BT_DEVICE_NAME_BUFFER_SIZE];
        size_t name_len = strlen(device->name);
        size_t copy_len = (name_len < sizeof(name_buf) - 1) ? name_len : sizeof(name_buf) - 1;
        memcpy(name_buf, device->name, copy_len);
        name_buf[copy_len] = '\0';
        
        *is_left = (strstr(name_buf, "Joy-Con(L)") != NULL || 
                   strstr(name_buf, "Joy-Con (L)") != NULL || 
                   strstr(name_buf, "Joy-Con L") != NULL ||
                   strstr(name_buf, "joy-con(l)") != NULL ||
                   strstr(name_buf, "joy-con (l)") != NULL);
        *is_right = (strstr(name_buf, "Joy-Con(R)") != NULL || 
                    strstr(name_buf, "Joy-Con (R)") != NULL || 
                    strstr(name_buf, "Joy-Con R") != NULL ||
                    strstr(name_buf, "joy-con(r)") != NULL ||
                    strstr(name_buf, "joy-con (r)") != NULL);
    }
}

void joycon_bt_classic_discovery_subscribe(const joycon_bt_classic_discovery_callbacks_t *callbacks)
{
    if (callbacks) {
        // Безопасно: копируем структуры одинакового типа, sizeof гарантирует правильный размер
        memcpy(&discovery_callbacks, callbacks, sizeof(joycon_bt_classic_discovery_callbacks_t));
    } else {
        memset(&discovery_callbacks, 0, sizeof(joycon_bt_classic_discovery_callbacks_t));
    }
}

void joycon_bt_classic_discovery_clear_cache(void)
{
    if (device_cache_mutex != NULL) {
        if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memset(device_cache, 0, sizeof(device_cache));
            ESP_LOGI(TAG, "Device cache cleared");
            xSemaphoreGive(device_cache_mutex);
        }
    }
}

// Вспомогательные функции для внутреннего использования
bool joycon_bt_classic_discovery_is_left_discovered_in_scan(void)
{
    return left_discovered_in_scan;
}

bool joycon_bt_classic_discovery_is_right_discovered_in_scan(void)
{
    return right_discovered_in_scan;
}

void joycon_bt_classic_discovery_set_left_discovered(bool discovered)
{
    left_discovered_in_scan = discovered;
}

void joycon_bt_classic_discovery_set_right_discovered(bool discovered)
{
    right_discovered_in_scan = discovered;
}

/**
 * @brief Обработать событие изменения состояния сканирования
 */
static void handle_discovery_state_changed_event(esp_bt_gap_cb_param_t *param)
{
    // Состояние сканирования изменилось
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        ESP_LOGI(TAG, "Discovery stopped");
        
        if (discovery_callbacks.on_discovery_complete) {
            discovery_callbacks.on_discovery_complete();
        }
    } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        ESP_LOGI(TAG, "Discovery started");
    }
}

/**
 * @brief Обработать событие получения результата SDP
 */
static void handle_sdp_result_event(esp_bt_gap_cb_param_t *param)
{
    // SDP результат получен
    uint8_t *bda = param->rmt_srvc_rec.bda;
    
    discovered_bt_classic_device_t *cached_device = find_or_create_device_cache_internal(bda);
    if (!cached_device) {
        return;
    }
    
    // Помечаем SDP как завершенный
    if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        cached_device->sdp_complete = true;
        xSemaphoreGive(device_cache_mutex);
    }
    
    // Определяем тип Joy-Con
    bool is_left = false;
    bool is_right = false;
    joycon_bt_classic_discovery_identify_joycon_type(cached_device, bda, &is_left, &is_right);
    
    // Если устройство определено как Joy-Con, устанавливаем has_hid_service
    if (is_left || is_right) {
        if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            cached_device->has_hid_service = true;
            xSemaphoreGive(device_cache_mutex);
        }
    }
    
    // Также проверяем по MAC адресу (Joy-Con имеют характерные OUI)
    bool mac_looks_like_joycon = (bda[5] == JOYCON_OUI_1_BYTE_5 && bda[4] == JOYCON_OUI_1_BYTE_4) ||
                                (bda[5] == JOYCON_OUI_2_BYTE_5 && bda[4] == JOYCON_OUI_2_BYTE_4);
    
    if (mac_looks_like_joycon) {
        if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            cached_device->has_hid_service = true;
            xSemaphoreGive(device_cache_mutex);
        }
    }
    
    ESP_LOGI(TAG, "SDP completed for %02x:%02x:%02x:%02x:%02x:%02x (HID=%s)",
             MAC_STR(bda), cached_device->has_hid_service ? "yes" : "no");
    
    // Уведомляем через callback о найденном Joy-Con (если определен по MAC или имени)
    if ((is_left || is_right) && discovery_callbacks.on_device_found) {
        discovery_callbacks.on_device_found(cached_device, is_left, is_right);
    } else if (mac_looks_like_joycon && cached_device->has_hid_service && discovery_callbacks.on_device_found) {
        // Устройство похоже на Joy-Con по MAC, но тип не определен
        discovery_callbacks.on_device_found(cached_device, false, false);
    }
}

/**
 * @brief Обработать событие обнаружения устройства
 */
static void handle_discovery_result_event(esp_bt_gap_cb_param_t *param)
{
    // Устройство обнаружено
    uint8_t *bda = param->disc_res.bda;
    
    discovered_bt_classic_device_t *cached_device = find_or_create_device_cache_internal(bda);
    if (!cached_device) {
        return;
    }
    
    // УМНАЯ ФИЛЬТРАЦИЯ: Сначала проверяем, нужно ли обрабатывать это устройство
    // 1. Если есть сохраненные MAC-адреса - проверяем по MAC
    // 2. Если устройство не соответствует сохраненным адресам, но выглядит как Joy-Con
    //    и не все джойконы подключены - тоже обрабатываем (для поиска второго джойкона)
    // 3. Если сохраненных адресов нет (первый запуск) - фильтруем по имени "joy-con"
    
    bool is_saved_joycon = false;
    bool is_left_saved = false;
    bool is_right_saved = false;
    bool should_process = false;
    
    // Проверяем, подключены ли оба джойкона (для определения, нужно ли искать второй)
    bool both_connected_for_filter = false;
    extern SemaphoreHandle_t connection_mutex;
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
    
    if (connection_mutex != NULL) {
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
            joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
            if (left && right) {
                both_connected_for_filter = (left->connected || left->connecting) &&
                                           (right->connected || right->connecting);
            }
            xSemaphoreGive(connection_mutex);
        }
    }
    
    if (has_saved_joycon_addresses()) {
        // Есть сохраненные адреса - сначала проверяем по MAC
        is_saved_joycon = is_saved_joycon_address(bda, &is_left_saved, &is_right_saved);
        should_process = is_saved_joycon;
        
        // Если устройство не соответствует сохраненным адресам, но не все джойконы подключены,
        // проверяем, является ли оно Joy-Con (для поиска второго джойкона)
        if (!should_process && !both_connected_for_filter) {
            // Получаем имя из EIR, если доступно
            if (param->disc_res.num_prop > 0) {
                for (int i = 0; i < param->disc_res.num_prop; i++) {
                    esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
                    if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                        if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                            size_t name_len = prop->len < sizeof(cached_device->name) - 1 ? 
                                            prop->len : sizeof(cached_device->name) - 1;
                            if (name_len > 0) {
                                memcpy(cached_device->name, prop->val, name_len);
                                cached_device->name[name_len] = '\0';
                                cached_device->has_name = true;
                            }
                            xSemaphoreGive(device_cache_mutex);
                        }
                        break;
                    }
                }
            }
            
            // Проверяем по имени, если оно получено
            if (cached_device->has_name) {
                char name_lower[BT_DEVICE_NAME_BUFFER_SIZE];
                size_t name_len = strlen(cached_device->name);
                size_t copy_len = (name_len < sizeof(name_lower) - 1) ? name_len : sizeof(name_lower) - 1;
                memcpy(name_lower, cached_device->name, copy_len);
                name_lower[copy_len] = '\0';
                for (size_t i = 0; i < copy_len; i++) {
                    if (name_lower[i] >= 'A' && name_lower[i] <= 'Z') {
                        name_lower[i] = name_lower[i] - 'A' + 'a';
                    }
                }
                should_process = (strstr(name_lower, "joy-con") != NULL);
            } else {
                // Проверяем по MAC OUI (Joy-Con имеют характерные OUI)
                bool mac_looks_like_joycon = (bda[5] == JOYCON_OUI_1_BYTE_5 && bda[4] == JOYCON_OUI_1_BYTE_4) ||
                                            (bda[5] == JOYCON_OUI_2_BYTE_5 && bda[4] == JOYCON_OUI_2_BYTE_4);
                if (mac_looks_like_joycon) {
                    // Имя не получено, но MAC похож на Joy-Con - запрашиваем имя
                    esp_err_t ret = esp_bt_gap_read_remote_name(bda);
                    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                        ESP_LOGD(TAG, "Failed to request remote name for %02x:%02x:%02x:%02x:%02x:%02x: %d",
                                 MAC_STR(bda), ret);
                    }
                    // Откладываем проверку до получения имени
                    return;
                }
            }
        }
        
        if (!should_process) {
            // Устройство не соответствует сохраненным адресам и не выглядит как Joy-Con
            ESP_LOGD(TAG, "Skipping device %02x:%02x:%02x:%02x:%02x:%02x (not a saved Joy-Con%s)",
                     MAC_STR(bda), both_connected_for_filter ? " and both Joy-Con connected" : "");
            return;
        }
    } else {
        // Нет сохраненных адресов (первый запуск) - фильтруем по имени "joy-con"
        // Сначала получаем имя из EIR (Extended Inquiry Response), если доступно
        if (param->disc_res.num_prop > 0) {
            for (int i = 0; i < param->disc_res.num_prop; i++) {
                esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
                if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                    // Имя найдено в EIR
                    if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        size_t name_len = prop->len < sizeof(cached_device->name) - 1 ? 
                                        prop->len : sizeof(cached_device->name) - 1;
                        if (name_len > 0) {
                            memcpy(cached_device->name, prop->val, name_len);
                            cached_device->name[name_len] = '\0';
                            cached_device->has_name = true;
                        }
                        xSemaphoreGive(device_cache_mutex);
                    }
                    break;
                }
            }
        }
        
        // Проверяем имя, если оно уже получено
        if (cached_device->has_name) {
            char name_lower[BT_DEVICE_NAME_BUFFER_SIZE];
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
            should_process = (strstr(name_lower, "joy-con") != NULL);
            
            if (!should_process) {
                // Имя не содержит "joy-con" - пропускаем устройство
                ESP_LOGD(TAG, "Skipping non-Joy-Con device %02x:%02x:%02x:%02x:%02x:%02x (name='%s', doesn't contain 'joy-con')",
                         MAC_STR(bda), cached_device->name);
                // Не логируем, не делаем SDP
                return;
            }
        } else {
            // Имя еще не получено - запрашиваем его и отложим проверку
            // Устройство будет проверено в событии READ_REMOTE_NAME_EVT
            esp_err_t ret = esp_bt_gap_read_remote_name(bda);
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG, "Failed to request remote name for %02x:%02x:%02x:%02x:%02x:%02x: %d",
                         MAC_STR(bda), ret);
                // Не смогли запросить имя - пропускаем устройство
                ESP_LOGD(TAG, "Skipping device %02x:%02x:%02x:%02x:%02x:%02x (cannot get name)",
                         MAC_STR(bda));
                return;
            }
            // Имя запрошено, проверка будет в READ_REMOTE_NAME_EVT
            // Не делаем SDP запрос до получения имени
            return;
        }
    }
    
    // Устройство прошло фильтрацию - обрабатываем его
    // Получаем имя из EIR, если еще не получено
    if (!cached_device->has_name && param->disc_res.num_prop > 0) {
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
            if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                // Имя найдено в EIR
                if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    size_t name_len = prop->len < sizeof(cached_device->name) - 1 ? 
                                    prop->len : sizeof(cached_device->name) - 1;
                    if (name_len > 0) {
                        memcpy(cached_device->name, prop->val, name_len);
                        cached_device->name[name_len] = '\0';
                        cached_device->has_name = true;
                    }
                    xSemaphoreGive(device_cache_mutex);
                }
                break;
            }
        }
    }
    
    // Если имя еще не получено (для сохраненных адресов), запрашиваем его
    if (!cached_device->has_name && is_saved_joycon) {
        esp_err_t ret = esp_bt_gap_read_remote_name(bda);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGD(TAG, "Failed to request remote name for %02x:%02x:%02x:%02x:%02x:%02x: %d",
                     MAC_STR(bda), ret);
        }
    }
    
    // Логируем обнаружение устройства (только один раз)
    if (!cached_device->logged_as_new_device) {
        if (is_saved_joycon) {
            ESP_LOGI(TAG, "Saved Joy-Con discovered: addr=%02x:%02x:%02x:%02x:%02x:%02x, type=%s",
                     MAC_STR(bda), is_left_saved ? "Left" : "Right");
        } else {
            ESP_LOGI(TAG, "Joy-Con candidate discovered: addr=%02x:%02x:%02x:%02x:%02x:%02x, name='%s'",
                     MAC_STR(bda), cached_device->has_name ? cached_device->name : "no name");
        }
        cached_device->logged_as_new_device = true;
    }
    
    // Проверяем количество активных подключений перед SDP запросом
    // Разрешаем SDP запросы, если не все джойконы подключены (чтобы найти второй джойкон)
    bool both_connected = false;
    extern SemaphoreHandle_t connection_mutex;
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
    
    if (connection_mutex != NULL) {
        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
            joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
            if (left && right) {
                // Проверяем, подключены ли ОБА джойкона
                both_connected = (left->connected || left->connecting) &&
                                (right->connected || right->connecting);
            }
            xSemaphoreGive(connection_mutex);
        }
    }
    
    // Запрашиваем SDP если:
    // 1. Не все джойконы подключены (чтобы найти второй), ИЛИ
    // 2. Устройство точно подходит (сохраненный адрес)
    if (!both_connected || is_saved_joycon) {
        // Запрашиваем SDP для проверки HID сервиса
        if (!cached_device->sdp_requested) {
            if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                cached_device->sdp_requested = true;
                xSemaphoreGive(device_cache_mutex);
            }
            esp_err_t ret = esp_bt_gap_get_remote_services(bda);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to request SDP for %02x:%02x:%02x:%02x:%02x:%02x: %d",
                         MAC_STR(bda), ret);
                if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    cached_device->sdp_requested = false;
                    xSemaphoreGive(device_cache_mutex);
                }
            }
        }
    } else {
        ESP_LOGD(TAG, "Skipping SDP request for %02x:%02x:%02x:%02x:%02x:%02x (both Joy-Con already connected)",
                 MAC_STR(bda));
    }
}

/**
 * @brief Обработать событие получения имени удаленного устройства
 */
static void handle_read_remote_name_event(esp_bt_gap_cb_param_t *param)
{
    // Результат запроса имени устройства получен
    if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
        discovered_bt_classic_device_t *cached_device = find_or_create_device_cache_internal(param->read_rmt_name.bda);
        if (cached_device) {
            bool name_was_new = false;
            if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                // Сохраняем имя, если его еще нет
                if (!cached_device->has_name) {
                    size_t name_len = strnlen((const char*)param->read_rmt_name.rmt_name, 
                                             sizeof(param->read_rmt_name.rmt_name));
                    if (name_len > 0) {
                        size_t copy_len = name_len < sizeof(cached_device->name) - 1 ? 
                                        name_len : sizeof(cached_device->name) - 1;
                        memcpy(cached_device->name, param->read_rmt_name.rmt_name, copy_len);
                        cached_device->name[copy_len] = '\0';
                        cached_device->has_name = true;
                        name_was_new = true;
                        ESP_LOGI(TAG, "Remote name received for %02x:%02x:%02x:%02x:%02x:%02x: '%s'",
                                 MAC_STR(param->read_rmt_name.bda), cached_device->name);
                    }
                }
                xSemaphoreGive(device_cache_mutex);
            }
            
            // После получения имени проверяем, нужно ли обрабатывать это устройство
            // 1. Если есть сохраненные адреса - проверяем по MAC
            // 2. Если устройство не соответствует сохраненным адресам, но выглядит как Joy-Con
            //    и не все джойконы подключены - тоже обрабатываем (для поиска второго джойкона)
            // 3. Если сохраненных адресов нет (первый запуск) - фильтруем по имени "joy-con"
            if (name_was_new || cached_device->has_name) {
                bool should_process = false;
                bool is_saved_joycon = false;
                bool is_left_saved = false;
                bool is_right_saved = false;
                
                // Проверяем, подключены ли оба джойкона
                bool both_connected_for_filter = false;
                extern SemaphoreHandle_t connection_mutex;
                extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
                extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
                
                if (connection_mutex != NULL) {
                    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
                        joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
                        if (left && right) {
                            both_connected_for_filter = (left->connected || left->connecting) &&
                                                       (right->connected || right->connecting);
                        }
                        xSemaphoreGive(connection_mutex);
                    }
                }
                
                // Проверяем, есть ли сохраненные адреса
                if (has_saved_joycon_addresses()) {
                    // Есть сохраненные адреса - сначала проверяем по MAC
                    is_saved_joycon = is_saved_joycon_address(param->read_rmt_name.bda, &is_left_saved, &is_right_saved);
                    should_process = is_saved_joycon;
                    
                    // Если устройство не соответствует сохраненным адресам, но не все джойконы подключены,
                    // проверяем, является ли оно Joy-Con (для поиска второго джойкона)
                    if (!should_process && !both_connected_for_filter) {
                        if (cached_device->has_name && strlen(cached_device->name) > 0) {
                            char name_lower[BT_DEVICE_NAME_BUFFER_SIZE];
                            size_t name_len = strlen(cached_device->name);
                            size_t copy_len = (name_len < sizeof(name_lower) - 1) ? name_len : sizeof(name_lower) - 1;
                            memcpy(name_lower, cached_device->name, copy_len);
                            name_lower[copy_len] = '\0';
                            for (size_t i = 0; i < copy_len; i++) {
                                if (name_lower[i] >= 'A' && name_lower[i] <= 'Z') {
                                    name_lower[i] = name_lower[i] - 'A' + 'a';
                                }
                            }
                            should_process = (strstr(name_lower, "joy-con") != NULL);
                        }
                    }
                } else {
                    // Нет сохраненных адресов (первый запуск) - фильтруем по имени "joy-con"
                    if (cached_device->has_name && strlen(cached_device->name) > 0) {
                        char name_lower[BT_DEVICE_NAME_BUFFER_SIZE];
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
                        should_process = (strstr(name_lower, "joy-con") != NULL);
                        
                        if (!should_process) {
                            // Имя не содержит "joy-con" - пропускаем устройство
                            ESP_LOGD(TAG, "Skipping non-Joy-Con device %02x:%02x:%02x:%02x:%02x:%02x (name='%s', doesn't contain 'joy-con')",
                                     MAC_STR(param->read_rmt_name.bda), cached_device->name);
                            return;
                        }
                    } else {
                        // Имени нет - пропускаем устройство
                        ESP_LOGD(TAG, "Skipping device %02x:%02x:%02x:%02x:%02x:%02x (no name)",
                                 MAC_STR(param->read_rmt_name.bda));
                        return;
                    }
                }
                
                // Устройство прошло фильтрацию - определяем тип Joy-Con
                bool is_left = false;
                bool is_right = false;
                joycon_bt_classic_discovery_identify_joycon_type(cached_device, 
                                                                  param->read_rmt_name.bda, 
                                                                  &is_left, &is_right);
                
                // Запрашиваем SDP для инициирования ACL соединения (если еще не запрошен)
                // Только для устройств, которые прошли фильтрацию
                if (should_process && !cached_device->sdp_requested) {
                    // Проверяем, подключены ли ОБА джойкона перед SDP запросом
                    // Разрешаем SDP запросы, если не все джойконы подключены (чтобы найти второй джойкон)
                    bool both_connected = false;
                    extern SemaphoreHandle_t connection_mutex;
                    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void);
                    extern joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void);
                    
                    if (connection_mutex != NULL) {
                        if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            joycon_bt_classic_connection_t *left = joycon_bt_classic_state_get_left_joycon();
                            joycon_bt_classic_connection_t *right = joycon_bt_classic_state_get_right_joycon();
                            if (left && right) {
                                // Проверяем, подключены ли ОБА джойкона
                                both_connected = (left->connected || left->connecting) &&
                                                (right->connected || right->connecting);
                            }
                            xSemaphoreGive(connection_mutex);
                        }
                    }
                    
                    // Запрашиваем SDP если:
                    // 1. Не все джойконы подключены (чтобы найти второй), ИЛИ
                    // 2. Устройство точно подходит (сохраненный адрес)
                    if (!both_connected || is_saved_joycon) {
                        if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                            cached_device->sdp_requested = true;
                            xSemaphoreGive(device_cache_mutex);
                        }
                        esp_err_t sdp_ret = esp_bt_gap_get_remote_services(param->read_rmt_name.bda);
                        if (sdp_ret != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to request SDP for %02x:%02x:%02x:%02x:%02x:%02x: %d",
                                     MAC_STR(param->read_rmt_name.bda), sdp_ret);
                            if (xSemaphoreTake(device_cache_mutex, pdMS_TO_TICKS(BT_CLASSIC_DEVICE_CACHE_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                                cached_device->sdp_requested = false;
                                xSemaphoreGive(device_cache_mutex);
                            }
                        } else {
                            ESP_LOGI(TAG, "SDP request sent for %02x:%02x:%02x:%02x:%02x:%02x",
                                     MAC_STR(param->read_rmt_name.bda));
                        }
                    }
                }
                
                // Уведомляем через callback о найденном Joy-Con (если определен)
                if ((is_left || is_right) && discovery_callbacks.on_device_found) {
                    discovery_callbacks.on_device_found(cached_device, is_left, is_right);
                }
            }
        }
    } else {
        ESP_LOGD(TAG, "Failed to read remote name for %02x:%02x:%02x:%02x:%02x:%02x: status=%d",
                 MAC_STR(param->read_rmt_name.bda), param->read_rmt_name.stat);
    }
}

/**
 * @brief Обработать GAP событие discovery
 * 
 * Эта функция вызывается из joycon_bt_classic_scan.c для маршрутизации
 * событий discovery в модуль discovery.
 */
void joycon_bt_classic_discovery_handle_gap_event(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (!param) {
        return;
    }
    
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            handle_discovery_result_event(param);
            break;
        
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            handle_discovery_state_changed_event(param);
            break;
        
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            handle_sdp_result_event(param);
            break;
        
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            handle_read_remote_name_event(param);
            break;
        
        default:
            // Другие события не обрабатываются здесь
            break;
    }
}

/**
 * @file joycon_bt_classic_state.c
 * @brief Реализация модуля управления состоянием подключений Joy-Con
 * 
 * Централизованное управление состоянием обоих Joy-Con:
 * - Инкапсуляция структур подключения (left_joycon, right_joycon)
 * - Управление переходами состояний
 * - Синхронизация доступа
 * - Callbacks на изменение состояния
 */

#include "joycon_bt_classic_state.h"
#include "joycon_bt_classic_internal.h"
#include "../joycon_constants.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "JOYCON_BT_CLASSIC_STATE";

// Внутренние структуры подключения (инкапсулированы в модуле)
// ВНИМАНИЕ: Используем имена с _internal, чтобы избежать конфликта с макросами
static joycon_bt_classic_connection_t left_joycon_internal = {0};
static joycon_bt_classic_connection_t right_joycon_internal = {0};

// Callback для изменений состояния
static joycon_bt_classic_state_callback_t state_callback = NULL;

// Мьютекс для синхронизации доступа к состоянию
static SemaphoreHandle_t state_mutex = NULL;

/**
 * @brief Получить структуру подключения по типу (внутренняя функция)
 */
static joycon_bt_classic_connection_t* get_connection_internal(joycon_type_t type)
{
    if (type == JOYCON_TYPE_LEFT) {
        return &left_joycon_internal;
    } else if (type == JOYCON_TYPE_RIGHT) {
        return &right_joycon_internal;
    }
    return NULL;
}

/**
 * @brief Проверить допустимость перехода состояния
 * 
 * Определяет разрешенные переходы между состояниями:
 * - DISCONNECTED → SCANNING (OK)
 * - SCANNING → CONNECTING (OK)
 * - CONNECTING → CONNECTED (OK)
 * - CONNECTING → DISCONNECTED (OK) - при ошибке подключения
 * - CONNECTED → DISCONNECTED (OK) - при отключении
 * - Любой → ERROR (OK) - в случае ошибки
 */
static bool is_valid_transition(joycon_state_t current_state, joycon_state_t new_state)
{
    // Переход в ERROR всегда разрешен
    if (new_state == JOYCON_STATE_ERROR) {
        return true;
    }
    
    // Одинаковое состояние - не является переходом, но разрешено
    if (current_state == new_state) {
        return true;
    }
    
    // Разрешенные переходы
    switch (current_state) {
        case JOYCON_STATE_DISCONNECTED:
            return (new_state == JOYCON_STATE_SCANNING || new_state == JOYCON_STATE_CONNECTING);
            
        case JOYCON_STATE_SCANNING:
            return (new_state == JOYCON_STATE_CONNECTING || new_state == JOYCON_STATE_DISCONNECTED);
            
        case JOYCON_STATE_CONNECTING:
            return (new_state == JOYCON_STATE_CONNECTED || new_state == JOYCON_STATE_DISCONNECTED);
            
        case JOYCON_STATE_CONNECTED:
            return (new_state == JOYCON_STATE_DISCONNECTED);
            
        case JOYCON_STATE_ERROR:
            return (new_state == JOYCON_STATE_DISCONNECTED || new_state == JOYCON_STATE_SCANNING);
            
        default:
            return false;
    }
}

esp_err_t joycon_bt_classic_state_init(void)
{
    if (state_mutex != NULL) {
        ESP_LOGW(TAG, "State module already initialized");
        return ESP_OK;
    }
    
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Инициализация структур подключения
    memset(&left_joycon_internal, 0, sizeof(left_joycon_internal));
    memset(&right_joycon_internal, 0, sizeof(right_joycon_internal));
    left_joycon_internal.type = JOYCON_TYPE_LEFT;
    right_joycon_internal.type = JOYCON_TYPE_RIGHT;
    
    // Инициализация файловых дескрипторов L2CAP
    left_joycon_internal.l2cap_interrupt_fd = -1;
    left_joycon_internal.l2cap_control_fd = -1;
    right_joycon_internal.l2cap_interrupt_fd = -1;
    right_joycon_internal.l2cap_control_fd = -1;
    
    // Начальное состояние - DISCONNECTED
    left_joycon_internal.state = JOYCON_STATE_DISCONNECTED;
    right_joycon_internal.state = JOYCON_STATE_DISCONNECTED;
    left_joycon_internal.connected = false;
    left_joycon_internal.connecting = false;
    right_joycon_internal.connected = false;
    right_joycon_internal.connecting = false;
    
    ESP_LOGI(TAG, "State module initialized");
    return ESP_OK;
}

void joycon_bt_classic_state_deinit(void)
{
    if (state_mutex != NULL) {
        vSemaphoreDelete(state_mutex);
        state_mutex = NULL;
    }
    
    state_callback = NULL;
    
    ESP_LOGI(TAG, "State module deinitialized");
}

joycon_state_t joycon_bt_classic_state_get(joycon_type_t type)
{
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "State module not initialized");
        return JOYCON_STATE_ERROR;
    }
    
    joycon_state_t state = JOYCON_STATE_ERROR;
    
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection_internal(type);
        if (conn) {
            state = conn->state;
        }
        xSemaphoreGive(state_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take state mutex");
    }
    
    return state;
}

esp_err_t joycon_bt_classic_state_set(joycon_type_t type, joycon_state_t new_state)
{
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "State module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection_internal(type);
        if (conn) {
            joycon_state_t current_state = conn->state;
            
            // Проверяем валидность перехода (кроме случая, когда состояние не меняется)
            if (current_state != new_state && !is_valid_transition(current_state, new_state)) {
                xSemaphoreGive(state_mutex);
                ESP_LOGW(TAG, "Invalid state transition for %s: %d -> %d",
                         type == JOYCON_TYPE_LEFT ? "Left" : "Right",
                         current_state, new_state);
                return ESP_ERR_INVALID_STATE;
            }
            
            // Обновляем состояние
            if (current_state != new_state) {
                conn->state = new_state;
                
                // Синхронизируем connected с state
                conn->connected = (new_state == JOYCON_STATE_CONNECTED);
                
                // Синхронизируем connecting с state
                conn->connecting = (new_state == JOYCON_STATE_CONNECTING);
                
                xSemaphoreGive(state_mutex);
                
                // Вызываем callback вне мьютекса
                if (state_callback) {
                    state_callback(type, new_state);
                }
            } else {
                // Состояние не изменилось - не вызываем callback
                xSemaphoreGive(state_mutex);
            }
            
            return ESP_OK;
        }
        xSemaphoreGive(state_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take state mutex for state update");
    }
    
    return ESP_ERR_TIMEOUT;
}

bool joycon_bt_classic_state_can_transition(joycon_type_t type, joycon_state_t new_state)
{
    if (state_mutex == NULL) {
        return false;
    }
    
    bool can_transition = false;
    
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection_internal(type);
        if (conn) {
            can_transition = is_valid_transition(conn->state, new_state);
        }
        xSemaphoreGive(state_mutex);
    }
    
    return can_transition;
}

joycon_bt_classic_connection_t* joycon_bt_classic_state_get_connection(joycon_type_t type)
{
    // ВНИМАНИЕ: Эта функция возвращает указатель на внутреннюю структуру
    // Мьютекс должен быть захвачен вызывающей стороной через connection_mutex
    // Используется для обратной совместимости с существующим кодом
    
    return get_connection_internal(type);
}

void joycon_bt_classic_state_subscribe(joycon_bt_classic_state_callback_t callback)
{
    state_callback = callback;
}

// Функции для обратной совместимости - экспорт структур подключения
// ВНИМАНИЕ: Эти функции используются для миграции существующего кода
// В будущем их можно будет убрать после полной миграции

/**
 * @brief Получить указатель на левый Joy-Con (для обратной совместимости)
 * 
 * @deprecated Используйте joycon_bt_classic_state_get_connection() вместо этого
 */
joycon_bt_classic_connection_t* joycon_bt_classic_state_get_left_joycon(void)
{
    return &left_joycon_internal;
}

/**
 * @brief Получить указатель на правый Joy-Con (для обратной совместимости)
 * 
 * @deprecated Используйте joycon_bt_classic_state_get_connection() вместо этого
 */
joycon_bt_classic_connection_t* joycon_bt_classic_state_get_right_joycon(void)
{
    return &right_joycon_internal;
}

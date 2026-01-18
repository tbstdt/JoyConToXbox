/**
 * @file joycon_bt_classic_l2cap.c
 * @brief Модуль работы с L2CAP каналами для Joy-Con через Bluetooth Classic
 * 
 * Содержит:
 * - Инициализацию L2CAP модуля
 * - Открытие и закрытие L2CAP каналов (Interrupt и Control)
 * - Отправку Output Reports через Control канал
 * - Обработку Input Reports через Interrupt канал
 * - Управление состоянием L2CAP каналов
 */

#include "joycon_bt_classic_l2cap.h"
#include "joycon_bt_classic_internal.h"
#include "joycon_bt_classic.h"  // Для JOYCON_HID_REPORT_SIZE
#include "../joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_l2cap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

static const char *TAG = "JOYCON_BT_CLASSIC_L2CAP";

// Внешние объявления
extern joycon_bt_classic_connection_t left_joycon;
extern joycon_bt_classic_connection_t right_joycon;
extern SemaphoreHandle_t connection_mutex;
extern joycon_bt_classic_data_callback_t data_callback;
extern bool bt_classic_initialized;

// Флаг инициализации L2CAP модуля
static bool l2cap_initialized = false;
static SemaphoreHandle_t l2cap_init_semaphore = NULL;
static bool l2cap_init_in_progress = false;

// Структура для отслеживания состояния подключения L2CAP каналов
typedef struct {
    bool interrupt_connecting;
    bool control_connecting;
    uint32_t interrupt_handle;
    uint32_t control_handle;
    uint16_t interrupt_psm;  // PSM для Interrupt канала (для определения канала)
    uint16_t control_psm;    // PSM для Control канала (для определения канала)
    // Ожидаемые PSM для определения каналов при открытии
    uint16_t expected_interrupt_psm;  // Ожидаемый PSM для Interrupt канала
    uint16_t expected_control_psm;    // Ожидаемый PSM для Control канала
} l2cap_channel_state_t;

static l2cap_channel_state_t l2cap_states[2] = {0}; // Индекс 0 = Left, 1 = Right

// Резервный механизм handle -> PSM можно добавить в будущем при необходимости

/**
 * @brief Получить индекс джойкона по типу
 */
static int get_joycon_index(joycon_type_t type)
{
    return (type == JOYCON_TYPE_LEFT) ? 0 : 1;
}

/**
 * @brief Получить адрес джойкона по типу
 */
static uint8_t* get_joycon_addr(joycon_type_t type)
{
    if (type == JOYCON_TYPE_LEFT) {
        return left_joycon.addr;
    } else if (type == JOYCON_TYPE_RIGHT) {
        return right_joycon.addr;
    }
    return NULL;
}

/**
 * @brief Задача для установки режима Input Report Mode
 */
static void set_input_mode_task(void *pvParameters)
{
    joycon_type_t type = (joycon_type_t)(uintptr_t)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(200)); // Задержка для стабилизации
    joycon_bt_classic_set_input_report_mode(type, 0x30); // Стандартный отчет
    vTaskDelete(NULL);
}

/**
 * @brief Задача для переподключения L2CAP канала после неожиданного закрытия
 */
static void l2cap_reconnect_task(void *pvParameters)
{
    joycon_type_t type = (joycon_type_t)(uintptr_t)pvParameters;
    // Задержка перед переподключением (1 секунда)
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Attempting to reconnect L2CAP channel for %s Joy-Con",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    esp_err_t ret = joycon_bt_classic_open_l2cap_channels(type);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reconnect L2CAP channel for %s Joy-Con: %d",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right", ret);
    }
    vTaskDelete(NULL);
}

/**
 * @brief Callback для обработки событий L2CAP
 */
static void l2cap_callback(esp_bt_l2cap_cb_event_t event, esp_bt_l2cap_cb_param_t *param)
{
    if (!param) {
        return;
    }
    
    switch (event) {
        case ESP_BT_L2CAP_INIT_EVT: {
            if (param->init.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(TAG, "L2CAP initialized successfully (callback received)");
                l2cap_initialized = true;
                l2cap_init_in_progress = false;
                // Сигнализируем, что инициализация завершена
                if (l2cap_init_semaphore != NULL) {
                    xSemaphoreGive(l2cap_init_semaphore);
                }
            } else {
                ESP_LOGE(TAG, "L2CAP initialization failed in callback: %d", param->init.status);
                l2cap_initialized = false;
                l2cap_init_in_progress = false;
                // Сигнализируем об ошибке
                if (l2cap_init_semaphore != NULL) {
                    xSemaphoreGive(l2cap_init_semaphore);
                }
            }
            break;
        }
        
        case ESP_BT_L2CAP_UNINIT_EVT: {
            ESP_LOGI(TAG, "L2CAP deinitialized");
            l2cap_initialized = false;
            break;
        }
        
        case ESP_BT_L2CAP_CL_INIT_EVT: {
            // Инициирование подключения к L2CAP каналу
            ESP_LOGI(TAG, "[L2CAP] CL_INIT_EVT received: status=%d (0x%x), handle=%lu, sec_id=%d",
                     param->cl_init.status, param->cl_init.status, param->cl_init.handle, param->cl_init.sec_id);
            if (param->cl_init.status == ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGI(TAG, "[L2CAP] L2CAP connection initiated successfully, handle=%lu, sec_id=%d",
                         param->cl_init.handle, param->cl_init.sec_id);
                // PSM будет сохранен при вызове esp_bt_l2cap_connect() в структуре l2cap_states
            } else {
                ESP_LOGE(TAG, "[L2CAP] L2CAP connection initiation FAILED: status=%d (0x%x), handle=%lu",
                         param->cl_init.status, param->cl_init.status, param->cl_init.handle);
                // ВАЖНО: Очищаем ожидаемые PSM при ошибке инициации подключения
                // Если esp_bt_l2cap_connect() вернул ESP_OK, но инициация провалилась,
                // ожидаемые PSM остаются установленными, что может привести к ошибке
                // "l2cap_malloc_slot unable to register fd" при следующей попытке
                // Очищаем ожидаемые PSM для обоих джойконов, так как не можем точно определить
                // какой именно джойкон вызвал ошибку (в cl_init нет поля rem_bda)
                if (connection_mutex != NULL) {
                    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        // Очищаем ожидаемые PSM для обоих джойконов
                        for (int idx = 0; idx < 2; idx++) {
                            // Очищаем только если ожидаемые PSM установлены (значит была попытка подключения)
                            if (l2cap_states[idx].expected_interrupt_psm != 0 || 
                                l2cap_states[idx].expected_control_psm != 0) {
                                l2cap_states[idx].expected_interrupt_psm = 0;
                                l2cap_states[idx].expected_control_psm = 0;
                                l2cap_states[idx].interrupt_connecting = false;
                                l2cap_states[idx].control_connecting = false;
                                ESP_LOGI(TAG, "Cleared expected PSM states for joycon index %d due to L2CAP initiation failure", idx);
                            }
                        }
                        xSemaphoreGive(connection_mutex);
                    }
                }
            }
            break;
        }
        
        case ESP_BT_L2CAP_OPEN_EVT: {
            // L2CAP канал открыт
            uint8_t *bda = param->open.rem_bda;
            uint32_t handle = param->open.handle;
            int fd = param->open.fd;
            
            ESP_LOGI(TAG, "[L2CAP] OPEN_EVT received: status=%d (0x%x), handle=%lu, fd=%d, addr=%02x:%02x:%02x:%02x:%02x:%02x",
                     param->open.status, param->open.status, handle, fd, MAC_STR(bda));
            
            // Определяем, какой это джойкон
            joycon_type_t conn_type = JOYCON_TYPE_UNKNOWN;
            joycon_bt_classic_connection_t *conn = NULL;
            
            if (connection_mutex != NULL) {
                if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    if (memcmp(bda, left_joycon.addr, 6) == 0) {
                        conn = &left_joycon;
                        conn_type = JOYCON_TYPE_LEFT;
                    } else if (memcmp(bda, right_joycon.addr, 6) == 0) {
                        conn = &right_joycon;
                        conn_type = JOYCON_TYPE_RIGHT;
                    }
                    
                    if (conn) {
                        // Определяем, какой это канал (Interrupt или Control) по ожидаемому PSM
                        int idx = get_joycon_index(conn_type);
                        
                        // Устанавливаем неблокирующий режим для файлового дескриптора
                        int flags = fcntl(fd, F_GETFL, 0);
                        if (flags >= 0) {
                            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
                        }
                        
                        // Определяем тип канала по ожидаемому PSM из состояния
                        // Проверяем, какой канал ожидается (Interrupt или Control)
                        bool is_expected_interrupt = !conn->l2cap_interrupt_connected && 
                                                     l2cap_states[idx].expected_interrupt_psm == HID_INTERRUPT_PSM;
                        bool is_expected_control = !conn->l2cap_control_connected && 
                                                   l2cap_states[idx].expected_control_psm == HID_CONTROL_PSM;
                        
                        // Если ожидаемый PSM не установлен, используем резервный механизм по порядку открытия
                        if (!is_expected_interrupt && !is_expected_control) {
                            if (!conn->l2cap_interrupt_connected && !conn->l2cap_control_connected) {
                                // Первый открытый канал - предполагаем Interrupt (эвристика)
                                is_expected_interrupt = true;
                                ESP_LOGW(TAG, "Using fallback heuristic: first channel = Interrupt for %s Joy-Con",
                                         conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
                            } else if (!conn->l2cap_control_connected) {
                                // Второй канал - предполагаем Control (эвристика)
                                is_expected_control = true;
                                ESP_LOGW(TAG, "Using fallback heuristic: second channel = Control for %s Joy-Con",
                                         conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
                            }
                        }
                        
                        if (is_expected_interrupt) {
                            // Interrupt канал
                            conn->l2cap_interrupt_connected = true;
                            conn->l2cap_interrupt_handle = handle;
                            conn->l2cap_interrupt_fd = fd;
                            l2cap_states[idx].interrupt_handle = handle;
                            l2cap_states[idx].interrupt_psm = HID_INTERRUPT_PSM;
                            // Сбрасываем ожидаемый PSM
                            l2cap_states[idx].expected_interrupt_psm = 0;
                            ESP_LOGI(TAG, "L2CAP Interrupt channel opened for %s Joy-Con, handle=%lu, fd=%d, PSM=0x%02x",
                                     conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right", handle, fd, HID_INTERRUPT_PSM);
                            
                            // Теперь открываем Control канал после успешного открытия Interrupt канала
                            // Это предотвращает ошибку "l2cap_malloc_slot unable to register fd"
                            // Используем прямой вызов esp_bt_l2cap_connect() вместо retry логики,
                            // так как первый канал уже открыт успешно
                            if (!conn->l2cap_control_connected && l2cap_states[idx].expected_control_psm != 0) {
                                esp_err_t control_ret = esp_bt_l2cap_connect(ESP_BT_L2CAP_SEC_NONE, HID_CONTROL_PSM, bda);
                                if (control_ret == ESP_OK) {
                                    ESP_LOGI(TAG, "Initiating Control channel connection after Interrupt channel opened (PSM=0x%02x)", HID_CONTROL_PSM);
                                } else {
                                    ESP_LOGE(TAG, "Failed to initiate Control channel connection: %d", control_ret);
                                    l2cap_states[idx].expected_control_psm = 0;
                                }
                            }
                        } else if (is_expected_control) {
                            // Control канал
                            conn->l2cap_control_connected = true;
                            conn->l2cap_control_handle = handle;
                            conn->l2cap_control_fd = fd;
                            l2cap_states[idx].control_handle = handle;
                            l2cap_states[idx].control_psm = HID_CONTROL_PSM;
                            // Сбрасываем ожидаемый PSM
                            l2cap_states[idx].expected_control_psm = 0;
                            ESP_LOGI(TAG, "L2CAP Control channel opened for %s Joy-Con, handle=%lu, fd=%d, PSM=0x%02x",
                                     conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right", handle, fd, HID_CONTROL_PSM);
                            
                            // Оба канала открыты - отправляем команду Set Input Report Mode
                            // Задержка будет в отдельной задаче, чтобы не блокировать callback
                            xTaskCreate(set_input_mode_task, "set_mode", 2048, 
                                       (void*)(uintptr_t)conn_type, 5, NULL);
                        } else {
                            ESP_LOGW(TAG, "Unexpected L2CAP channel open event for %s Joy-Con, handle=%lu (both channels already connected)",
                                     conn_type == JOYCON_TYPE_LEFT ? "Left" : "Right", handle);
                        }
                    }
                    xSemaphoreGive(connection_mutex);
                }
            }
            
            if (param->open.status != ESP_BT_L2CAP_SUCCESS) {
                ESP_LOGW(TAG, "L2CAP channel open failed: %d", param->open.status);
                // Очищаем ожидаемые PSM при ошибке открытия канала
                // Это предотвращает проблемы при переподключении
                uint8_t *bda = param->open.rem_bda;
                if (bda && connection_mutex != NULL) {
                    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        int idx = -1;
                        if (memcmp(bda, left_joycon.addr, 6) == 0) {
                            idx = get_joycon_index(JOYCON_TYPE_LEFT);
                        } else if (memcmp(bda, right_joycon.addr, 6) == 0) {
                            idx = get_joycon_index(JOYCON_TYPE_RIGHT);
                        }
                        if (idx >= 0 && idx < 2) {
                            l2cap_states[idx].expected_interrupt_psm = 0;
                            l2cap_states[idx].expected_control_psm = 0;
                            l2cap_states[idx].interrupt_connecting = false;
                            l2cap_states[idx].control_connecting = false;
                        }
                        xSemaphoreGive(connection_mutex);
                    }
                }
            }
            break;
        }
        
        case ESP_BT_L2CAP_CLOSE_EVT: {
            // L2CAP канал закрыт
            uint32_t handle = param->close.handle;
            bool async = param->close.async;
            
            // Находим, какой джойкон и канал закрыт
            joycon_type_t closed_type = JOYCON_TYPE_UNKNOWN;
            bool was_interrupt = false;
            bool was_connected = false;
            
            if (connection_mutex != NULL) {
                if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    for (int i = 0; i < 2; i++) {
                        joycon_bt_classic_connection_t *conn = (i == 0) ? &left_joycon : &right_joycon;
                        if (conn->l2cap_interrupt_handle == handle) {
                            closed_type = (i == 0) ? JOYCON_TYPE_LEFT : JOYCON_TYPE_RIGHT;
                            was_interrupt = true;
                            was_connected = conn->l2cap_interrupt_connected;
                            conn->l2cap_interrupt_connected = false;
                            conn->l2cap_interrupt_handle = 0;
                            conn->l2cap_interrupt_fd = -1;
                            // Очищаем состояние канала
                            if (i < 2) {
                                l2cap_states[i].interrupt_handle = 0;
                                l2cap_states[i].interrupt_psm = 0;
                            }
                            ESP_LOGI(TAG, "L2CAP Interrupt channel closed for %s Joy-Con (async=%d)",
                                     i == 0 ? "Left" : "Right", async);
                            break;
                        } else if (conn->l2cap_control_handle == handle) {
                            closed_type = (i == 0) ? JOYCON_TYPE_LEFT : JOYCON_TYPE_RIGHT;
                            was_interrupt = false;
                            was_connected = conn->l2cap_control_connected;
                            conn->l2cap_control_connected = false;
                            conn->l2cap_control_handle = 0;
                            conn->l2cap_control_fd = -1;
                            // Очищаем состояние канала
                            if (i < 2) {
                                l2cap_states[i].control_handle = 0;
                                l2cap_states[i].control_psm = 0;
                            }
                            ESP_LOGI(TAG, "L2CAP Control channel closed for %s Joy-Con (async=%d)",
                                     i == 0 ? "Left" : "Right", async);
                            break;
                        }
                    }
                    xSemaphoreGive(connection_mutex);
                }
            }
            
            // Если канал был подключен (неожиданное закрытие), пытаемся переподключить
            if (was_connected && closed_type != JOYCON_TYPE_UNKNOWN) {
                ESP_LOGW(TAG, "Unexpected L2CAP %s channel closure for %s Joy-Con, attempting to reconnect...",
                         was_interrupt ? "Interrupt" : "Control",
                         closed_type == JOYCON_TYPE_LEFT ? "Left" : "Right");
                
                // Создаем задачу для переподключения канала с задержкой
                // Это позволяет избежать блокировки callback'а
                BaseType_t task_created = xTaskCreate(
                    l2cap_reconnect_task,
                    "l2cap_reconnect",
                    3072,
                    (void*)(uintptr_t)closed_type,
                    4,
                    NULL
                );
                
                if (task_created != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create L2CAP reconnection task");
                }
            }
            
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief Задача для чтения данных из L2CAP Interrupt канала
 */
static void l2cap_read_task(void *pvParameters)
{
    joycon_type_t type = (joycon_type_t)(uintptr_t)pvParameters;
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    
    ESP_LOGI(TAG, "L2CAP read task started for %s Joy-Con", type_str);
    
    uint8_t buffer[64]; // Буфер для чтения данных
    int fd = -1;
    
    while (1) {
        // Получаем file descriptor Interrupt канала
        if (connection_mutex != NULL) {
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                joycon_bt_classic_connection_t *conn = get_connection(type);
                if (conn && conn->l2cap_interrupt_connected && conn->l2cap_interrupt_fd >= 0) {
                    fd = conn->l2cap_interrupt_fd;
                } else {
                    fd = -1;
                }
                xSemaphoreGive(connection_mutex);
            }
        }
        
        if (fd >= 0) {
            // Читаем данные из L2CAP канала
            ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                // Проверяем корректность размера пакета Joy-Con
                // Joy-Con обычно отправляет отчеты размером 50 байт (49 байт данных + 1 байт заголовка)
                // но могут быть и меньшие отчеты (минимум 10 байт для базовой информации)
                // L2CAP может отправлять несколько отчетов в одном пакете, поэтому нужно обработать их по отдельности
                if (bytes_read < 10) {
                    ESP_LOGW(TAG, "Received too small packet from %s Joy-Con: %d bytes (minimum 10)",
                             type_str, bytes_read);
                    // Пропускаем пакет, но не прерываем чтение
                } else if (bytes_read > JOYCON_HID_REPORT_SIZE) {
                    // L2CAP может отправить несколько отчетов в одном пакете
                    // Обрабатываем каждый отчет отдельно (по 50 байт)
                    size_t offset = 0;
                    while (offset + JOYCON_HID_REPORT_SIZE <= (size_t)bytes_read) {
                        if (data_callback) {
                            data_callback(type, buffer + offset, JOYCON_HID_REPORT_SIZE);
                        }
                        offset += JOYCON_HID_REPORT_SIZE;
                    }
                    // Если остались байты - это начало следующего отчета, его нужно прочитать в следующей итерации
                    // Пока просто логируем
                    if (offset < (size_t)bytes_read) {
                        ESP_LOGD(TAG, "Incomplete packet from %s Joy-Con: %d bytes remaining (will be read in next iteration)",
                                 type_str, bytes_read - offset);
                    }
                } else {
                    // Стандартный размер пакета - передаем в callback
                    if (data_callback && bytes_read >= 10) {
                        data_callback(type, buffer, bytes_read);
                    }
                }
            } else if (bytes_read < 0) {
                // Ошибка чтения
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGW(TAG, "Error reading from L2CAP Interrupt channel for %s Joy-Con: %d",
                             type_str, errno);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Небольшая задержка при ошибке
                }
            }
        } else {
            // Канал еще не открыт - ждем
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Небольшая задержка для предотвращения перегрузки
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Инициализация L2CAP модуля
 */
esp_err_t joycon_bt_classic_l2cap_init(void)
{
    // Эта функция вызывается после esp_bluedroid_enable() в joycon_bt_classic_init(),
    // но до установки bt_classic_initialized в true. Проверку статуса Bluedroid не делаем,
    // так как порядок вызовов гарантирован кодом инициализации.
    
    if (l2cap_initialized) {
        ESP_LOGW(TAG, "L2CAP already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing L2CAP module...");
    
    // Создаем семафор для ожидания инициализации (если еще не создан)
    if (l2cap_init_semaphore == NULL) {
        l2cap_init_semaphore = xSemaphoreCreateBinary();
        if (l2cap_init_semaphore == NULL) {
            ESP_LOGE(TAG, "Failed to create L2CAP init semaphore");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Помечаем, что инициализация началась
    l2cap_init_in_progress = true;
    
    // Регистрация callback
    esp_err_t ret = esp_bt_l2cap_register_callback(l2cap_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register L2CAP callback: %d", ret);
        l2cap_init_in_progress = false;
        return ret;
    }
    
    // Инициализация L2CAP (асинхронная операция)
    ret = esp_bt_l2cap_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize L2CAP: %d", ret);
        l2cap_init_in_progress = false;
        return ret;
    }
    
    // Ждем завершения инициализации через callback (максимум 2 секунды)
    ESP_LOGI(TAG, "[L2CAP] Waiting for L2CAP initialization callback...");
    if (xSemaphoreTake(l2cap_init_semaphore, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (l2cap_initialized) {
            ESP_LOGI(TAG, "[L2CAP] L2CAP initialization completed successfully (callback received)");
            ESP_LOGI(TAG, "[L2CAP] L2CAP status: initialized=%s, init_in_progress=%s",
                     l2cap_initialized ? "yes" : "no", l2cap_init_in_progress ? "yes" : "no");
        } else {
            ESP_LOGE(TAG, "[L2CAP] L2CAP initialization failed (callback reported failure)");
            l2cap_init_in_progress = false;
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "[L2CAP] L2CAP initialization timeout - callback not received within 2 seconds");
        ESP_LOGE(TAG, "[L2CAP] Current state: initialized=%s, init_in_progress=%s",
                 l2cap_initialized ? "yes" : "no", l2cap_init_in_progress ? "yes" : "no");
        l2cap_init_in_progress = false;
        return ESP_ERR_TIMEOUT;
    }
    
    // ВАЖНО: В ESP-IDF для работы с L2CAP через файловые дескрипторы (fd) НЕОБХОДИМО
    // явно зарегистрировать VFS с помощью esp_bt_l2cap_vfs_register().
    // Без этого вызова файловые дескрипторы не могут быть зарегистрированы, что приводит
    // к ошибке "l2cap_malloc_slot unable to register fd!" при вызове esp_bt_l2cap_connect().
    // Эта функция должна быть вызвана ПОСЛЕ esp_bt_l2cap_init() и ДО esp_bt_l2cap_connect().
    ESP_LOGI(TAG, "[L2CAP] Registering L2CAP VFS for file descriptors...");
    ret = esp_bt_l2cap_vfs_register();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[L2CAP] Failed to register L2CAP VFS: %d (0x%x)", ret, ret);
        ESP_LOGE(TAG, "[L2CAP] This will cause 'l2cap_malloc_slot unable to register fd' errors!");
        // Продолжаем - возможно, в этой версии ESP-IDF VFS регистрируется автоматически
        // или функция недоступна, но это критическая проблема
    } else {
        ESP_LOGI(TAG, "[L2CAP] L2CAP VFS registered successfully - file descriptors can now be used");
    }
    
    // Создаем задачи для чтения данных из L2CAP каналов
    // Эти задачи будут ждать открытия каналов
    xTaskCreate(l2cap_read_task, "l2cap_read_l", 4096, (void*)(uintptr_t)JOYCON_TYPE_LEFT, 5, NULL);
    xTaskCreate(l2cap_read_task, "l2cap_read_r", 4096, (void*)(uintptr_t)JOYCON_TYPE_RIGHT, 5, NULL);
    
    ESP_LOGI(TAG, "[L2CAP] L2CAP module initialized - read tasks created");
    return ESP_OK;
}

/**
 * @brief Деинициализация L2CAP модуля
 */
esp_err_t joycon_bt_classic_l2cap_deinit(void)
{
    if (!l2cap_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing L2CAP module...");
    
    // Закрываем все каналы
    joycon_bt_classic_close_l2cap_channels(JOYCON_TYPE_LEFT);
    joycon_bt_classic_close_l2cap_channels(JOYCON_TYPE_RIGHT);
    
    // Деинициализация L2CAP
    esp_err_t ret = esp_bt_l2cap_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize L2CAP: %d", ret);
        return ret;
    }
    
    l2cap_initialized = false;
    
    // Освобождаем семафор при деинициализации
    if (l2cap_init_semaphore != NULL) {
        vSemaphoreDelete(l2cap_init_semaphore);
        l2cap_init_semaphore = NULL;
    }
    
    ESP_LOGI(TAG, "L2CAP module deinitialized");
    return ESP_OK;
}

// Константы для retry логики L2CAP подключений
#define L2CAP_CONNECT_MAX_RETRIES 3
#define L2CAP_CONNECT_BASE_DELAY_MS 200
#define L2CAP_CONNECT_MAX_DELAY_MS 1000

/**
 * @brief Вычисление экспоненциальной задержки для L2CAP подключения
 * 
 * @param attempt Номер попытки (начиная с 0)
 * @return uint32_t Задержка в миллисекундах
 */
static uint32_t calculate_l2cap_retry_delay(uint32_t attempt)
{
    if (attempt >= L2CAP_CONNECT_MAX_RETRIES) {
        return L2CAP_CONNECT_MAX_DELAY_MS;
    }
    
    // Экспоненциальная задержка: base * 2^attempt, ограниченная максимумом
    uint32_t delay = L2CAP_CONNECT_BASE_DELAY_MS * (1UL << attempt);
    if (delay > L2CAP_CONNECT_MAX_DELAY_MS) {
        delay = L2CAP_CONNECT_MAX_DELAY_MS;
    }
    
    return delay;
}

/**
 * @brief Подключиться к L2CAP каналу с retry логикой
 * 
 * @param sec_mask Маска безопасности
 * @param psm PSM канала
 * @param addr MAC адрес устройства
 * @return esp_err_t ESP_OK при успехе
 */
static esp_err_t l2cap_connect_with_retry(esp_bt_l2cap_cntl_flags_t sec_mask, 
                                          uint16_t psm, uint8_t *addr)
{
    esp_err_t ret = ESP_FAIL;
    
    ESP_LOGI(TAG, "[L2CAP] l2cap_connect_with_retry: PSM=0x%02x, addr=%02x:%02x:%02x:%02x:%02x:%02x, max_retries=%d",
             psm, MAC_STR(addr), L2CAP_CONNECT_MAX_RETRIES);
    
    for (uint32_t attempt = 0; attempt < L2CAP_CONNECT_MAX_RETRIES; attempt++) {
        ESP_LOGI(TAG, "[L2CAP] Attempt %lu/%d: Calling esp_bt_l2cap_connect(sec_mask=0x%x, psm=0x%02x)",
                 attempt + 1, L2CAP_CONNECT_MAX_RETRIES, sec_mask, psm);
        ret = esp_bt_l2cap_connect(sec_mask, psm, addr);
        ESP_LOGI(TAG, "[L2CAP] esp_bt_l2cap_connect() returned: %d (0x%x)", ret, ret);
        if (ret == ESP_OK) {
            if (attempt > 0) {
                ESP_LOGI(TAG, "[L2CAP] L2CAP channel connected on attempt %lu (PSM=0x%02x)", 
                         attempt + 1, psm);
            } else {
                ESP_LOGI(TAG, "[L2CAP] L2CAP connection initiated successfully on first attempt (PSM=0x%02x)", psm);
            }
            return ESP_OK;
        }
        
        // Если это не последняя попытка, ждем перед следующей
        if (attempt < L2CAP_CONNECT_MAX_RETRIES - 1) {
            uint32_t delay_ms = calculate_l2cap_retry_delay(attempt);
            ESP_LOGW(TAG, "L2CAP connection failed (attempt %lu/%d, PSM=0x%02x): %d, retrying in %lu ms",
                     attempt + 1, L2CAP_CONNECT_MAX_RETRIES, psm, ret, delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    ESP_LOGE(TAG, "L2CAP connection failed after %d attempts (PSM=0x%02x): %d", 
             L2CAP_CONNECT_MAX_RETRIES, psm, ret);
    return ret;
}

/**
 * @brief Открыть L2CAP каналы для Joy-Con
 */
esp_err_t joycon_bt_classic_open_l2cap_channels(joycon_type_t type)
{
    // Если L2CAP еще не инициализирован, ждем его инициализации
    if (!l2cap_initialized) {
        ESP_LOGW(TAG, "L2CAP not initialized yet, waiting for initialization...");
        
        // Если семафор не существует, значит инициализация не запускалась
        if (l2cap_init_semaphore == NULL) {
            ESP_LOGE(TAG, "L2CAP initialization was not started - joycon_bt_classic_l2cap_init() must be called first");
            return ESP_ERR_INVALID_STATE;
        }
        
        // Ждем завершения инициализации, проверяя флаг (максимум 2 секунды)
        TickType_t start_tick = xTaskGetTickCount();
        const TickType_t timeout_ticks = pdMS_TO_TICKS(2000);
        bool init_complete = false;
        
        while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
            if (l2cap_initialized) {
                init_complete = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Небольшая задержка
        }
        
        if (!init_complete) {
            ESP_LOGE(TAG, "L2CAP initialization timeout (waited %d ms)", 2000);
            return ESP_ERR_TIMEOUT;
        }
        
        ESP_LOGI(TAG, "L2CAP initialized successfully (waited)");
    }
    
    uint8_t *addr = get_joycon_addr(type);
    if (!addr || is_mac_addr_zero(addr)) {
        ESP_LOGE(TAG, "Invalid address for Joy-Con %s",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        return ESP_ERR_INVALID_ARG;
    }
    
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    ESP_LOGI(TAG, "[L2CAP] Opening L2CAP channels for %s Joy-Con at %02x:%02x:%02x:%02x:%02x:%02x",
             type_str, MAC_STR(addr));
    ESP_LOGI(TAG, "[L2CAP] L2CAP initialized: %s, init_in_progress: %s",
             l2cap_initialized ? "yes" : "no", l2cap_init_in_progress ? "yes" : "no");
    
    // Используем минимальные требования безопасности (Joy-Con обычно не требуют шифрования)
    esp_bt_l2cap_cntl_flags_t sec_mask = ESP_BT_L2CAP_SEC_NONE;
    
    // Получаем индекс для состояния L2CAP
    int idx = get_joycon_index(type);
    ESP_LOGI(TAG, "[L2CAP] Joy-Con index: %d", idx);
    
    // Очищаем состояния L2CAP только если каналы уже были открыты (для переподключения)
    // При первом подключении состояния должны быть чистыми, и их не нужно очищать
    if (idx >= 0 && idx < 2) {
        bool has_open_channels = false;
        if (connection_mutex != NULL) {
            if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                joycon_bt_classic_connection_t *conn = get_connection(type);
                if (conn) {
                    // Проверяем, были ли открыты каналы ранее
                    has_open_channels = (conn->l2cap_interrupt_fd >= 0) || 
                                       (conn->l2cap_control_fd >= 0) ||
                                       (conn->l2cap_interrupt_handle != 0) ||
                                       (conn->l2cap_control_handle != 0);
                    
                    // Очищаем только если каналы были открыты (переподключение)
                    if (has_open_channels) {
                        if (conn->l2cap_interrupt_fd >= 0) {
                            close(conn->l2cap_interrupt_fd);
                            conn->l2cap_interrupt_fd = -1;
                        }
                        if (conn->l2cap_control_fd >= 0) {
                            close(conn->l2cap_control_fd);
                            conn->l2cap_control_fd = -1;
                        }
                        // Полностью очищаем состояние L2CAP каналов
                        l2cap_states[idx].interrupt_connecting = false;
                        l2cap_states[idx].control_connecting = false;
                        l2cap_states[idx].interrupt_handle = 0;
                        l2cap_states[idx].control_handle = 0;
                        l2cap_states[idx].interrupt_psm = 0;
                        l2cap_states[idx].control_psm = 0;
                        l2cap_states[idx].expected_interrupt_psm = 0;
                        l2cap_states[idx].expected_control_psm = 0;
                    }
                }
                xSemaphoreGive(connection_mutex);
            }
        }
        
        // Если не было открытых каналов, просто сбрасываем ожидаемые PSM на всякий случай
        if (!has_open_channels) {
            ESP_LOGI(TAG, "[L2CAP] No previously open channels found, clearing expected PSM as precaution");
            l2cap_states[idx].expected_interrupt_psm = 0;
            l2cap_states[idx].expected_control_psm = 0;
        } else {
            ESP_LOGI(TAG, "[L2CAP] Found previously open channels, cleaned up states");
        }
    }
    
    // Устанавливаем ожидаемые PSM для определения каналов при открытии
    if (idx >= 0 && idx < 2) {
        ESP_LOGI(TAG, "[L2CAP] Setting expected PSM: Interrupt=0x%02x, Control=0x%02x",
                 HID_INTERRUPT_PSM, HID_CONTROL_PSM);
        l2cap_states[idx].expected_interrupt_psm = HID_INTERRUPT_PSM;
        l2cap_states[idx].expected_control_psm = HID_CONTROL_PSM;
        ESP_LOGI(TAG, "[L2CAP] State after setting PSM: interrupt_connecting=%d, control_connecting=%d",
                 l2cap_states[idx].interrupt_connecting, l2cap_states[idx].control_connecting);
    }
    
    // Открываем Interrupt канал (PSM 0x13) для получения Input Reports с retry логикой
    // Control канал будет открыт автоматически в callback'е после успешного открытия Interrupt канала
    // Это предотвращает ошибку "l2cap_malloc_slot unable to register fd" при одновременном открытии каналов
    ESP_LOGI(TAG, "[L2CAP] About to call esp_bt_l2cap_connect() for Interrupt channel (PSM=0x%02x)", HID_INTERRUPT_PSM);
    esp_err_t ret = l2cap_connect_with_retry(sec_mask, HID_INTERRUPT_PSM, addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Interrupt channel after retries: %d", ret);
        // Сбрасываем ожидаемый PSM при ошибке
        l2cap_states[idx].expected_interrupt_psm = 0;
        l2cap_states[idx].expected_control_psm = 0;
        return ret;
    }
    ESP_LOGI(TAG, "Connecting to L2CAP Interrupt channel (PSM=0x%02x)", HID_INTERRUPT_PSM);
    ESP_LOGI(TAG, "Control channel will be opened automatically after Interrupt channel is established");
    
    // Возвращаем ESP_OK - Control канал будет открыт в callback'е ESP_BT_L2CAP_OPEN_EVT для Interrupt канала
    return ESP_OK;
}

/**
 * @brief Закрыть L2CAP каналы для Joy-Con
 */
esp_err_t joycon_bt_classic_close_l2cap_channels(joycon_type_t type)
{
    if (connection_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    joycon_bt_classic_connection_t *conn = get_connection(type);
    if (!conn) {
        xSemaphoreGive(connection_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Получаем индекс для очистки состояния l2cap_states
    int idx = get_joycon_index(type);
    
    // Закрываем Interrupt канал
    if (conn->l2cap_interrupt_fd >= 0) {
        close(conn->l2cap_interrupt_fd);
        conn->l2cap_interrupt_fd = -1;
    }
    conn->l2cap_interrupt_connected = false;
    conn->l2cap_interrupt_handle = 0;
    
    // Закрываем Control канал
    if (conn->l2cap_control_fd >= 0) {
        close(conn->l2cap_control_fd);
        conn->l2cap_control_fd = -1;
    }
    conn->l2cap_control_connected = false;
    conn->l2cap_control_handle = 0;
    
    // Очищаем состояние L2CAP каналов для предотвращения проблем при переподключении
    // Это важно, чтобы избежать ошибок "l2cap_malloc_slot unable to register fd"
    if (idx >= 0 && idx < 2) {
        l2cap_states[idx].interrupt_connecting = false;
        l2cap_states[idx].control_connecting = false;
        l2cap_states[idx].interrupt_handle = 0;
        l2cap_states[idx].control_handle = 0;
        l2cap_states[idx].interrupt_psm = 0;
        l2cap_states[idx].control_psm = 0;
        l2cap_states[idx].expected_interrupt_psm = 0;
        l2cap_states[idx].expected_control_psm = 0;
    }
    
    xSemaphoreGive(connection_mutex);
    
    ESP_LOGI(TAG, "L2CAP channels closed for %s Joy-Con",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right");
    
    return ESP_OK;
}

/**
 * @brief Отправить Output Report через L2CAP Control канал
 */
esp_err_t joycon_bt_classic_send_output_report(joycon_type_t type, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (connection_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    int fd = -1;
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(BT_CLASSIC_CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection(type);
        if (conn && conn->l2cap_control_connected && conn->l2cap_control_fd >= 0) {
            fd = conn->l2cap_control_fd;
        }
        xSemaphoreGive(connection_mutex);
    }
    
    if (fd < 0) {
        ESP_LOGW(TAG, "Control channel not ready for %s Joy-Con",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Отправляем данные через file descriptor с обработкой частичных записей
    const uint8_t *remaining_data = data;
    size_t remaining_len = len;
    const uint32_t max_write_attempts = 5;  // Максимум попыток записи
    uint32_t write_attempts = 0;
    
    while (remaining_len > 0 && write_attempts < max_write_attempts) {
        ssize_t bytes_written = write(fd, remaining_data, remaining_len);
        
        if (bytes_written < 0) {
            // Ошибка записи
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Буфер полон, небольшая задержка и повтор
                vTaskDelay(pdMS_TO_TICKS(10));
                write_attempts++;
                continue;
            } else {
                ESP_LOGE(TAG, "Failed to write to Control channel: %d (attempt %lu/%d)",
                         errno, write_attempts + 1, max_write_attempts);
                return ESP_FAIL;
            }
        } else if (bytes_written == 0) {
            // Ничего не записано, возможно буфер полон
            vTaskDelay(pdMS_TO_TICKS(10));
            write_attempts++;
            continue;
        } else if ((size_t)bytes_written < remaining_len) {
            // Частичная запись - дозаписываем оставшиеся данные
            if (write_attempts == 0) {
                ESP_LOGD(TAG, "Partial write to Control channel: %d/%d bytes, continuing...",
                         bytes_written, remaining_len);
            }
            remaining_data += bytes_written;
            remaining_len -= bytes_written;
            write_attempts++;
            // Небольшая задержка перед следующей попыткой
            if (remaining_len > 0) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        } else {
            // Все данные записаны
            break;
        }
    }
    
    if (remaining_len > 0) {
        ESP_LOGW(TAG, "Failed to write all data to Control channel: %d/%d bytes written after %lu attempts",
                 len - remaining_len, len, write_attempts);
        return ESP_FAIL;
    }
    
    if (write_attempts > 0) {
        ESP_LOGD(TAG, "Output report sent successfully to %s Joy-Con (%lu write attempts)",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right", write_attempts + 1);
    }
    
    return ESP_OK;
}

/**
 * @brief Отправить Subcommand джойкону
 */
esp_err_t joycon_bt_classic_send_subcommand(joycon_type_t type, uint8_t subcmd, const uint8_t *data, size_t data_len)
{
    // Формат Output Report для Joy-Con:
    // [0x01, 0x00, 0x01, Subcommand, ...data]
    // 0x01 - Output Report ID
    // 0x00, 0x01 - Timestamp/Rumble (если нужно)
    // Subcommand - ID subcommand
    // data - данные subcommand
    
    uint8_t report[64] = {0};
    report[0] = 0x01; // Output Report ID
    report[1] = 0x00; // Rumble/Timestamp байт 1
    report[2] = 0x01; // Rumble/Timestamp байт 2
    report[3] = subcmd; // Subcommand ID
    
    if (data && data_len > 0) {
        size_t copy_len = (data_len < sizeof(report) - 4) ? data_len : sizeof(report) - 4;
        memcpy(&report[4], data, copy_len);
    }
    
    size_t report_len = 4 + (data_len > 0 ? data_len : 0);
    
    return joycon_bt_classic_send_output_report(type, report, report_len);
}

/**
 * @brief Установить режим Input Report Mode
 */
esp_err_t joycon_bt_classic_set_input_report_mode(joycon_type_t type, uint8_t report_id)
{
    // Subcommand 0x50 - Set Input Report Mode
    // Data: [report_id, enable]
    uint8_t data[] = {report_id, 0x01}; // report_id, enable=1
    
    ESP_LOGI(TAG, "Setting Input Report Mode for %s Joy-Con (report_id=0x%02x)",
             type == JOYCON_TYPE_LEFT ? "Left" : "Right", report_id);
    
    esp_err_t ret = joycon_bt_classic_send_subcommand(type, 0x50, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Input Report Mode: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "Input Report Mode set successfully");
    return ESP_OK;
}

/**
 * @brief Проверить, открыты ли L2CAP каналы для джойкона
 */
bool joycon_bt_classic_l2cap_channels_ready(joycon_type_t type)
{
    if (connection_mutex == NULL) {
        return false;
    }
    
    bool ready = false;
    if (xSemaphoreTake(connection_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        joycon_bt_classic_connection_t *conn = get_connection(type);
        if (conn) {
            ready = conn->l2cap_interrupt_connected && conn->l2cap_control_connected;
        }
        xSemaphoreGive(connection_mutex);
    }
    
    return ready;
}

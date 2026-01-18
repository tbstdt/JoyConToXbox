/**
 * @file main.c
 * @brief Главный файл приложения Joy-Con to Xbox Controller
 * 
 * Координирует работу всех модулей системы:
 * - Инициализация компонентов (BLE, HID, эмулятор Xbox, менеджер Joy-Con)
 * - Создание и управление задачами FreeRTOS
 * - Основной цикл обработки данных от Joy-Con и отправки на хост
 * - Мониторинг подключения и автоматическое переподключение
 * - Обработка кнопки для очистки сохраненных адресов
 * 
 * Архитектура:
 * - main_task: основной цикл (100 Гц) - получение данных, преобразование, отправка
 * - connection_monitor_task: мониторинг подключения и переподключение
 * - button_task: обработка кнопки устройства
 */

#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "joycon/joycon_types.h"
#include "joycon/bt_classic/joycon_bt_classic.h"

#include "status/status_led.h"
#include "ble_hid/ble_hid.h"
#include "joycon/joycon_manager.h"
#include "joycon/joycon_constants.h"
#include "xbox/xbox_emulator.h"

static const char *TAG = "MAIN";

// Задержка индикации LED при очистке адресов (мс)
#define LED_FEEDBACK_DELAY_MS 500

// Буфер для HID отчета (статический для предсказуемости использования памяти)
// Используется только в main_task, поэтому безопасен без дополнительной синхронизации
static uint8_t xbox_report[XBOX_HID_REPORT_SIZE];

/**
 * @brief Callback для получения команд вибрации от BLE HID
 * 
 * Вызывается когда хост (компьютер/планшет) отправляет команду вибрации
 * через BLE HID. Передает команду в менеджер Joy-Con для отправки на контроллеры.
 * 
 * @param left_motor Сила вибрации левого мотора (0-255)
 * @param right_motor Сила вибрации правого мотора (0-255)
 */
static void on_ble_hid_vibration(uint8_t left_motor, uint8_t right_motor)
{
    ESP_LOGD(TAG, "Vibration: L=%d, R=%d", left_motor, right_motor);
    esp_err_t ret = joycon_manager_set_vibration(left_motor, right_motor);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set vibration: %d", ret);
    }
}

/**
 * @brief Основная задача обработки данных от Joy-Con
 * 
 * Главный цикл приложения, который:
 * 1. Получает объединенное состояние от обоих Joy-Con
 * 2. Преобразует его в формат Xbox контроллера
 * 3. Отправляет через BLE HID на хост (компьютер/планшет)
 * 4. Обновляет индикацию состояния подключения
 * 
 * Работает с частотой UPDATE_RATE_HZ (100 Гц, интервал 10 мс).
 * 
 * @param pvParameters Параметры задачи (не используется, должен быть NULL)
 */
static void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Main task started");
    
    joycon_combined_state_t joycon_state;
    bool both_connected = false;
    bool left_connected_prev = false;
    bool right_connected_prev = false;
    
    while (1) {
        // Шаг 1: Получаем объединенное состояние от Joy-Con
        // Эта функция объединяет данные от левого и правого Joy-Con в единую структуру
        esp_err_t ret = joycon_manager_get_combined_state(&joycon_state);
        
        // Шаг 2: Проверяем состояние подключения из уже полученной структуры
        // Используем данные из joycon_state, чтобы избежать повторных вызовов API
        bool new_both_connected = joycon_state.left_connected && joycon_state.right_connected;
        bool left_connected = joycon_state.left_connected;
        bool right_connected = joycon_state.right_connected;
        
        // Шаг 3: Обновляем индикацию при изменении состояния подключения
        // Проверяем изменения каждого джойкона отдельно, чтобы корректно обработать
        // случай, когда один подключается, а второй еще нет
        if (new_both_connected != both_connected || 
            left_connected != left_connected_prev || 
            right_connected != right_connected_prev) {
            
            both_connected = new_both_connected;
            left_connected_prev = left_connected;
            right_connected_prev = right_connected;
            
            if (both_connected) {
                ESP_LOGI(TAG, "Both Joy-Con connected!");
                status_led_set_state(STATUS_LED_CONNECTED);
            } else {
                // Если хотя бы один подключен, показываем CONNECTING, иначе SCANNING
                // Используем данные из joycon_state вместо повторного вызова API
                bool any_connected = left_connected || right_connected;
                if (any_connected) {
                    ESP_LOGI(TAG, "One Joy-Con connected (L:%d, R:%d)", left_connected, right_connected);
                    status_led_set_state(STATUS_LED_CONNECTING);
                } else {
                    status_led_set_state(STATUS_LED_SCANNING);
                }
            }
        }
        
        if (ret == ESP_OK && joycon_state.valid) {
            // Шаг 4: Преобразуем состояние Joy-Con в формат Xbox контроллера
            // Это включает преобразование кнопок, стиков, D-Pad и триггеров
            ret = xbox_emulator_convert_to_report(&joycon_state, xbox_report);
            if (ret == ESP_OK) {
                // Шаг 5: Отправляем отчет через BLE HID на хост
                // Проверяем готовность BLE HID перед отправкой, чтобы не засорять логи ошибками
                // когда хост еще не подключен (ESP_ERR_INVALID_STATE)
                if (ble_hid_is_ready()) {
                    ret = ble_hid_send_report(xbox_report, XBOX_HID_REPORT_SIZE);
                    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                        // Логируем только реальные ошибки, не ESP_ERR_INVALID_STATE
                        // (который возникает когда хост не подключен или не подписан на notifications)
                        ESP_LOGW(TAG, "Failed to send BLE HID report: %d", ret);
                    }
                }
            }
        }
        
        // Шаг 6: Обновляем индикацию LED (мигание для состояний SCANNING/CONNECTING)
        status_led_update();
        
        // Задержка для поддержания частоты обновления (100 Гц = 10 мс)
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

/**
 * @brief Структура для хранения состояния переподключения Joy-Con
 * 
 * Используется для отслеживания попыток переподключения и управления
 * экспоненциальной задержкой между попытками.
 */
typedef struct {
    uint32_t reconnect_attempts;  ///< Количество попыток переподключения
    joycon_state_t last_state;    ///< Последнее известное состояние подключения
} joycon_reconnect_state_t;

/**
 * @brief Вычисление экспоненциальной задержки для переподключения
 * 
 * Использует экспоненциальную задержку с ограничениями для предотвращения переполнения
 * и слишком долгих ожиданий.
 * 
 * @param attempts Количество попыток переподключения
 * @return uint32_t Задержка в миллисекундах (от 0 до RECONNECT_MAX_DELAY_MS)
 */
static uint32_t calculate_reconnect_delay(uint32_t attempts)
{
    // Ограничиваем количество попыток для безопасного вычисления
    uint32_t attempt_limit = (attempts < RECONNECT_MAX_ATTEMPTS) ? 
                             attempts : RECONNECT_MAX_ATTEMPTS;
    
    // Ограничиваем attempt_limit для безопасного сдвига
    if (attempt_limit > RECONNECT_MAX_SAFE_SHIFT) {
        attempt_limit = RECONNECT_MAX_SAFE_SHIFT;
    }
    
    // Безопасное вычисление экспоненциальной задержки
    // Используем 64-битную арифметику для предотвращения переполнения
    uint64_t calculated_delay_64 = ((uint64_t)1 << attempt_limit) * RECONNECT_BASE_DELAY_MS;
    
    // Ограничиваем максимальную задержку и проверяем на переполнение uint32_t
    if (calculated_delay_64 > RECONNECT_MAX_DELAY_MS || calculated_delay_64 == 0 || 
        calculated_delay_64 > UINT32_MAX) {
        return RECONNECT_MAX_DELAY_MS;
    }
    
    return (uint32_t)calculated_delay_64;
}

/**
 * @brief Обработка переподключения для одного Joy-Con
 * 
 * Управляет логикой переподключения при потере соединения:
 * - Вычисляет экспоненциальную задержку между попытками
 * - Запускает сканирование при отключении
 * - Сбрасывает счетчик при успешном подключении
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @param current_state Текущее состояние подключения
 * @param reconnect_state Указатель на структуру состояния переподключения
 * @return Задержка в миллисекундах до следующей попытки (0 если не требуется)
 */
static uint32_t handle_joycon_reconnection(joycon_type_t type, joycon_state_t current_state,
                                          joycon_reconnect_state_t *reconnect_state)
{
    bool is_connected = (current_state == JOYCON_STATE_CONNECTED);
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    
    if (!is_connected) {
        if (current_state == JOYCON_STATE_DISCONNECTED || current_state == JOYCON_STATE_ERROR) {
            // Вычисляем задержку переподключения
            uint32_t delay_ms = calculate_reconnect_delay(reconnect_state->reconnect_attempts);
            
            // Если состояние изменилось или это первая попытка, логируем и запускаем сканирование
            if (reconnect_state->last_state != current_state || reconnect_state->reconnect_attempts == 0) {
                ESP_LOGI(TAG, "%s Joy-Con disconnected, attempting reconnect (attempt %lu, delay %lu ms)...",
                         type_str, reconnect_state->reconnect_attempts + 1, delay_ms);
                reconnect_state->reconnect_attempts++;
                reconnect_state->last_state = current_state;
                
                // Запускаем сканирование через Bluetooth Classic
                extern esp_err_t joycon_bt_classic_start_scan(void);
                joycon_bt_classic_start_scan();
            }
            
            return delay_ms;
        } else if (current_state == JOYCON_STATE_CONNECTING) {
            // Сбрасываем счетчик при подключении
            reconnect_state->reconnect_attempts = 0;
        }
    } else {
        // Подключен - сбрасываем счетчик
        if (reconnect_state->last_state != JOYCON_STATE_CONNECTED) {
            ESP_LOGI(TAG, "%s Joy-Con reconnected successfully", type_str);
            reconnect_state->reconnect_attempts = 0;
            reconnect_state->last_state = JOYCON_STATE_CONNECTED;
        }
    }
    
    return 0; // Нет задержки
}

/**
 * @brief Мьютекс для защиты состояния переподключения
 * 
 * Защищает структуры reconnect_states от одновременного доступа
 * из разных задач. Создается в app_main() перед запуском задач.
 */
static SemaphoreHandle_t reconnect_mutex = NULL;

/**
 * @brief Задача для мониторинга состояния подключения Joy-Con
 * 
 * Отслеживает состояние подключения обоих Joy-Con и управляет
 * автоматическим переподключением при потере соединения.
 * Использует экспоненциальную задержку для переподключения.
 * 
 * Алгоритм работы:
 * 1. Проверяет состояние подключения обоих Joy-Con
 * 2. Для каждого отключенного Joy-Con запускает переподключение
 * 3. Вычисляет задержку до следующей попытки (экспоненциальная)
 * 4. Ждет перед следующей проверкой
 * 
 * @param pvParameters Параметры задачи (не используется, должен быть NULL)
 */
static void connection_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Connection monitor task started");
    
    // Мьютекс должен быть создан в app_main() до запуска задачи
    if (reconnect_mutex == NULL) {
        ESP_LOGE(TAG, "Reconnect mutex not initialized");
        vTaskDelete(NULL);
        return;
    }
    
    // Состояние переподключения для каждого Joy-Con
    static joycon_reconnect_state_t reconnect_states[2] = {0};
    
    while (1) {
        // Проверяем таймауты подключения
        extern void joycon_bt_classic_check_connection_timeouts(void);
        joycon_bt_classic_check_connection_timeouts();
        
        // Проверяем состояние подключения через Bluetooth Classic
        joycon_state_t left_state = joycon_bt_classic_get_state(JOYCON_TYPE_LEFT);
        joycon_state_t right_state = joycon_bt_classic_get_state(JOYCON_TYPE_RIGHT);
        
        bool left_connected = (left_state == JOYCON_STATE_CONNECTED);
        bool right_connected = (right_state == JOYCON_STATE_CONNECTED);
        
        // Обработка переподключения для каждого Joy-Con (защищено мьютексом)
        uint32_t left_delay = 0;
        uint32_t right_delay = 0;
        
        if (xSemaphoreTake(reconnect_mutex, pdMS_TO_TICKS(CONNECTION_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            left_delay = handle_joycon_reconnection(JOYCON_TYPE_LEFT, left_state, &reconnect_states[0]);
            right_delay = handle_joycon_reconnection(JOYCON_TYPE_RIGHT, right_state, &reconnect_states[1]);
            xSemaphoreGive(reconnect_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take reconnect mutex, skipping this cycle");
        }
        
        // Выбираем максимальную задержку, если нужно ждать переподключения
        uint32_t max_delay = (left_delay > right_delay) ? left_delay : right_delay;
        
        if (max_delay > 0) {
            // Ждем перед следующей попыткой переподключения
            vTaskDelay(pdMS_TO_TICKS(max_delay));
        } else {
            // Проверяем каждые 5 секунд (если оба подключены)
            if (left_connected && right_connected) {
                vTaskDelay(pdMS_TO_TICKS(CONNECTION_MONITOR_INTERVAL_MS));
            } else {
                // Небольшая задержка, если один из Joy-Con не подключен
                vTaskDelay(pdMS_TO_TICKS(CONNECTION_CHECK_INTERVAL_MS));
            }
        }
    }
}

/**
 * @brief Задача для обработки кнопки Atom Lite
 * 
 * Обрабатывает нажатия кнопки на устройстве (например, M5Stack Atom Lite).
 * При удержании кнопки в течение 3 секунд очищает сохраненные MAC-адреса
 * Joy-Con из NVS, что позволяет подключиться к новым контроллерам.
 * 
 * Алгоритм работы:
 * 1. Опрашивает состояние кнопки каждые BUTTON_POLL_INTERVAL_MS мс
 * 2. Отслеживает время удержания кнопки
 * 3. При удержании > 3 секунд: очищает адреса и перезапускает сканирование
 * 4. Показывает визуальную индикацию (красный -> зеленый)
 * 
 * @param pvParameters Параметры задачи (не используется, должен быть NULL)
 */
static void button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Button task started");
    
    // Настройка GPIO для кнопки
    // Для Atom Lite кнопка обычно на GPIO 39, но может быть и GPIO 41
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO %d: %d", BUTTON_GPIO, ret);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Button GPIO %d configured successfully", BUTTON_GPIO);
    
    bool button_pressed = false;
    uint32_t press_start_time = 0;
    const TickType_t hold_ticks = pdMS_TO_TICKS(BUTTON_CLEAR_HOLD_TIME_MS);
    
    while (1) {
        int level = gpio_get_level(BUTTON_GPIO);
        
        // Кнопка нажата (низкий уровень из-за pull-up)
        if (level == 0) {
            if (!button_pressed) {
                // Начало нажатия
                button_pressed = true;
                press_start_time = xTaskGetTickCount();
                ESP_LOGI(TAG, "Button pressed, hold for %d ms to clear saved addresses",
                         BUTTON_CLEAR_HOLD_TIME_MS);
            } else {
                // Кнопка удерживается
                TickType_t hold_time = xTaskGetTickCount() - press_start_time;
                
                if (hold_time >= hold_ticks) {
                    // Кнопка удерживается достаточно долго - очищаем адреса
                    ESP_LOGI(TAG, "Button held for %d ms - clearing saved Joy-Con addresses",
                             BUTTON_CLEAR_HOLD_TIME_MS);
                    
                    // Мигаем красным для индикации
                    status_led_set_state(STATUS_LED_ERROR);
                    vTaskDelay(pdMS_TO_TICKS(LED_FEEDBACK_DELAY_MS));
                    
                    // Очищаем сохраненные адреса
                    esp_err_t ret = joycon_bt_classic_clear_saved_addresses();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "Saved Joy-Con addresses cleared successfully");
                        // Мигаем зеленым для подтверждения
                        status_led_set_state(STATUS_LED_CONNECTED);
                        vTaskDelay(pdMS_TO_TICKS(LED_FEEDBACK_DELAY_MS));
                        status_led_set_state(STATUS_LED_SCANNING);
                    } else {
                        ESP_LOGE(TAG, "Failed to clear addresses: %d", ret);
                    }
                    
                    // Ждем отпускания кнопки
                    while (gpio_get_level(BUTTON_GPIO) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    button_pressed = false;
                }
            }
        } else {
            // Кнопка отпущена
            if (button_pressed) {
                TickType_t hold_time = xTaskGetTickCount() - press_start_time;
                if (hold_time < hold_ticks) {
                    ESP_LOGD(TAG, "Button released (held for %lu ms, need %d ms)",
                             hold_time * portTICK_PERIOD_MS, BUTTON_CLEAR_HOLD_TIME_MS);
                }
                button_pressed = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS)); // Проверяем каждые 50ms
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Joy-Con to Xbox Controller starting...");
    
    // Инициализация NVS (нужно для некоторых компонентов)
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased (ret=0x%x)", ret);
        ESP_LOGI(TAG, "Erasing NVS partition...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_LOGI(TAG, "Re-initializing NVS after erase...");
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: 0x%x", ret);
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Инициализация компонентов
    ESP_LOGI(TAG, "Initializing components...");
    
    // 1. Индикация состояния
    ESP_LOGI(TAG, "Initializing status LED...");
    ret = status_led_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize status LED: %d (0x%x)", ret, ret);
        // Продолжаем работу без LED - это не критично для основной функциональности
    } else {
        ESP_LOGI(TAG, "Status LED initialized");
        status_led_set_state(STATUS_LED_SCANNING);
    }
    
    // 2. Менеджер Joy-Con (инициализирует Bluetooth Classic стек - должен быть первым!)
    ESP_LOGI(TAG, "Initializing Joy-Con manager (Bluetooth Classic stack)...");
    ESP_ERROR_CHECK(joycon_manager_init());
    ESP_LOGI(TAG, "Joy-Con manager initialized");
    
    // Ждем инициализации Bluedroid стека
    ESP_LOGI(TAG, "Waiting for Bluedroid initialization...");
    ret = joycon_bt_classic_wait_for_init(5000); // 5 секунд таймаут
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Bluedroid init timeout (ret=%d), continuing anyway", ret);
    } else {
        ESP_LOGI(TAG, "Bluedroid initialized successfully");
    }
    
    // 4. Эмулятор Xbox
    ESP_LOGI(TAG, "Initializing Xbox emulator...");
    ret = xbox_emulator_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Xbox emulator: %d (0x%x)", ret, ret);
        // Продолжаем работу - эмулятор может инициализироваться позже или использовать значения по умолчанию
    } else {
        ESP_LOGI(TAG, "Xbox emulator initialized");
    }
    
    // 5. BLE HID (Bluedroid BLE GATT сервер)
    ESP_LOGI(TAG, "Initializing BLE HID...");
    ret = ble_hid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE HID: %d (0x%x)", ret, ret);
        // Продолжаем работу - BLE HID может инициализироваться позже или устройство работает только через USB
        ESP_LOGW(TAG, "Continuing without BLE HID - device may not be discoverable via BLE");
    } else {
        ESP_LOGI(TAG, "BLE HID initialized");
        ble_hid_set_vibration_callback(on_ble_hid_vibration);
    }
    
    // Подключение к Joy-Con
    ESP_LOGI(TAG, "Connecting to Joy-Con devices...");
    ret = joycon_manager_connect_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate connection to Joy-Con devices: %d (0x%x)", ret, ret);
        // Продолжаем работу - connection_monitor_task будет пытаться переподключиться автоматически
        ESP_LOGW(TAG, "Connection will be retried automatically by connection monitor task");
    } else {
        ESP_LOGI(TAG, "Joy-Con connection initiated");
    }
    
    // Создаем мьютекс для защиты состояния переподключения перед запуском задач
    ESP_LOGI(TAG, "Creating reconnect mutex...");
    reconnect_mutex = xSemaphoreCreateMutex();
    if (reconnect_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create reconnect mutex");
        return; // Критическая ошибка - не можем продолжить без мьютекса
    }
    ESP_LOGI(TAG, "Reconnect mutex created successfully");
    
    // Создаем задачи с проверкой успешности создания
    if (xTaskCreate(main_task, "main_task", TASK_MAIN_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main_task");
        return; // Критическая ошибка - не можем продолжить без основной задачи
    }
    ESP_LOGI(TAG, "Main task created successfully");
    
    if (xTaskCreate(connection_monitor_task, "conn_monitor", TASK_MONITOR_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create connection_monitor_task");
        // Продолжаем работу, так как это не критично (можно переподключаться вручную)
    } else {
        ESP_LOGI(TAG, "Connection monitor task created successfully");
    }
    
    if (xTaskCreate(button_task, "button_task", TASK_BUTTON_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button_task");
        // Продолжаем работу, так как кнопка не критична для основной функциональности
    } else {
        ESP_LOGI(TAG, "Button task created successfully");
    }
    
    ESP_LOGI(TAG, "Application started successfully");
    // НЕ устанавливаем STATUS_LED_READY здесь - это будет сделано в main_task
    // когда оба джойкона будут подключены
}

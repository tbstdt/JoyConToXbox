/**
 * @file joycon_manager.c
 * @brief Менеджер управления Joy-Con контроллерами
 * 
 * Координирует работу с левым и правым Joy-Con:
 * - Объединение данных от обоих контроллеров в единую структуру
 * - Преобразование кнопок Joy-Con в формат Xbox контроллера
 * - Управление вибрацией (HD Rumble)
 * - Синхронизация состояния через мьютексы
 * 
 * Взаимодействие с модулями:
 * - joycon_ble.c: получение данных и управление подключением
 * - joycon_parser.c: парсинг HID отчетов от Joy-Con
 * - xbox_emulator.c: передача объединенного состояния для преобразования
 */

#include "joycon_manager.h"
#include "joycon_constants.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include "joycon/bt_classic/joycon_bt_classic.h"

static const char *TAG = "JOYCON_MANAGER";

// Состояния Joy-Con
static joycon_parsed_state_t left_state = {0};
static joycon_parsed_state_t right_state = {0};

// Предыдущие состояния кнопок для отслеживания изменений
static joycon_parsed_state_t left_state_prev = {0};
static joycon_parsed_state_t right_state_prev = {0};

// Мьютекс для потокобезопасности
static SemaphoreHandle_t state_mutex = NULL;

/**
 * @brief Логирование изменений состояния кнопок Joy-Con
 * 
 * Сравнивает текущее состояние кнопок с предыдущим и логирует только изменения
 * (нажатие/отпускание), чтобы не засорять логи.
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @param current Текущее состояние кнопок
 * @param prev Предыдущее состояние кнопок
 */
static void log_button_changes(joycon_type_t type, const joycon_parsed_state_t *current, const joycon_parsed_state_t *prev)
{
    if (!current || !prev || !current->valid) {
        return;
    }
    
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    
    if (type == JOYCON_TYPE_LEFT) {
        // Левый Joy-Con кнопки
        if (current->button_down != prev->button_down) {
            ESP_LOGI(TAG, "[%s] D-Pad DOWN: %s", type_str, current->button_down ? "PRESSED" : "RELEASED");
        }
        if (current->button_up != prev->button_up) {
            ESP_LOGI(TAG, "[%s] D-Pad UP: %s", type_str, current->button_up ? "PRESSED" : "RELEASED");
        }
        if (current->button_right != prev->button_right) {
            ESP_LOGI(TAG, "[%s] D-Pad RIGHT: %s", type_str, current->button_right ? "PRESSED" : "RELEASED");
        }
        if (current->button_left != prev->button_left) {
            ESP_LOGI(TAG, "[%s] D-Pad LEFT: %s", type_str, current->button_left ? "PRESSED" : "RELEASED");
        }
        if (current->button_sr != prev->button_sr) {
            ESP_LOGI(TAG, "[%s] SR: %s", type_str, current->button_sr ? "PRESSED" : "RELEASED");
        }
        if (current->button_sl != prev->button_sl) {
            ESP_LOGI(TAG, "[%s] SL: %s", type_str, current->button_sl ? "PRESSED" : "RELEASED");
        }
        if (current->button_l_stick != prev->button_l_stick) {
            ESP_LOGI(TAG, "[%s] LEFT STICK PRESS: %s", type_str, current->button_l_stick ? "PRESSED" : "RELEASED");
        }
        if (current->button_minus != prev->button_minus) {
            ESP_LOGI(TAG, "[%s] MINUS: %s", type_str, current->button_minus ? "PRESSED" : "RELEASED");
        }
        if (current->button_capture != prev->button_capture) {
            ESP_LOGI(TAG, "[%s] CAPTURE: %s", type_str, current->button_capture ? "PRESSED" : "RELEASED");
        }
        if (current->button_home != prev->button_home) {
            ESP_LOGI(TAG, "[%s] HOME: %s", type_str, current->button_home ? "PRESSED" : "RELEASED");
        }
        
        // Логируем изменение D-Pad значения, если оно изменилось
        if (current->dpad != prev->dpad) {
            const char *dpad_str;
            switch (current->dpad) {
                case 0: dpad_str = "UP"; break;
                case 1: dpad_str = "UP-RIGHT"; break;
                case 2: dpad_str = "RIGHT"; break;
                case 3: dpad_str = "DOWN-RIGHT"; break;
                case 4: dpad_str = "DOWN"; break;
                case 5: dpad_str = "DOWN-LEFT"; break;
                case 6: dpad_str = "LEFT"; break;
                case 7: dpad_str = "UP-LEFT"; break;
                case 8: dpad_str = "NEUTRAL"; break;
                default: dpad_str = "UNKNOWN"; break;
            }
            ESP_LOGI(TAG, "[%s] D-Pad: %s", type_str, dpad_str);
        }
    } else {
        // Правый Joy-Con кнопки
        if (current->button_y != prev->button_y) {
            ESP_LOGI(TAG, "[%s] Y: %s", type_str, current->button_y ? "PRESSED" : "RELEASED");
        }
        if (current->button_x != prev->button_x) {
            ESP_LOGI(TAG, "[%s] X: %s", type_str, current->button_x ? "PRESSED" : "RELEASED");
        }
        if (current->button_b != prev->button_b) {
            ESP_LOGI(TAG, "[%s] B: %s", type_str, current->button_b ? "PRESSED" : "RELEASED");
        }
        if (current->button_a != prev->button_a) {
            ESP_LOGI(TAG, "[%s] A: %s", type_str, current->button_a ? "PRESSED" : "RELEASED");
        }
        if (current->button_sr != prev->button_sr) {
            ESP_LOGI(TAG, "[%s] SR: %s", type_str, current->button_sr ? "PRESSED" : "RELEASED");
        }
        if (current->button_sl != prev->button_sl) {
            ESP_LOGI(TAG, "[%s] SL: %s", type_str, current->button_sl ? "PRESSED" : "RELEASED");
        }
        if (current->button_r_stick != prev->button_r_stick) {
            ESP_LOGI(TAG, "[%s] RIGHT STICK PRESS: %s", type_str, current->button_r_stick ? "PRESSED" : "RELEASED");
        }
        if (current->button_plus != prev->button_plus) {
            ESP_LOGI(TAG, "[%s] PLUS: %s", type_str, current->button_plus ? "PRESSED" : "RELEASED");
        }
        if (current->button_home != prev->button_home) {
            ESP_LOGI(TAG, "[%s] HOME: %s", type_str, current->button_home ? "PRESSED" : "RELEASED");
        }
        if (current->button_r != prev->button_r) {
            ESP_LOGI(TAG, "[%s] R: %s", type_str, current->button_r ? "PRESSED" : "RELEASED");
        }
        if (current->button_zr != prev->button_zr) {
            ESP_LOGI(TAG, "[%s] ZR: %s", type_str, current->button_zr ? "PRESSED" : "RELEASED");
        }
    }
}

// Callback для обработки данных от Bluetooth Classic
static void on_joycon_data(joycon_type_t type, const uint8_t *data, size_t len)
{
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(JOYCON_MANAGER_STATE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take mutex in on_joycon_data callback for %s Joy-Con, data lost (len=%d)",
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right", len);
        return;
    }
    
    joycon_parsed_state_t *state = (type == JOYCON_TYPE_LEFT) ? &left_state : &right_state;
    joycon_parsed_state_t *prev_state = (type == JOYCON_TYPE_LEFT) ? &left_state_prev : &right_state_prev;
    
    // Сохраняем предыдущее состояние перед парсингом
    memcpy(prev_state, state, sizeof(joycon_parsed_state_t));
    
    bool parse_result = joycon_parser_parse_report(data, len, type, state);
    if (!parse_result) {
        ESP_LOGW(TAG, "Failed to parse report from %s Joy-Con (len=%d)", 
                 type == JOYCON_TYPE_LEFT ? "Left" : "Right", len);
        // Помечаем состояние как невалидное при неудачном парсинге
        state->valid = false;
    } else if (state->valid) {
        // Логируем изменения кнопок только при успешном парсинге
        log_button_changes(type, state, prev_state);
    }
    xSemaphoreGive(state_mutex);
}

// Callback для изменения состояния подключения
static void on_joycon_state_change(joycon_type_t type, joycon_state_t bt_state)
{
    const char *type_str = (type == JOYCON_TYPE_LEFT) ? "Left" : "Right";
    const char *state_str;
    
    switch (bt_state) {
    case JOYCON_STATE_DISCONNECTED:
        state_str = "Disconnected";
        break;
    case JOYCON_STATE_SCANNING:
        state_str = "Scanning";
        break;
    case JOYCON_STATE_CONNECTING:
        state_str = "Connecting";
        break;
    case JOYCON_STATE_CONNECTED:
        state_str = "Connected";
        break;
    case JOYCON_STATE_ERROR:
        state_str = "Error";
        break;
    default:
        state_str = "Unknown";
    }
    
    ESP_LOGI(TAG, "Joy-Con %s: %s", type_str, state_str);
}

esp_err_t joycon_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing Joy-Con manager...");
    
    // Создание мьютекса
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Инициализация парсера
    joycon_parser_init();
    
    // Инициализация Bluetooth Classic для Joy-Con
    esp_err_t ret = joycon_bt_classic_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth Classic: %d", ret);
        return ret;
    }
    
    // Установка callbacks
    joycon_bt_classic_set_data_callback(on_joycon_data);
    joycon_bt_classic_set_state_callback(on_joycon_state_change);
    
    // Очистка состояний
    memset(&left_state, 0, sizeof(left_state));
    memset(&right_state, 0, sizeof(right_state));
    memset(&left_state_prev, 0, sizeof(left_state_prev));
    memset(&right_state_prev, 0, sizeof(right_state_prev));
    
    ESP_LOGI(TAG, "Joy-Con manager initialized");
    
    return ESP_OK;
}

esp_err_t joycon_manager_connect_all(void)
{
    ESP_LOGI(TAG, "Connecting to Joy-Con devices...");
    
    // Начинаем сканирование через Bluetooth Classic
    esp_err_t ret = joycon_bt_classic_start_scan();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan: %d", ret);
        return ret;
    }
    
    // Сканирование будет автоматически подключать найденные Joy-Con
    // через callback в joycon_bt_classic_scan.c
    
    return ESP_OK;
}

esp_err_t joycon_manager_get_combined_state(joycon_combined_state_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Очищаем структуру
    memset(state, 0, sizeof(joycon_combined_state_t));
    
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(JOYCON_MANAGER_STATE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Проверяем подключение
    state->left_connected = (joycon_bt_classic_get_state(JOYCON_TYPE_LEFT) == JOYCON_STATE_CONNECTED);
    state->right_connected = (joycon_bt_classic_get_state(JOYCON_TYPE_RIGHT) == JOYCON_STATE_CONNECTED);
    
    // Объединяем кнопки
    if (state->right_connected && right_state.valid) {
        // Кнопки от правого Joy-Con
        state->button_a = right_state.button_a;
        state->button_b = right_state.button_b;
        state->button_x = right_state.button_x;
        state->button_y = right_state.button_y;
        state->button_rb = right_state.button_r;
        state->button_rt = right_state.button_zr;
        state->button_start = right_state.button_plus;
        state->button_menu = right_state.button_home;
        state->button_r_stick = right_state.button_r_stick;
        
        // Правый стик
        state->stick_r_x = right_state.stick_r_x;
        state->stick_r_y = right_state.stick_r_y;
    }
    
    if (state->left_connected && left_state.valid) {
        // Кнопки от левого Joy-Con
        state->button_lb = left_state.button_sl; // Используем SL как LB
        state->button_lt = left_state.button_capture; // Используем Capture как LT
        state->button_back = left_state.button_minus;
        state->button_view = left_state.button_home;
        state->button_l_stick = left_state.button_l_stick;
        
        // D-Pad от левого Joy-Con
        state->dpad = left_state.dpad;
        
        // Левый стик
        state->stick_l_x = left_state.stick_l_x;
        state->stick_l_y = left_state.stick_l_y;
    }
    
    state->valid = (state->left_connected || state->right_connected);
    
    xSemaphoreGive(state_mutex);
    
    return ESP_OK;
}

/**
 * @brief Установить вибрацию для Joy-Con
 * 
 * ВНИМАНИЕ: Используется упрощенная реализация HD Rumble.
 * Полная реализация HD Rumble требует более сложной обработки частот (LF/HF) и амплитуд,
 * включая калибровку и преобразование частот в диапазон 40-1250 Hz.
 * Текущая реализация использует фиксированную частоту ~160 Hz для базовой вибрации.
 */
esp_err_t joycon_manager_set_vibration(uint8_t left_motor, uint8_t right_motor)
{
    esp_err_t ret = ESP_OK;
    esp_err_t ret_left = ESP_OK;
    esp_err_t ret_right = ESP_OK;
    
    // Формат команды вибрации Joy-Con (HD Rumble)
    // Joy-Con использует Output Report с subcommand для rumble
    // Формат: [0x01 (Output Report ID), JOYCON_RUMBLE_SUBCOMMAND (Subcommand), Timer, 
    //          LF_L, LF_H, HF_L, HF_H (для левого), LF_L, LF_H, HF_L, HF_H (для правого)]
    // Размер: 1 + 1 + 1 + 4 + 4 = 11 байт
    // HD Rumble использует низкую (LF) и высокую (HF) частоты с амплитудами
    uint8_t vibration_data[11] = {0};
    
    // Output Report ID и Subcommand
    vibration_data[0] = 0x01; // Output Report ID
    vibration_data[1] = JOYCON_RUMBLE_SUBCOMMAND; // Subcommand ID для rumble
    vibration_data[2] = 0x00; // Timer (0 = бесконечная вибрация)
    
    // Преобразуем силу вибрации Xbox (0-255) в формат HD Rumble Joy-Con
    // HD Rumble использует комбинацию низкой и высокой частот с амплитудами
    // Упрощенная реализация: используем фиксированную частоту и масштабируем амплитуду
    
    // Для HD Rumble: частота кодируется как LF (low frequency) и HF (high frequency)
    // LF и HF - это 16-битные значения (little-endian)
    // Амплитуда также кодируется как 16-битное значение
    
    // Левый мотор (байты 3-6: LF_L, LF_H, HF_L, HF_H)
    // HD Rumble формат: LF кодирует частоту низкой частоты, HF - высокой частоты
    // Упрощенная реализация: используем только LF с фиксированной частотой JOYCON_RUMBLE_BASE_FREQ_HZ Hz
    // Амплитуда контролируется через силу вибрации Xbox (0-255)
    // Для упрощенной вибрации достаточно установить LF и оставить HF нулевым
    if (left_motor > 0) {
        // Используем базовую частоту LF для простой вибрации
        // Для упрощенной версии не кодируем амплитуду отдельно, используем базовую частоту
        vibration_data[3] = JOYCON_RUMBLE_BASE_FREQ_VALUE; // LF Low: базовая частота
        vibration_data[4] = 0x00; // LF High: старший байт частоты
        // HF остается нулевым для упрощенной версии (используем только LF)
        vibration_data[5] = 0x00; // HF Low
        vibration_data[6] = 0x00; // HF High
    } else {
        // Выключаем вибрацию: все нули
        vibration_data[3] = 0x00;
        vibration_data[4] = 0x00;
        vibration_data[5] = 0x00;
        vibration_data[6] = 0x00;
    }
    
    // Правый мотор (байты 7-10: LF_L, LF_H, HF_L, HF_H)
    if (right_motor > 0) {
        vibration_data[7] = JOYCON_RUMBLE_BASE_FREQ_VALUE; // LF Low: базовая частота
        vibration_data[8] = 0x00; // LF High: старший байт частоты
        // HF остается нулевым для упрощенной версии
        vibration_data[9] = 0x00; // HF Low
        vibration_data[10] = 0x00; // HF High
    } else {
        // Выключаем вибрацию
        vibration_data[7] = 0x00;
        vibration_data[8] = 0x00;
        vibration_data[9] = 0x00;
        vibration_data[10] = 0x00;
    }
    
    // Отправляем вибрацию левому Joy-Con
    if (joycon_bt_classic_get_state(JOYCON_TYPE_LEFT) == JOYCON_STATE_CONNECTED) {
        ret_left = joycon_bt_classic_send_vibration(JOYCON_TYPE_LEFT, vibration_data, sizeof(vibration_data));
    }
    
    // Отправляем вибрацию правому Joy-Con
    if (joycon_bt_classic_get_state(JOYCON_TYPE_RIGHT) == JOYCON_STATE_CONNECTED) {
        ret_right = joycon_bt_classic_send_vibration(JOYCON_TYPE_RIGHT, vibration_data, sizeof(vibration_data));
    }
    
    if (ret_left != ESP_OK && ret_right != ESP_OK) {
        ret = ESP_FAIL;
    }
    
    return ret;
}

bool joycon_manager_both_connected(void)
{
    return (joycon_bt_classic_get_state(JOYCON_TYPE_LEFT) == JOYCON_STATE_CONNECTED &&
            joycon_bt_classic_get_state(JOYCON_TYPE_RIGHT) == JOYCON_STATE_CONNECTED);
}

esp_err_t joycon_manager_get_joycon_state(joycon_type_t type, joycon_parsed_state_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(JOYCON_MANAGER_STATE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (type == JOYCON_TYPE_LEFT) {
        memcpy(state, &left_state, sizeof(joycon_parsed_state_t));
    } else if (type == JOYCON_TYPE_RIGHT) {
        memcpy(state, &right_state, sizeof(joycon_parsed_state_t));
    } else {
        xSemaphoreGive(state_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

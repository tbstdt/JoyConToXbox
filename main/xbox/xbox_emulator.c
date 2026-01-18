/**
 * @file xbox_emulator.c
 * @brief Эмулятор Xbox контроллера
 * 
 * Преобразует данные от Joy-Con контроллеров в формат Xbox контроллера:
 * - Преобразование кнопок и D-Pad
 * - Преобразование стиков с применением мертвой зоны и масштабированием
 * - Формирование HID отчета Xbox (20 байт)
 * - Управление мертвой зоной стиков (сохранение в NVS)
 * 
 * Алгоритм преобразования стиков:
 * 1. Центрирование значений (Joy-Con: 0-4095, центр 2048)
 * 2. Применение мертвой зоны
 * 3. Масштабирование в диапазон Xbox (-32767 до +32767)
 * 4. Ограничение диапазона
 * 
 * Оптимизация: кэширование коэффициентов преобразования для ускорения работы.
 * 
 * Взаимодействие:
 * - Получает объединенное состояние от joycon_manager.c
 * - Формирует HID отчет для отправки через ble_hid.c
 */

#include "xbox_emulator.h"
#include "joycon_manager.h"
#include "joycon/joycon_constants.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "XBOX_EMULATOR";
static const char *NVS_NAMESPACE = "xbox_emu";

// Мертвая зона для стиков (по умолчанию 10% от максимума Xbox)
static uint16_t stick_deadzone = (JOYCON_STICK_DEADZONE_DEFAULT * XBOX_STICK_MAX) / JOYCON_STICK_MAX;

// Кэшированные значения для оптимизации преобразования стиков
// Предвычисляются при изменении мертвой зоны, чтобы избежать повторных вычислений
static uint16_t cached_deadzone_joycon = 0;  // Мертвая зона в единицах Joy-Con
static int32_t cached_stick_range = 0;       // Диапазон стика Joy-Con (4095 - 2048 = 2047)
static bool conversion_cache_valid = false;  // Флаг валидности кэша

/**
 * @brief Предвычисление коэффициентов масштабирования при изменении мертвой зоны
 * 
 * Оптимизация: вместо пересчета коэффициентов при каждом преобразовании стика,
 * предвычисляем их один раз при инициализации или изменении мертвой зоны.
 * Это ускоряет преобразование стиков в основном цикле (100 Гц).
 * 
 * Вычисляет:
 * - cached_deadzone_joycon: мертвая зона в единицах Joy-Con (0-4095)
 * - cached_stick_range: диапазон стика Joy-Con от центра до максимума (~2047)
 * 
 * Эти значения используются в convert_stick_value() для быстрого преобразования.
 */
static void update_conversion_cache(void)
{
    // Шаг 1: Преобразуем мертвую зону из диапазона Xbox в диапазон Joy-Con
    // Мертвая зона задается в единицах Xbox (0-32767), но применяется к значениям Joy-Con
    // Формула: deadzone_joycon = (deadzone_xbox * JOYCON_MAX) / XBOX_MAX
    cached_deadzone_joycon = (stick_deadzone * JOYCON_STICK_MAX) / XBOX_STICK_MAX;
    
    // Шаг 2: Предвычисляем диапазон стика Joy-Con (используется для масштабирования)
    // Диапазон = максимальное значение - центральное значение = 4095 - 2048 = 2047
    // Это значение используется в формуле масштабирования: scaled = (centered * XBOX_MAX) / stick_range
    cached_stick_range = (JOYCON_STICK_MAX - JOYCON_STICK_CENTER); // ~2047
    
    // Шаг 3: Помечаем кэш как валидный
    conversion_cache_valid = true;
    ESP_LOGD(TAG, "Conversion cache updated: deadzone_joycon=%d, stick_range=%ld", 
             cached_deadzone_joycon, cached_stick_range);
}

/**
 * @brief Преобразование значения стика Joy-Con в значение Xbox
 * 
 * Алгоритм преобразования:
 * 1. Центрирование: вычитаем центр из значения стика Joy-Con
 *    - Joy-Con диапазон: 0-4095, центр: 2048
 *    - Получаем значение от -2048 до +2047 (±2047)
 * 
 * 2. Применение мертвой зоны: если отклонение от центра меньше мертвой зоны,
 *    возвращаем 0 (стик в центре)
 * 
 * 3. Масштабирование: преобразуем из диапазона Joy-Con (±2047) в диапазон Xbox (±32767)
 *    Формула: scaled = (centered * XBOX_MAX) / JOYCON_RANGE
 *    - XBOX_MAX = 32767
 *    - JOYCON_RANGE = 4095 - 2048 = 2047
 * 
 * 4. Ограничение: проверяем выход за пределы диапазона Xbox
 * 
 * @param joycon_value Значение стика Joy-Con (0-4095)
 * @param center Центральное значение стика (обычно 2048)
 * @param deadzone Мертвая зона в единицах Joy-Con (0-4095)
 * @return Преобразованное значение стика Xbox (-32767 до 32767)
 */
static int16_t convert_stick_value(uint16_t joycon_value, uint16_t center, uint16_t deadzone)
{
    // Шаг 1: Центрируем значение (получаем отклонение от центра)
    // Joy-Con диапазон: 0-4095, центр: 2048
    // Результат: отклонение от центра [-2048, +2047]
    // Используем int32_t для предотвращения переполнения при вычитании
    int32_t centered = (int32_t)joycon_value - (int32_t)center;
    
    // Шаг 2: Применяем мертвую зону (игнорируем малые отклонения)
    // Мертвая зона устраняет дрейф стика в центральном положении
    // Используем кэшированное значение мертвой зоны для оптимизации (предвычислено в update_conversion_cache)
    uint16_t effective_deadzone = conversion_cache_valid ? cached_deadzone_joycon : deadzone;
    int32_t abs_centered = (centered < 0) ? -centered : centered;
    if (abs_centered < effective_deadzone) {
        return 0; // Стик в мертвой зоне - возвращаем центр (0 в диапазоне Xbox)
    }
    
    // Шаг 3: Масштабируем до диапазона Xbox
    // Преобразуем из диапазона Joy-Con (±2047) в диапазон Xbox (±32767)
    // Формула: scaled = (centered * XBOX_MAX) / JOYCON_RANGE
    // Используем кэшированное значение диапазона для оптимизации
    int32_t stick_range = conversion_cache_valid ? cached_stick_range : 
                         (JOYCON_STICK_MAX - JOYCON_STICK_CENTER);
    int32_t scaled = (centered * XBOX_STICK_MAX) / stick_range;
    
    // Шаг 4: Ограничиваем диапазон для безопасности
    // Защита от ошибок округления и переполнения
    // Обеспечиваем, что результат всегда в допустимом диапазоне Xbox [-32767, +32767]
    if (scaled > XBOX_STICK_MAX) scaled = XBOX_STICK_MAX;
    if (scaled < XBOX_STICK_MIN) scaled = XBOX_STICK_MIN;
    
    return (int16_t)scaled;
}


// Загрузка мертвой зоны из NVS с валидацией размера
static esp_err_t load_deadzone_from_nvs(uint16_t *deadzone)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Двухэтапное чтение: сначала проверяем размер, затем читаем данные
    size_t required_size = 0;
    ret = nvs_get_blob(nvs_handle, "deadzone", NULL, &required_size);
    if (ret == ESP_OK && required_size == sizeof(uint16_t)) {
        // Размер правильный, читаем данные
        required_size = sizeof(uint16_t);
        ret = nvs_get_blob(nvs_handle, "deadzone", deadzone, &required_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read deadzone from NVS: %d", ret);
        }
    } else if (ret == ESP_OK) {
        // Неправильный размер данных в NVS
        ESP_LOGW(TAG, "Invalid deadzone size in NVS: %d (expected %d)", required_size, sizeof(uint16_t));
        ret = ESP_ERR_INVALID_SIZE;
    }
    
    nvs_close(nvs_handle);
    return ret;
}

// Сохранение мертвой зоны в NVS
static esp_err_t save_deadzone_to_nvs(uint16_t deadzone)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", ret);
        return ret;
    }
    
    ret = nvs_set_blob(nvs_handle, "deadzone", &deadzone, sizeof(deadzone));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Deadzone saved to NVS: %d", deadzone);
    } else {
        ESP_LOGE(TAG, "Failed to save deadzone to NVS: %d", ret);
    }
    
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t xbox_emulator_init(void)
{
    ESP_LOGI(TAG, "Initializing Xbox emulator...");
    
    // Загружаем мертвую зону из NVS
    uint16_t saved_deadzone = 0;
    esp_err_t ret = load_deadzone_from_nvs(&saved_deadzone);
    if (ret == ESP_OK) {
        stick_deadzone = saved_deadzone;
        ESP_LOGI(TAG, "Deadzone loaded from NVS: %d", stick_deadzone);
    } else {
        // Используем значение по умолчанию
        ESP_LOGI(TAG, "Using default deadzone: %d", stick_deadzone);
    }
    
    // Предвычисляем коэффициенты масштабирования для оптимизации
    update_conversion_cache();
    
    ESP_LOGI(TAG, "Xbox emulator initialized");
    return ESP_OK;
}

esp_err_t xbox_emulator_convert_to_report(const joycon_combined_state_t *joycon_state,
                                          uint8_t *report)
{
    if (!joycon_state || !report) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Очищаем отчет
    memset(report, 0, XBOX_HID_REPORT_SIZE);
    
    // Заголовок отчета
    report[0] = 0x01; // Report ID
    report[1] = XBOX_HID_REPORT_SIZE - 2; // Размер данных
    
    // Кнопки (byte 2-3)
    uint8_t buttons1 = 0;
    uint8_t buttons2 = 0;
    
    if (joycon_state->button_a) buttons1 |= XBOX_BUTTON_A;
    if (joycon_state->button_b) buttons1 |= XBOX_BUTTON_B;
    if (joycon_state->button_x) buttons1 |= XBOX_BUTTON_X;
    if (joycon_state->button_y) buttons1 |= XBOX_BUTTON_Y;
    if (joycon_state->button_lb) buttons1 |= XBOX_BUTTON_LB;
    if (joycon_state->button_rb) buttons1 |= XBOX_BUTTON_RB;
    if (joycon_state->button_back) buttons1 |= XBOX_BUTTON_BACK;
    if (joycon_state->button_start) buttons1 |= XBOX_BUTTON_START;
    
    if (joycon_state->button_l_stick) buttons2 |= XBOX_BUTTON_LEFT_STICK;
    if (joycon_state->button_r_stick) buttons2 |= XBOX_BUTTON_RIGHT_STICK;
    
    report[2] = buttons1;
    report[3] = buttons2;
    
    // Триггеры (byte 4-5)
    report[4] = joycon_state->button_lt ? 255 : 0; // LT
    report[5] = joycon_state->button_rt ? 255 : 0; // RT
    
    // D-Pad (byte 3, биты 4-7)
    uint8_t dpad_value = XBOX_DPAD_NONE;
    switch (joycon_state->dpad) {
    case 0: dpad_value = XBOX_DPAD_UP; break;
    case 1: dpad_value = XBOX_DPAD_UP_RIGHT; break;
    case 2: dpad_value = XBOX_DPAD_RIGHT; break;
    case 3: dpad_value = XBOX_DPAD_DOWN_RIGHT; break;
    case 4: dpad_value = XBOX_DPAD_DOWN; break;
    case 5: dpad_value = XBOX_DPAD_DOWN_LEFT; break;
    case 6: dpad_value = XBOX_DPAD_LEFT; break;
    case 7: dpad_value = XBOX_DPAD_UP_LEFT; break;
    default: dpad_value = XBOX_DPAD_NONE; break;
    }
    report[3] |= (dpad_value << 4);
    
    // Преобразование стиков
    // Используем кэшированные значения для мертвой зоны (предвычислены при инициализации или изменении мертвой зоны)
    // Это избегает повторных вычислений при каждом преобразовании стика
    
    // Левый стик (byte 6-9)
    // deadzone передается, но может быть переопределен кэшированным значением внутри функции
    int16_t left_x = convert_stick_value(joycon_state->stick_l_x, JOYCON_STICK_CENTER, cached_deadzone_joycon);
    int16_t left_y = convert_stick_value(joycon_state->stick_l_y, JOYCON_STICK_CENTER, cached_deadzone_joycon);
    // Инвертируем Y (Joy-Con Y растет вниз, Xbox Y растет вверх)
    left_y = -left_y;
    
    report[6] = left_x & 0xFF;
    report[7] = (left_x >> 8) & 0xFF;
    report[8] = left_y & 0xFF;
    report[9] = (left_y >> 8) & 0xFF;
    
    // Правый стик (byte 10-13)
    int16_t right_x = convert_stick_value(joycon_state->stick_r_x, JOYCON_STICK_CENTER, cached_deadzone_joycon);
    int16_t right_y = convert_stick_value(joycon_state->stick_r_y, JOYCON_STICK_CENTER, cached_deadzone_joycon);
    // Инвертируем Y
    right_y = -right_y;
    
    report[10] = right_x & 0xFF;
    report[11] = (right_x >> 8) & 0xFF;
    report[12] = right_y & 0xFF;
    report[13] = (right_y >> 8) & 0xFF;
    
    // Остальные байты зарезервированы (byte 14-19)
    
    return ESP_OK;
}

esp_err_t xbox_emulator_handle_vibration(const uint8_t *report, size_t len)
{
    if (!report || len < 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Формат команды вибрации Xbox:
    // Byte 0: Report ID
    // Byte 1: Left motor (0-255)
    // Byte 2: Right motor (0-255)
    
    uint8_t left_motor = report[1];
    uint8_t right_motor = report[2];
    
    // Передаем команду вибрации в менеджер Joy-Con
    return joycon_manager_set_vibration(left_motor, right_motor);
}

void xbox_emulator_set_deadzone(uint16_t deadzone)
{
    if (deadzone > XBOX_STICK_MAX) {
        deadzone = XBOX_STICK_MAX;
    }
    stick_deadzone = deadzone;
    
    // Обновляем кэш коэффициентов масштабирования при изменении мертвой зоны
    // Это оптимизирует последующие преобразования стиков
    update_conversion_cache();
    
    // Сохраняем мертвую зону в NVS
    esp_err_t ret = save_deadzone_to_nvs(deadzone);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save deadzone to NVS: %d", ret);
    }
    
    ESP_LOGI(TAG, "Deadzone set to %d", deadzone);
}

uint16_t xbox_emulator_get_deadzone(void)
{
    return stick_deadzone;
}

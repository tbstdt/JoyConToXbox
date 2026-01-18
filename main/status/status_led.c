/**
 * @file status_led.c
 * @brief Модуль управления RGB LED индикацией состояния
 * 
 * Управляет WS2812 RGB LED для визуальной индикации состояния системы:
 * - Синий (мигание): поиск Joy-Con
 * - Желтый (мигание): подключение к Joy-Con
 * - Зеленый: оба Joy-Con подключены
 * - Красный: ошибка подключения
 * 
 * Реализация:
 * - Использует RMT модуль ESP32 для генерации точных таймингов WS2812
 * - Протокол WS2812 требует точных таймингов (0.3us, 0.6us, 0.9us)
 * - RMT позволяет генерировать эти тайминги аппаратно
 * - Формат данных: GRB (не RGB!) - особенность протокола WS2812
 * 
 * Взаимодействие:
 * - Вызывается из main.c для обновления индикации состояния
 * - Обновляется в основном цикле для мигания LED
 */

#include "status_led.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_hid/ble_hid.h"

static const char *TAG = "STATUS_LED";

// GPIO для WS2812 LED на Atom Lite (обычно GPIO 27)
#define LED_GPIO 27

// Константы для RMT и WS2812
#define WS2812_RMT_RESOLUTION_HZ      (10 * 1000 * 1000)  // Разрешение RMT: 10 MHz для точных таймингов

// Тайминги WS2812 при 10MHz (в тактах RMT)
// WS2812 требует точных таймингов для передачи данных
#define WS2812_T0H_TICKS              3    // T0H = 0.3us при 10MHz (3 такта) - время HIGH для бита 0
#define WS2812_T0L_TICKS              9    // T0L = 0.9us при 10MHz (9 тактов) - время LOW для бита 0
#define WS2812_T1H_TICKS              6    // T1H = 0.6us при 10MHz (6 тактов) - время HIGH для бита 1
#define WS2812_T1L_TICKS              6    // T1L = 0.6us при 10MHz (6 тактов) - время LOW для бита 1
#define WS2812_RESET_PULSE_TICKS      500  // Reset pulse = 50us при 10MHz (500 тактов) - низкий уровень для сброса

// RMT канал и encoder для управления WS2812
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static status_led_state_t current_state = STATUS_LED_OFF;
static uint32_t blink_counter = 0;

// Константы для мигания LED
// UPDATE_INTERVAL_MS = 10мс, поэтому 200 тиков = 2000мс = 2 секунды периода
#define STATUS_LED_BLINK_MODULO          200    // Модуль для счетчика мигания
#define STATUS_LED_BLINK_PERIOD_TICKS    200    // Период мигания (в тиках счетчика) - 200 тиков × 10мс = 2000мс (2 секунды)
#define STATUS_LED_BLINK_ON_TICKS        40     // Время включенного состояния (в тиках счетчика) - 40 тиков × 10мс = 400мс (более заметное мигание)

/**
 * @brief Структура для RMT encoder WS2812
 * 
 * WS2812 требует точных таймингов для передачи данных (протокол с одним проводом).
 * RMT (Remote Control) модуль ESP32 используется для генерации этих таймингов.
 * 
 * Структура содержит:
 * - bytes_encoder: кодирует RGB данные в RMT символы (биты 0 и 1)
 * - copy_encoder: копирует reset код (низкий уровень 50us)
 * - state: состояние кодирования (0 = данные, 1 = reset)
 * - reset_code: символ для reset импульса
 */
typedef struct {
    rmt_encoder_t base;                ///< Базовый encoder интерфейс
    rmt_encoder_handle_t bytes_encoder; ///< Encoder для RGB данных (кодирует биты)
    rmt_encoder_handle_t copy_encoder;  ///< Encoder для reset кода (копирует символ)
    int state;                         ///< Состояние кодирования (0=данные, 1=reset)
    rmt_symbol_word_t reset_code;      ///< Reset код (50us низкий уровень)
} rmt_ws2812_encoder_t;

/**
 * @brief Callback для кодирования RGB данных в RMT символы для WS2812
 * 
 * WS2812 использует протокол с одним проводом, где каждый бит кодируется
 * определенной последовательностью HIGH/LOW уровней:
 * - Бит 0: T0H=0.3us HIGH, T0L=0.9us LOW
 * - Бит 1: T1H=0.6us HIGH, T1L=0.6us LOW
 * 
 * После всех данных требуется reset импульс: 50us LOW уровня.
 * 
 * Алгоритм работы:
 * 1. Кодирует RGB данные в RMT символы (bytes_encoder)
 * 2. После завершения данных добавляет reset код (copy_encoder)
 * 3. Возвращает состояние кодирования
 * 
 * @param encoder Указатель на encoder
 * @param channel RMT канал для передачи
 * @param primary_data RGB данные (3 байта: G, R, B - WS2812 использует формат GRB!)
 * @param data_size Размер данных (обычно 3 байта)
 * @param ret_state Указатель на состояние кодирования
 * @return Количество закодированных символов
 */
static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state) {
    case 0: // Отправка RGB данных
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = 1; // Переход к reset коду
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        __attribute__((fallthrough));
    case 1: // Отправка reset кода
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812_encoder->reset_code,
                                                sizeof(ws2812_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812_encoder->state = RMT_ENCODING_RESET;
            *ret_state = RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
    }
    *ret_state = RMT_ENCODING_RESET;
    return encoded_symbols;
}

static esp_err_t rmt_del_ws2812_encoder(rmt_encoder_t *encoder)
{
    if (encoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    if (ws2812_encoder != NULL) {
        if (ws2812_encoder->bytes_encoder != NULL) {
            rmt_del_encoder(ws2812_encoder->bytes_encoder);
            ws2812_encoder->bytes_encoder = NULL;
        }
        if (ws2812_encoder->copy_encoder != NULL) {
            rmt_del_encoder(ws2812_encoder->copy_encoder);
            ws2812_encoder->copy_encoder = NULL;
        }
        free(ws2812_encoder);
        ws2812_encoder = NULL;
    }
    return ESP_OK;
}

static esp_err_t rmt_ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

/**
 * @brief Создание encoder для WS2812 LED
 * 
 * Создает составной encoder для управления WS2812 через RMT модуль ESP32.
 * WS2812 требует точных таймингов, которые RMT может генерировать аппаратно.
 * 
 * Алгоритм работы:
 * 1. Создает bytes_encoder для кодирования RGB данных в биты
 * 2. Настраивает тайминги для битов 0 и 1 (T0H, T0L, T1H, T1L)
 * 3. Создает copy_encoder для reset кода
 * 4. Настраивает reset код (50us LOW)
 * 
 * Тайминги при 10MHz RMT:
 * - T0H = 0.3us = 3 такта (бит 0, HIGH)
 * - T0L = 0.9us = 9 тактов (бит 0, LOW)
 * - T1H = 0.6us = 6 тактов (бит 1, HIGH)
 * - T1L = 0.6us = 6 тактов (бит 1, LOW)
 * - Reset = 50us = 500 тактов (LOW уровень)
 * 
 * @param ret_encoder Указатель на handle созданного encoder
 * @return esp_err_t ESP_OK при успехе, код ошибки при неудаче
 */
static esp_err_t create_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = NULL;
    esp_err_t ret = ESP_OK;

    ws2812_encoder = calloc(1, sizeof(rmt_ws2812_encoder_t));
    if (ws2812_encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Bytes encoder для RGB данных
    // WS2812 использует протокол с одним проводом, где каждый бит кодируется таймингами
    // Тайминги для WS2812 при 10MHz: T0H=0.3us=3 такта, T0L=0.9us=9 тактов, T1H=0.6us=6 тактов, T1L=0.6us=6 тактов
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,                              // Начальный уровень HIGH
            .duration0 = WS2812_T0H_TICKS,            // T0H = 0.3us при 10MHz (3 такта)
            .level1 = 0,                              // Затем уровень LOW
            .duration1 = WS2812_T0L_TICKS,            // T0L = 0.9us (9 тактов)
        },
        .bit1 = {
            .level0 = 1,                              // Начальный уровень HIGH
            .duration0 = WS2812_T1H_TICKS,            // T1H = 0.6us при 10MHz (6 тактов)
            .level1 = 0,                              // Затем уровень LOW
            .duration1 = WS2812_T1L_TICKS,            // T1L = 0.6us (6 тактов)
        },
        .flags = {
            .msb_first = 1                            // WS2812 передает биты MSB first (старший бит первым)
        }
    };
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        goto cleanup_ws2812_encoder;
    }

    // Copy encoder для reset кода
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        goto cleanup_ws2812_encoder;
    }

    // Настройка reset кода (50us низкий уровень при 10MHz = 500 тактов)
    // WS2812 требует reset импульса после передачи данных для синхронизации
    // Reset импульс: минимум 50us LOW уровня (по спецификации WS2812)
    ws2812_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,                                  // LOW уровень
        .duration0 = WS2812_RESET_PULSE_TICKS,        // 50us при 10MHz (500 тактов)
        .level1 = 0,                                  // Продолжение LOW
        .duration1 = 0,                               // Нет дополнительного времени
    };

    ws2812_encoder->base.encode = rmt_encode_ws2812;
    ws2812_encoder->base.del = rmt_del_ws2812_encoder;
    ws2812_encoder->base.reset = rmt_ws2812_encoder_reset;
    ws2812_encoder->state = RMT_ENCODING_RESET;

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;

cleanup_ws2812_encoder:
    // Единая точка освобождения ресурсов при ошибках
    if (ws2812_encoder != NULL) {
        if (ws2812_encoder->bytes_encoder != NULL) {
            rmt_del_encoder(ws2812_encoder->bytes_encoder);
            ws2812_encoder->bytes_encoder = NULL;
        }
        if (ws2812_encoder->copy_encoder != NULL) {
            rmt_del_encoder(ws2812_encoder->copy_encoder);
            ws2812_encoder->copy_encoder = NULL;
        }
        free(ws2812_encoder);
        ws2812_encoder = NULL;
    }
    return ret;
}

/**
 * @brief Преобразование RGB в формат для WS2812 и отправка
 * 
 * WS2812 использует нестандартный порядок цветов: GRB (Green, Red, Blue),
 * а не стандартный RGB. Это особенность протокола WS2812.
 * 
 * Алгоритм:
 * 1. Преобразует RGB в формат GRB
 * 2. Кодирует данные через RMT encoder
 * 3. Отправляет через RMT канал на LED
 * 
 * @param r Красный компонент (0-255)
 * @param g Зеленый компонент (0-255)
 * @param b Синий компонент (0-255)
 */
static void set_led_color_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_chan == NULL || led_encoder == NULL) {
        ESP_LOGW(TAG, "LED channel or encoder not initialized");
        return;
    }

    // WS2812 использует формат GRB (не RGB!)
    // Это особенность протокола WS2812: порядок байтов Green, Red, Blue
    uint8_t rgb_data[3] = {g, r, b};
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // Не повторять
    };

    esp_err_t ret = rmt_transmit(led_chan, led_encoder, rgb_data, sizeof(rgb_data), &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit LED data: %d", ret);
    } else {
        ESP_LOGD(TAG, "LED color set: R=%d, G=%d, B=%d", r, g, b);
    }
}

// Инициализация WS2812 через RMT
static esp_err_t init_ws2812(void)
{
    esp_err_t ret;

    // Конфигурация RMT канала для WS2812
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = WS2812_RMT_RESOLUTION_HZ, // 10MHz для точных таймингов
        .trans_queue_depth = 4,
    };
    
    ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel: %d", ret);
        return ret;
    }

    // Создание encoder для WS2812
    ret = create_ws2812_encoder(&led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %d", ret);
        rmt_del_channel(led_chan);
        led_chan = NULL;
        return ret;
    }

    // Включение RMT канала
    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %d", ret);
        rmt_del_encoder(led_encoder);
        rmt_del_channel(led_chan);
        led_chan = NULL;
        led_encoder = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "WS2812 LED initialized successfully");
    return ESP_OK;
}

esp_err_t status_led_init(void)
{
    ESP_LOGI(TAG, "Initializing status LED...");
    
    // Инициализация WS2812 через RMT
    esp_err_t ret = init_ws2812();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WS2812 LED");
        return ret;
    }
    
    current_state = STATUS_LED_OFF;
    status_led_set_state(STATUS_LED_OFF);
    
    ESP_LOGI(TAG, "Status LED initialized");
    return ESP_OK;
}

esp_err_t status_led_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing status LED...");
    
    // Выключаем LED перед освобождением ресурсов
    status_led_set_color(0, 0, 0);
    
    // Освобождаем encoder
    if (led_encoder != NULL) {
        rmt_del_encoder(led_encoder);
        led_encoder = NULL;
    }
    
    // Отключаем и удаляем канал
    if (led_chan != NULL) {
        rmt_disable(led_chan);
        rmt_del_channel(led_chan);
        led_chan = NULL;
    }
    
    current_state = STATUS_LED_OFF;
    blink_counter = 0;
    
    ESP_LOGI(TAG, "Status LED deinitialized");
    return ESP_OK;
}

esp_err_t status_led_set_state(status_led_state_t state)
{
    current_state = state;
    blink_counter = 0;
    
    switch (state) {
    case STATUS_LED_OFF:
        status_led_set_color(0, 0, 0);
        break;
        
    case STATUS_LED_SCANNING:
        // Мигание синим (будет обновляться в status_led_update)
        // Не устанавливаем цвет здесь - status_led_update() будет управлять миганием
        break;
        
    case STATUS_LED_CONNECTING:
        // Мигание желтым
        status_led_set_color(255, 255, 0);
        break;
        
    case STATUS_LED_CONNECTED:
        // Мигание зеленым (будет обновляться в status_led_update)
        // Не устанавливаем цвет здесь - status_led_update() будет управлять миганием
        break;
        
    case STATUS_LED_ERROR:
        // Постоянный красный
        status_led_set_color(255, 0, 0);
        break;
        
    case STATUS_LED_READY:
        // Постоянный синий
        status_led_set_color(0, 0, 255);
        break;
    }
    
    return ESP_OK;
}

esp_err_t status_led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    set_led_color_rgb(r, g, b);
    return ESP_OK;
}

void status_led_off(void)
{
    status_led_set_state(STATUS_LED_OFF);
}

void status_led_update(void)
{
    // Проверяем подключение BLE HID - если подключен, светодиод горит постоянно (не мигает)
    bool ble_hid_connected = ble_hid_is_ready();
    
    if (ble_hid_connected) {
        // BLE HID подключен - светодиод горит постоянно соответствующим цветом
        switch (current_state) {
        case STATUS_LED_SCANNING:
            status_led_set_color(0, 0, 255); // Синий
            break;
        case STATUS_LED_CONNECTING:
            status_led_set_color(255, 255, 0); // Желтый
            break;
        case STATUS_LED_CONNECTED:
            status_led_set_color(0, 255, 0); // Зеленый
            break;
        default:
            // Для других состояний не меняем цвет (оставляем как установлено)
            break;
        }
        return; // Не мигаем если BLE HID подключен
    }
    
    // BLE HID не подключен - мигаем
    // Обновление мигания для состояний SCANNING, CONNECTING и CONNECTED
    // Используем модульную арифметику для предотвращения переполнения
    blink_counter = (blink_counter + 1) % STATUS_LED_BLINK_MODULO;
    
    if (current_state == STATUS_LED_SCANNING) {
        // Мигание синим: период 2000мс (2 секунды), включено 400мс (более заметное мигание)
        if ((blink_counter % STATUS_LED_BLINK_PERIOD_TICKS) < STATUS_LED_BLINK_ON_TICKS) {
            status_led_set_color(0, 0, 255); // Синий
        } else {
            status_led_set_color(0, 0, 0);   // Выключено
        }
    } else if (current_state == STATUS_LED_CONNECTING) {
        // Мигание желтым: период 2000мс (2 секунды), включено 400мс (более заметное мигание)
        if ((blink_counter % STATUS_LED_BLINK_PERIOD_TICKS) < STATUS_LED_BLINK_ON_TICKS) {
            status_led_set_color(255, 255, 0); // Желтый
        } else {
            status_led_set_color(0, 0, 0);     // Выключено
        }
    } else if (current_state == STATUS_LED_CONNECTED) {
        // Мигание зеленым: период 2000мс (2 секунды), включено 400мс (более заметное мигание)
        if ((blink_counter % STATUS_LED_BLINK_PERIOD_TICKS) < STATUS_LED_BLINK_ON_TICKS) {
            status_led_set_color(0, 255, 0); // Зеленый
        } else {
            status_led_set_color(0, 0, 0);     // Выключено
        }
    }
}

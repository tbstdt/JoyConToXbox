/**
 * @file joycon_parser.c
 * @brief Парсер HID отчетов от Joy-Con контроллеров
 * 
 * Парсит сырые данные HID отчетов от Joy-Con и извлекает:
 * - Состояние кнопок (16-битная битовая маска)
 * - Координаты стиков (12-битные значения из 3 байтов)
 * - Значение D-Pad (преобразование отдельных кнопок)
 * - Данные акселерометра и гироскопа (если доступны)
 * - Уровень батареи
 * 
 * Поддерживает различные форматы отчетов:
 * - 0x21: короткий отчет (только кнопки и стики)
 * - 0x23: отчет с IMU данными
 * - 0x30: стандартный полный отчет
 * - 0x31: расширенный отчет
 * 
 * Взаимодействие:
 * - Вызывается из joycon_manager.c при получении данных от BLE
 * - Результат используется для формирования объединенного состояния
 */

#include "joycon_parser.h"
#include "joycon_constants.h"
#include "bt_classic/joycon_bt_classic.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "JOYCON_PARSER";

// Счетчики последовательных невалидных пакетов для тихой обработки
// Логируем только при превышении порога, чтобы не засорять логи мусорными данными
static uint32_t invalid_packet_count_left = 0;
static uint32_t invalid_packet_count_right = 0;
#define INVALID_PACKET_LOG_THRESHOLD 5  // Логировать только после 5 подряд невалидных пакетов


// Report ID для различных типов отчетов Joy-Con
#define JOYCON_REPORT_ID_SHORT       0x21  // Короткий отчет (стандартный input report)
#define JOYCON_REPORT_ID_WITH_IMU    0x23  // Отчет с акселерометром и гироскопом
#define JOYCON_REPORT_ID_STANDARD    0x30  // Стандартный полный отчет
#define JOYCON_REPORT_ID_EXTENDED    0x31  // Расширенный отчет

// Минимальные размеры различных Report ID Joy-Con
#define JOYCON_REPORT_MIN_SIZE_21   10  // Короткий отчет (минимальный для кнопок и стиков)
#define JOYCON_REPORT_MIN_SIZE_23   12  // Отчет с акселерометром/гироскопом
#define JOYCON_REPORT_MIN_SIZE_30   10  // Стандартный полный отчет
#define JOYCON_REPORT_MIN_SIZE_31   49  // Расширенный отчет
#define JOYCON_REPORT_MIN_SIZE_FULL 25  // Минимальный размер для полного парсинга (акселерометр/гироскоп)

// Макросы для извлечения битов из 16-битного слова кнопок
#define EXTRACT_BUTTON_BIT(buttons, bit)  (((buttons) >> (bit)) & 1)

/**
 * @brief Макросы для извлечения 12-битных координат стика из 3 байтов
 * 
 * Joy-Con кодирует координаты стиков как 12-битные значения (0-4095) в 3 байтах.
 * Формат упаковки данных оптимизирован для экономии места в BLE пакете.
 * 
 * Структура данных (offset обычно = 7, байты 7-9):
 * - Byte N (offset+0):   младшие 8 бит X стика (X[7:0])
 * - Byte N+1 (offset+1): старшие 4 бита X стика (X[11:8]) в битах [3:0] +
 *                        младшие 4 бита Y стика (Y[3:0]) в битах [7:4]
 * - Byte N+2 (offset+2): старшие 8 бит Y стика (Y[11:4])
 * 
 * Пример для X=0x3AB (12 бит):
 * - Byte 7 = 0xAB (младшие 8 бит)
 * - Byte 8[3:0] = 0x3 (старшие 4 бита)
 * - Результат: 0xAB | (0x3 << 8) = 0x3AB
 * 
 * Пример для Y=0x2CD (12 бит):
 * - Byte 8[7:4] = 0x2 (младшие 4 бита)
 * - Byte 9 = 0xCD (старшие 8 бит)
 * - Результат: (0x2 >> 4) | (0xCD << 4) = 0x2CD
 * 
 * @param data Указатель на массив данных отчета Joy-Con
 * @param offset Смещение начала данных стика (обычно 7 для байтов 7-9)
 * @return 12-битное значение координаты стика (0-4095)
 */
#define EXTRACT_STICK_X(data, offset)     ((data)[(offset) + 0] | (((data)[(offset) + 1] & 0x0F) << 8))
#define EXTRACT_STICK_Y(data, offset)     ((((data)[(offset) + 1] & 0xF0) >> 4) | ((data)[(offset) + 2] << 4))


// Joy-Con использует стандартный HID gamepad формат
// Формат отчета 0x30 (стандартный input report)
// Структура данных:
// Byte 0: Report ID (0x30)
// Byte 1: Timer (low)
// Byte 2: Timer (high)
// Byte 3-4: Battery and connection info
// Byte 5-6: Button states
// Byte 7-8: Left stick X/Y (12-bit each)
// Byte 9-10: Right stick X/Y (12-bit each)
// Byte 11-36: Vibration, accelerometer, gyro (опционально)

/**
 * @brief Парсинг HID отчета от Joy-Con
 * 
 * Парсит сырые данные HID отчета от Joy-Con контроллера и заполняет
 * структуру состояния с информацией о кнопках, стиках, D-Pad и датчиках.
 * 
 * Поддерживаемые Report ID:
 * - JOYCON_REPORT_ID_SHORT (0x21): короткий отчет (только кнопки и стики)
 * - JOYCON_REPORT_ID_WITH_IMU (0x23): отчет с акселерометром и гироскопом
 * - JOYCON_REPORT_ID_STANDARD (0x30): стандартный полный отчет
 * - JOYCON_REPORT_ID_EXTENDED (0x31): расширенный отчет
 * 
 * Алгоритм парсинга:
 * 1. Проверка валидности данных (размер, указатели)
 * 2. Определение типа отчета по Report ID
 * 3. Парсинг батареи и кнопок (общие для всех отчетов)
 * 4. Парсинг стиков (12-битные значения из 3 байтов)
 * 5. Парсинг D-Pad (преобразование отдельных кнопок)
 * 6. Парсинг акселерометра и гироскопа (если доступны)
 * 
 * @param data Указатель на сырые данные HID отчета
 * @param len Длина данных в байтах
 * @param type Тип Joy-Con (JOYCON_TYPE_LEFT или JOYCON_TYPE_RIGHT)
 * @param state Указатель на структуру состояния для заполнения
 * @return true если парсинг успешен, false при ошибке
 */
bool joycon_parser_parse_report(const uint8_t *data, size_t len, 
                                joycon_type_t type, joycon_parsed_state_t *state)
{
    if (!data || !state) {
        ESP_LOGW(TAG, "Invalid parameters: data=%p, state=%p", data, state);
        return false;
    }
    
    // Минимальный размер HID отчета Joy-Con (нужно хотя бы для кнопок и стиков)
    if (len < 10) {
        ESP_LOGW(TAG, "Report too short: len=%d", len);
        return false;
    }
    
    // Проверяем максимальный размер буфера для защиты от переполнения
    if (len > JOYCON_HID_REPORT_SIZE) {
        ESP_LOGW(TAG, "Report too large: len=%d (maximum %d)", len, JOYCON_HID_REPORT_SIZE);
        return false;
    }
    
    // Проверяем Report ID и минимальный размер отчета
    // Joy-Con поддерживает различные Report ID:
    // JOYCON_REPORT_ID_SHORT (0x21) - стандартный input report (короткий)
    // JOYCON_REPORT_ID_WITH_IMU (0x23) - стандартный input report (с акселерометром и гироскопом)
    // JOYCON_REPORT_ID_STANDARD (0x30) - стандартный input report (полный)
    // JOYCON_REPORT_ID_EXTENDED (0x31) - стандартный input report (расширенный)
    // 
    // ВАЖНО: Через Bluetooth Classic L2CAP пакеты могут иметь дополнительный байт заголовка
    // Если первый байт не является Report ID, возможно нужно проверить data[1]
    size_t data_offset = 0;  // Смещение для доступа к данным (для пропуска L2CAP заголовка)
    uint8_t report_id = data[0];
    
    // Логируем первые байты для отладки
    ESP_LOGD(TAG, "Parsing report: len=%d, first_byte=0x%02x, second_byte=0x%02x", 
             len, data[0], (len > 1) ? data[1] : 0);
    
    // Если первый байт не является Report ID, проверяем второй байт
    // (может быть, пакет через L2CAP имеет заголовок)
    if (report_id != JOYCON_REPORT_ID_SHORT && 
        report_id != JOYCON_REPORT_ID_WITH_IMU &&
        report_id != JOYCON_REPORT_ID_STANDARD &&
        report_id != JOYCON_REPORT_ID_EXTENDED &&
        len > 1) {
        // Проверяем, может быть Report ID во втором байте
        uint8_t second_byte = data[1];
        if (second_byte == JOYCON_REPORT_ID_SHORT || 
            second_byte == JOYCON_REPORT_ID_WITH_IMU ||
            second_byte == JOYCON_REPORT_ID_STANDARD ||
            second_byte == JOYCON_REPORT_ID_EXTENDED) {
            ESP_LOGD(TAG, "Report ID found at offset 1: 0x%02x (first byte was 0x%02x), using offset", 
                     second_byte, report_id);
            // Report ID во втором байте - используем смещение
            data_offset = 1;
            report_id = data[1];
        } else if (len == 36) {
            // Специальная обработка для пакетов длиной 36 байт (возможен стандартный отчет 0x30)
            // Попробуем обработать как стандартный отчет 0x30, пропуская первые байты
            // 36 байт может быть стандартным отчетом 0x30 (10+ байт данных) с дополнительным заголовком
            // Проверяем, есть ли структура стандартного отчета где-то в пакете
            for (size_t test_offset = 0; test_offset <= 2 && (test_offset + 10) <= len; test_offset++) {
                uint8_t test_byte = data[test_offset];
                if (test_byte == JOYCON_REPORT_ID_STANDARD) {
                    ESP_LOGW(TAG, "Found Report ID 0x30 at offset %d in 36-byte packet, using offset", test_offset);
                    data_offset = test_offset;
                    report_id = test_byte;
                    break;
                }
            }
            // Если Report ID не найден, но пакет 36 байт - возможно это стандартный отчет без ID
            // Пробуем обработать как стандартный отчет 0x30 с offset=0 (если структура похожа)
            if (report_id != JOYCON_REPORT_ID_STANDARD && len >= 10) {
                // Проверяем структуру: байт 3 должен содержать батарею (4 бита в верхней части)
                uint8_t battery_raw = (data[3] >> 4) & 0x0F;
                if (battery_raw <= 15) {
                    // Похоже на стандартный отчет Joy-Con - обрабатываем как 0x30
                    ESP_LOGW(TAG, "36-byte packet without Report ID, treating as 0x30 (battery=%d)", battery_raw);
                    report_id = JOYCON_REPORT_ID_STANDARD;
                    data_offset = 0;
                } else {
                    // Неизвестные байты - возможно мусорные данные или неполный пакет
                    if (type == JOYCON_TYPE_LEFT) {
                        if (invalid_packet_count_left >= INVALID_PACKET_LOG_THRESHOLD) {
                            // Предотвращаем переполнение - сбрасываем перед инкрементом
                            invalid_packet_count_left = 0;
                        }
                        invalid_packet_count_left++;
                        if (invalid_packet_count_left >= INVALID_PACKET_LOG_THRESHOLD) {
                            ESP_LOGW(TAG, "Unknown 36-byte packet: first=0x%02x, second=0x%02x, battery_raw=%d (count=%lu)",
                                     report_id, second_byte, battery_raw, (unsigned long)invalid_packet_count_left);
                            invalid_packet_count_left = 0; // Сброс после логирования для предотвращения переполнения
                        }
                    } else {
                        if (invalid_packet_count_right >= INVALID_PACKET_LOG_THRESHOLD) {
                            // Предотвращаем переполнение - сбрасываем перед инкрементом
                            invalid_packet_count_right = 0;
                        }
                        invalid_packet_count_right++;
                        if (invalid_packet_count_right >= INVALID_PACKET_LOG_THRESHOLD) {
                            ESP_LOGW(TAG, "Unknown 36-byte packet: first=0x%02x, second=0x%02x, battery_raw=%d (count=%lu)",
                                     report_id, second_byte, battery_raw, (unsigned long)invalid_packet_count_right);
                            invalid_packet_count_right = 0; // Сброс после логирования для предотвращения переполнения
                        }
                    }
                }
            }
        } else {
            // Неизвестные байты - возможно мусорные данные или неполный пакет
            // Увеличиваем счетчик и логируем только при превышении порога
            if (type == JOYCON_TYPE_LEFT) {
                if (invalid_packet_count_left >= INVALID_PACKET_LOG_THRESHOLD) {
                    // Предотвращаем переполнение - сбрасываем перед инкрементом
                    invalid_packet_count_left = 0;
                }
                invalid_packet_count_left++;
                if (invalid_packet_count_left >= INVALID_PACKET_LOG_THRESHOLD) {
                    ESP_LOGW(TAG, "Unknown first byte: 0x%02x, second byte: 0x%02x (count=%lu)",
                             report_id, second_byte, (unsigned long)invalid_packet_count_left);
                    invalid_packet_count_left = 0; // Сброс после логирования для предотвращения переполнения
                }
            } else {
                if (invalid_packet_count_right >= INVALID_PACKET_LOG_THRESHOLD) {
                    // Предотвращаем переполнение - сбрасываем перед инкрементом
                    invalid_packet_count_right = 0;
                }
                invalid_packet_count_right++;
                if (invalid_packet_count_right >= INVALID_PACKET_LOG_THRESHOLD) {
                    ESP_LOGW(TAG, "Unknown first byte: 0x%02x, second byte: 0x%02x (count=%lu)",
                             report_id, second_byte, (unsigned long)invalid_packet_count_right);
                    invalid_packet_count_right = 0; // Сброс после логирования для предотвращения переполнения
                }
            }
        }
    }
    
    uint16_t min_size = 0;
    bool supported_report = false;
    
    switch (report_id) {
    case JOYCON_REPORT_ID_SHORT: // Короткий отчет
        min_size = JOYCON_REPORT_MIN_SIZE_21;
        supported_report = true;
        break;
    case JOYCON_REPORT_ID_WITH_IMU: // Отчет с акселерометром/гироскопом
        min_size = JOYCON_REPORT_MIN_SIZE_23;
        supported_report = true;
        break;
    case JOYCON_REPORT_ID_STANDARD: // Стандартный полный отчет
        min_size = JOYCON_REPORT_MIN_SIZE_30;
        supported_report = true;
        break;
    case JOYCON_REPORT_ID_EXTENDED: // Расширенный отчет
        min_size = JOYCON_REPORT_MIN_SIZE_31;
        supported_report = true;
        break;
    default:
        // Неизвестный Report ID, пропускаем (не ошибка)
        // Увеличиваем счетчик для тихой обработки с защитой от переполнения
        if (type == JOYCON_TYPE_LEFT) {
            if (invalid_packet_count_left >= INVALID_PACKET_LOG_THRESHOLD) {
                invalid_packet_count_left = 0; // Предотвращаем переполнение
            }
            invalid_packet_count_left++;
        } else {
            if (invalid_packet_count_right >= INVALID_PACKET_LOG_THRESHOLD) {
                invalid_packet_count_right = 0; // Предотвращаем переполнение
            }
            invalid_packet_count_right++;
        }
        // Логируем только на уровне DEBUG, не WARNING
        ESP_LOGD(TAG, "Skipping unsupported report ID: 0x%02x", report_id);
        return false;
    }
    
    // При успешном распознавании Report ID сбрасываем счетчик невалидных пакетов
    if (type == JOYCON_TYPE_LEFT) {
        invalid_packet_count_left = 0;
    } else {
        invalid_packet_count_right = 0;
    }
    
    if (!supported_report) {
        return false;
    }
    
    // Проверяем минимальный размер для данного Report ID (с учетом смещения)
    if ((len - data_offset) < min_size) {
        ESP_LOGW(TAG, "Report too short for ID 0x%02x: len=%d, offset=%d (minimum %d)", 
                 report_id, len, data_offset, min_size);
        return false;
    }
    
    // Валидация батареи (должна быть в диапазоне 0-15)
    // Баттерия находится в байте 3 отчета (не считая возможный L2CAP заголовок)
    uint8_t battery_raw = (data[data_offset + 3] >> 4) & 0x0F;
    if (battery_raw > 15) {
        ESP_LOGW(TAG, "Invalid battery value: %d", battery_raw);
        return false;
    }
    
    // Очищаем структуру
    memset(state, 0, sizeof(joycon_parsed_state_t));
    
    // Парсинг батареи (byte 3, биты 4-7) - с учетом смещения
    state->battery = (data[data_offset + 3] >> 4) & 0x0F;
    state->battery = (state->battery * 100) / 15; // Преобразуем в проценты
    
    // Парсинг кнопок (bytes 5-6) - с учетом смещения
    uint16_t buttons = (data[data_offset + 6] << 8) | data[data_offset + 5];
    
    if (type == JOYCON_TYPE_LEFT) {
        // Левый Joy-Con - парсинг кнопок из 16-битного слова (bytes 5-6)
        // Биты кнопок следуют в порядке: bit 0-15 (little-endian)
        state->button_down = EXTRACT_BUTTON_BIT(buttons, 0);
        state->button_up = EXTRACT_BUTTON_BIT(buttons, 1);
        state->button_right = EXTRACT_BUTTON_BIT(buttons, 2);
        state->button_left = EXTRACT_BUTTON_BIT(buttons, 3);
        state->button_sr = EXTRACT_BUTTON_BIT(buttons, 4);
        state->button_sl = EXTRACT_BUTTON_BIT(buttons, 5);
        state->button_l_stick = EXTRACT_BUTTON_BIT(buttons, 6);
        state->button_minus = EXTRACT_BUTTON_BIT(buttons, 7);
        state->button_capture = EXTRACT_BUTTON_BIT(buttons, 8);
        state->button_home = EXTRACT_BUTTON_BIT(buttons, 9);
        
        // D-Pad: преобразование отдельных кнопок направления в значение D-Pad
        // Joy-Con использует 4 отдельные кнопки для направлений (Up, Down, Left, Right)
        // Преобразуем их в единое значение D-Pad для совместимости с Xbox контроллером
        // 
        // Значения D-Pad:
        // 0 = Up
        // 1 = Up-Right
        // 2 = Right
        // 3 = Down-Right
        // 4 = Down
        // 5 = Down-Left
        // 6 = Left
        // 7 = Up-Left
        // 8 = Neutral (ни одно направление не нажато)
        //
        // Приоритет проверки: сначала одиночные направления, затем диагонали
        if (state->button_up && !state->button_down && !state->button_left && !state->button_right) {
            state->dpad = 0; // Up
        } else if (state->button_up && state->button_right) {
            state->dpad = 1; // Up-Right
        } else if (state->button_right && !state->button_up && !state->button_down) {
            state->dpad = 2; // Right
        } else if (state->button_down && state->button_right) {
            state->dpad = 3; // Down-Right
        } else if (state->button_down && !state->button_up && !state->button_left && !state->button_right) {
            state->dpad = 4; // Down
        } else if (state->button_down && state->button_left) {
            state->dpad = 5; // Down-Left
        } else if (state->button_left && !state->button_up && !state->button_down) {
            state->dpad = 6; // Left
        } else if (state->button_up && state->button_left) {
            state->dpad = 7; // Up-Left
        } else {
            state->dpad = 8; // Neutral
        }
        
        // Левый стик (bytes 7-9) - 12-битные значения для каждой оси
        // Формат Joy-Con кодирования стиков:
        // - Byte 7: младшие 8 бит X стика (X[7:0])
        // - Byte 8[3:0]: старшие 4 бита X стика (X[11:8])
        // - Byte 8[7:4]: младшие 4 бита Y стика (Y[3:0])
        // - Byte 9: старшие 8 бит Y стика (Y[11:4])
        if ((len - data_offset) >= 10) {
            uint16_t stick_x = EXTRACT_STICK_X(data, data_offset + 7);
            uint16_t stick_y = EXTRACT_STICK_Y(data, data_offset + 7);
            
            // Валидация значений стика (должны быть в диапазоне 0-4095)
            if (stick_x <= JOYCON_STICK_MAX && stick_y <= JOYCON_STICK_MAX) {
                state->stick_l_x = stick_x;
                state->stick_l_y = stick_y;
            } else {
                ESP_LOGW(TAG, "Invalid left stick values: X=%d, Y=%d", stick_x, stick_y);
                // Используем центр по умолчанию
                state->stick_l_x = JOYCON_STICK_CENTER;
                state->stick_l_y = JOYCON_STICK_CENTER;
            }
        }
    } else if (type == JOYCON_TYPE_RIGHT) {
        // Правый Joy-Con - парсинг кнопок из 16-битного слова (bytes 5-6)
        state->button_y = EXTRACT_BUTTON_BIT(buttons, 0);
        state->button_x = EXTRACT_BUTTON_BIT(buttons, 1);
        state->button_b = EXTRACT_BUTTON_BIT(buttons, 2);
        state->button_a = EXTRACT_BUTTON_BIT(buttons, 3);
        state->button_sr = EXTRACT_BUTTON_BIT(buttons, 4);
        state->button_sl = EXTRACT_BUTTON_BIT(buttons, 5);
        state->button_r_stick = EXTRACT_BUTTON_BIT(buttons, 6);
        state->button_plus = EXTRACT_BUTTON_BIT(buttons, 7);
        state->button_home = EXTRACT_BUTTON_BIT(buttons, 8);
        state->button_r = EXTRACT_BUTTON_BIT(buttons, 9);
        state->button_zr = EXTRACT_BUTTON_BIT(buttons, 10);
        
        // Правый стик (bytes 7-9) - формат идентичен левому стику
        if ((len - data_offset) >= 10) {
            uint16_t stick_x = EXTRACT_STICK_X(data, data_offset + 7);
            uint16_t stick_y = EXTRACT_STICK_Y(data, data_offset + 7);
            
            // Валидация значений стика
            if (stick_x <= JOYCON_STICK_MAX && stick_y <= JOYCON_STICK_MAX) {
                state->stick_r_x = stick_x;
                state->stick_r_y = stick_y;
            } else {
                ESP_LOGW(TAG, "Invalid right stick values: X=%d, Y=%d", stick_x, stick_y);
                // Используем центр по умолчанию
                state->stick_r_x = JOYCON_STICK_CENTER;
                state->stick_r_y = JOYCON_STICK_CENTER;
            }
        }
    }
    
    // Парсинг акселерометра и гироскопа (если доступны, bytes 13-24)
    // Проверяем размер перед доступом к данным акселерометра и гироскопа
    if ((len - data_offset) >= JOYCON_REPORT_MIN_SIZE_FULL) {
        // Акселерометр (16-bit, little-endian) - с учетом смещения
        state->accel_x = (data[data_offset + 13] | (data[data_offset + 14] << 8));
        state->accel_y = (data[data_offset + 15] | (data[data_offset + 16] << 8));
        state->accel_z = (data[data_offset + 17] | (data[data_offset + 18] << 8));
        
        // Гироскоп (16-bit, little-endian) - с учетом смещения
        state->gyro_x = (data[data_offset + 19] | (data[data_offset + 20] << 8));
        state->gyro_y = (data[data_offset + 21] | (data[data_offset + 22] << 8));
        state->gyro_z = (data[data_offset + 23] | (data[data_offset + 24] << 8));
    }
    
    state->valid = true;
    
    return true;
}

void joycon_parser_init(void)
{
    ESP_LOGI(TAG, "Joy-Con parser initialized");
}

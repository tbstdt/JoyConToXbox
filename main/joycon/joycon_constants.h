/**
 * @file joycon_constants.h
 * @brief Константы для работы с Joy-Con контроллерами
 * 
 * Содержит все константы, используемые в проекте:
 * - Диапазоны значений стиков Joy-Con и Xbox
 * - Параметры задач FreeRTOS
 * - Параметры частоты обновления
 * - Параметры переподключения
 * - Параметры мониторинга подключения
 * - Константы вибрации
 */

#ifndef JOYCON_CONSTANTS_H
#define JOYCON_CONSTANTS_H

#include <stdint.h>

// Константы для стиков Joy-Con
#define JOYCON_STICK_MIN             0       // Минимальное значение стика
#define JOYCON_STICK_MAX             4095    // Максимальное значение стика
#define JOYCON_STICK_CENTER          2048    // Центральное значение стика
#define JOYCON_STICK_DEADZONE_DEFAULT 410    // Мертвая зона по умолчанию (~10% от 4095)

// Константы для стиков Xbox
#define XBOX_STICK_MIN               -32767  // Минимальное значение стика Xbox
#define XBOX_STICK_MAX               32767   // Максимальное значение стика Xbox

// Константы для задач FreeRTOS
#define TASK_MAIN_STACK_SIZE         4096    // Размер стека основной задачи
#define TASK_MONITOR_STACK_SIZE      2048    // Размер стека задачи мониторинга
#define TASK_BUTTON_STACK_SIZE       2048    // Размер стека задачи обработки кнопки
#define MAIN_TASK_PRIORITY           5       // Приоритет основной задачи
#define MONITOR_TASK_PRIORITY        3       // Приоритет задачи мониторинга
#define BUTTON_TASK_PRIORITY         2       // Приоритет задачи обработки кнопки

// Константы для частоты обновления
#define UPDATE_RATE_HZ               100     // Частота обновления (Hz)
#define UPDATE_INTERVAL_MS           (1000 / UPDATE_RATE_HZ)  // Интервал обновления (мс)

// Константы для переподключения
#define RECONNECT_MAX_ATTEMPTS       5       // Максимальное количество попыток для экспоненциальной задержки
#define RECONNECT_BASE_DELAY_MS      1000    // Базовая задержка переподключения (мс)
#define RECONNECT_MAX_SAFE_SHIFT     16      // Максимальный безопасный сдвиг для uint32_t (2^16 = 65536)
#define RECONNECT_MAX_DELAY_MS       60000   // Максимальная задержка переподключения (мс, 1 минута)

// Константы для мониторинга подключения
#define CONNECTION_MONITOR_INTERVAL_MS 5000  // Интервал проверки состояния подключения при подключенных обоих Joy-Con (мс)
#define CONNECTION_CHECK_INTERVAL_MS    1000  // Интервал проверки состояния подключения при отключенных Joy-Con (мс)
#define CONNECTION_MUTEX_TIMEOUT_MS      100   // Таймаут захвата мьютекса переподключения (мс)
#define JOYCON_MANAGER_STATE_MUTEX_TIMEOUT_MS 200  // Таймаут захвата мьютекса состояния Joy-Con (мс) - увеличен для критического callback

// Константы для обработки кнопки
#define BUTTON_GPIO                   39      // GPIO для кнопки Atom Lite (обычно GPIO 39)
#define BUTTON_CLEAR_HOLD_TIME_MS     3000    // Время удержания кнопки для очистки сохраненных адресов (мс)
#define BUTTON_POLL_INTERVAL_MS       50      // Интервал опроса состояния кнопки (мс)

// Константы для BLE синхронизации
#define BLE_SYNC_TIMEOUT_MS           5000    // Таймаут ожидания синхронизации BLE стека (мс)

// Константы для GATT Discovery
#define GATT_DISCOVERY_TIMEOUT_MS     10000   // Таймаут для процесса GATT discovery (мс)

// Константы для CCCD fallback
#define CCCD_FALLBACK_ENABLED         1       // Включить fallback для CCCD handle (1 = включено, 0 = выключено)
#define CCCD_FALLBACK_OFFSET          1       // Смещение от val_handle для fallback CCCD handle (обычно +1)

// Константы для вибрации Joy-Con (HD Rumble)
#define JOYCON_RUMBLE_BASE_FREQ_HZ      160     // Базовая частота вибрации (Hz)
#define JOYCON_RUMBLE_BASE_FREQ_VALUE   0xA0    // Значение базовой частоты в формате Joy-Con (0xA0 = 160 Hz)
#define JOYCON_RUMBLE_SUBCOMMAND        0x10    // Subcommand ID для команды rumble в Output Report

// Константы для MAC OUI Joy-Con (Organizationally Unique Identifier)
// Nintendo Joy-Con использует следующие OUI для MAC-адресов:
// Первый вариант: 00:09:9F:XX:XX:XX
#define JOYCON_OUI_1_BYTE_5             0x00    // Byte 5 первого OUI
#define JOYCON_OUI_1_BYTE_4             0x9F    // Byte 4 первого OUI
// Второй вариант: 98:B6:XX:XX:XX:XX
#define JOYCON_OUI_2_BYTE_5             0x98    // Byte 5 второго OUI
#define JOYCON_OUI_2_BYTE_4             0xB6    // Byte 4 второго OUI

// Константы для Bluetooth
#define BT_DEVICE_NAME_BUFFER_SIZE      32      // Размер буфера для имени Bluetooth устройства

#endif // JOYCON_CONSTANTS_H

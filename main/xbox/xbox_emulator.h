/**
 * @file xbox_emulator.h
 * @brief Заголовочный файл эмулятора Xbox контроллера
 * 
 * Определяет API для преобразования данных Joy-Con в формат Xbox контроллера.
 */

#ifndef XBOX_EMULATOR_H
#define XBOX_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "joycon_manager.h"
#include "xbox_hid_descriptor.h"

/**
 * @brief Инициализация эмулятора Xbox
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t xbox_emulator_init(void);

/**
 * @brief Преобразовать состояние Joy-Con в HID отчет Xbox
 * 
 * @param joycon_state Объединенное состояние Joy-Con
 * @param report Буфер для HID отчета (XBOX_HID_REPORT_SIZE байт)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t xbox_emulator_convert_to_report(const joycon_combined_state_t *joycon_state,
                                          uint8_t *report);

/**
 * @brief Обработать команду вибрации от Xbox
 * 
 * @param report HID отчет с командой вибрации
 * @param len Длина отчета
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t xbox_emulator_handle_vibration(const uint8_t *report, size_t len);

/**
 * @brief Установить мертвую зону для стиков
 * 
 * @param deadzone Значение мертвой зоны (0-32767)
 */
void xbox_emulator_set_deadzone(uint16_t deadzone);

/**
 * @brief Получить текущую мертвую зону
 * 
 * @return uint16_t Значение мертвой зоны
 */
uint16_t xbox_emulator_get_deadzone(void);

#endif // XBOX_EMULATOR_H

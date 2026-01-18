/**
 * @file joycon_manager.h
 * @brief Заголовочный файл менеджера Joy-Con
 * 
 * Определяет API для управления двумя Joy-Con контроллерами:
 * - Объединение состояния от левого и правого Joy-Con
 * - Преобразование в формат, совместимый с Xbox контроллером
 * - Управление вибрацией
 */

#ifndef JOYCON_MANAGER_H
#define JOYCON_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "joycon_parser.h"
#include "joycon_types.h"

/**
 * @brief Объединенное состояние от двух Joy-Con контроллеров
 * 
 * Объединяет данные от левого и правого Joy-Con в единую структуру,
 * которая затем преобразуется в формат Xbox контроллера.
 * Заполняется функцией joycon_manager_get_combined_state().
 */
typedef struct {
    // Кнопки Xbox контроллера (объединенные из обоих Joy-Con)
    bool button_a;          ///< Кнопка A (правый Joy-Con, button_a)
    bool button_b;          ///< Кнопка B (правый Joy-Con, button_b)
    bool button_x;          ///< Кнопка X (правый Joy-Con, button_x)
    bool button_y;          ///< Кнопка Y (правый Joy-Con, button_y)
    bool button_lb;         ///< Левая кнопка бампера (левый Joy-Con, button_sl)
    bool button_rb;         ///< Правая кнопка бампера (правый Joy-Con, button_r)
    bool button_lt;         ///< Левый триггер (левый Joy-Con, button_capture)
    bool button_rt;         ///< Правый триггер (правый Joy-Con, button_zr)
    bool button_back;       ///< Кнопка Back/View (левый Joy-Con, button_minus)
    bool button_start;      ///< Кнопка Start/Menu (правый Joy-Con, button_plus)
    bool button_view;       ///< Кнопка View (левый Joy-Con, button_home)
    bool button_menu;       ///< Кнопка Menu (правый Joy-Con, button_home)
    bool button_l_stick;    ///< Нажатие левого стика (левый Joy-Con)
    bool button_r_stick;    ///< Нажатие правого стика (правый Joy-Con)
    
    // D-Pad: значение от левого Joy-Con (0-8, где 8 = нейтральное положение)
    uint8_t dpad;
    
    // Аналоговые стики: значения от соответствующих Joy-Con (0-4095, центр ~2048)
    uint16_t stick_l_x;     ///< Левый стик, ось X (от левого Joy-Con)
    uint16_t stick_l_y;     ///< Левый стик, ось Y (от левого Joy-Con)
    uint16_t stick_r_x;     ///< Правый стик, ось X (от правого Joy-Con)
    uint16_t stick_r_y;     ///< Правый стик, ось Y (от правого Joy-Con)
    
    // Состояние подключения Joy-Con контроллеров
    bool left_connected;    ///< Левый Joy-Con подключен
    bool right_connected;   ///< Правый Joy-Con подключен
    
    // Флаг валидности данных: true если хотя бы один Joy-Con подключен и данные валидны
    bool valid;
} joycon_combined_state_t;

/**
 * @brief Инициализация менеджера Joy-Con
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_manager_init(void);

/**
 * @brief Подключить оба Joy-Con
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_manager_connect_all(void);

/**
 * @brief Получить объединенное состояние от обоих Joy-Con
 * 
 * @param state Указатель на структуру для заполнения
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_manager_get_combined_state(joycon_combined_state_t *state);

/**
 * @brief Установить вибрацию для Joy-Con
 * 
 * @param left_motor Сила вибрации левого мотора (0-255)
 * @param right_motor Сила вибрации правого мотора (0-255)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_manager_set_vibration(uint8_t left_motor, uint8_t right_motor);

/**
 * @brief Проверить, подключены ли оба Joy-Con
 * 
 * @return true если оба подключены
 */
bool joycon_manager_both_connected(void);

/**
 * @brief Получить состояние отдельного Joy-Con
 * 
 * @param type Тип Joy-Con
 * @param state Указатель на структуру состояния
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_manager_get_joycon_state(joycon_type_t type, joycon_parsed_state_t *state);

#endif // JOYCON_MANAGER_H

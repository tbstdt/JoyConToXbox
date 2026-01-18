/**
 * @file joycon_parser.h
 * @brief Заголовочный файл парсера Joy-Con
 * 
 * Определяет структуры данных и API для парсинга HID отчетов от Joy-Con.
 */

#ifndef JOYCON_PARSER_H
#define JOYCON_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "joycon_types.h"

/**
 * @brief Структура состояния Joy-Con после парсинга HID отчета
 * 
 * Содержит распарсенные данные от одного Joy-Con контроллера:
 * кнопки, стики, D-Pad, датчики движения и уровень батареи.
 * Заполняется функцией joycon_parser_parse_report().
 */
typedef struct {
    // Кнопки Joy-Con (битовая маска из HID отчета)
    bool button_y;          ///< Кнопка Y (правый Joy-Con)
    bool button_x;          ///< Кнопка X (правый Joy-Con)
    bool button_b;          ///< Кнопка B (правый Joy-Con)
    bool button_a;          ///< Кнопка A (правый Joy-Con)
    bool button_sr;         ///< Кнопка SR (боковая, правая)
    bool button_sl;         ///< Кнопка SL (боковая, левая)
    bool button_r;          ///< Кнопка R (правый Joy-Con, верхняя)
    bool button_zr;         ///< Кнопка ZR (правый Joy-Con, триггер)
    bool button_minus;      ///< Кнопка - (левый Joy-Con)
    bool button_plus;       ///< Кнопка + (правый Joy-Con)
    bool button_r_stick;    ///< Нажатие правого стика
    bool button_l_stick;    ///< Нажатие левого стика
    bool button_home;       ///< Кнопка Home
    bool button_capture;    ///< Кнопка Capture (левый Joy-Con)
    bool button_down;       ///< Кнопка направления вниз (левый Joy-Con, D-Pad)
    bool button_up;         ///< Кнопка направления вверх (левый Joy-Con, D-Pad)
    bool button_right;      ///< Кнопка направления вправо (левый Joy-Con, D-Pad)
    bool button_left;       ///< Кнопка направления влево (левый Joy-Con, D-Pad)
    
    // D-Pad: объединенное значение направлений (0-8, где 8 = нейтральное положение)
    // 0=Up, 1=Up-Right, 2=Right, 3=Down-Right, 4=Down, 5=Down-Left, 6=Left, 7=Up-Left, 8=Neutral
    uint8_t dpad;
    
    // Аналоговые стики: 12-битные значения (0-4095, центр ~2048)
    uint16_t stick_l_x;     ///< Левый стик, ось X (0-4095)
    uint16_t stick_l_y;     ///< Левый стик, ось Y (0-4095)
    uint16_t stick_r_x;     ///< Правый стик, ось X (0-4095)
    uint16_t stick_r_y;     ///< Правый стик, ось Y (0-4095)
    
    // Акселерометр: значения ускорения по осям (опционально, для калибровки)
    int16_t accel_x;        ///< Ускорение по оси X
    int16_t accel_y;        ///< Ускорение по оси Y
    int16_t accel_z;        ///< Ускорение по оси Z
    
    // Гироскоп: значения угловой скорости по осям (опционально)
    int16_t gyro_x;         ///< Угловая скорость по оси X
    int16_t gyro_y;         ///< Угловая скорость по оси Y
    int16_t gyro_z;         ///< Угловая скорость по оси Z
    
    // Батарея: уровень заряда в процентах (0-100%)
    uint8_t battery;
    
    // Флаг валидности данных: true если данные успешно распарсены
    bool valid;
} joycon_parsed_state_t;

/**
 * @brief Парсинг HID отчета от Joy-Con
 * 
 * @param data Указатель на сырые данные HID отчета
 * @param len Длина данных
 * @param type Тип Joy-Con (левый или правый)
 * @param state Указатель на структуру состояния для заполнения
 * @return true если парсинг успешен
 */
bool joycon_parser_parse_report(const uint8_t *data, size_t len, 
                                joycon_type_t type, joycon_parsed_state_t *state);

/**
 * @brief Инициализация парсера
 */
void joycon_parser_init(void);

#endif // JOYCON_PARSER_H

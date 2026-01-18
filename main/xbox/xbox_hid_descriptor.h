/**
 * @file xbox_hid_descriptor.h
 * @brief Определения для HID дескриптора Xbox контроллера
 * 
 * Содержит константы и структуры для работы с HID отчетами Xbox контроллера:
 * - Размер отчета
 * - Битовые маски кнопок
 * - Значения D-Pad
 * - Структура отчета
 */

#ifndef XBOX_HID_DESCRIPTOR_H
#define XBOX_HID_DESCRIPTOR_H

#include <stdint.h>

// HID дескриптор для Xbox контроллера
// Формат: стандартный HID gamepad с поддержкой вибрации

// Размер HID отчета
#define XBOX_HID_REPORT_SIZE 20

// Структура HID отчета Xbox контроллера
typedef struct {
    uint8_t report_id;      // 0x01 для input report
    uint8_t size;            // Размер отчета (18 байт данных + 2 байта заголовка)
    uint8_t buttons1;        // Кнопки: A, B, X, Y, LB, RB, Back, Start
    uint8_t buttons2;       // Кнопки: Left Stick, Right Stick, D-Pad
    uint8_t left_trigger;   // LT (0-255)
    uint8_t right_trigger;  // RT (0-255)
    int16_t left_stick_x;   // Левый стик X (-32768 до 32767)
    int16_t left_stick_y;   // Левый стик Y (-32768 до 32767)
    int16_t right_stick_x;  // Правый стик X (-32768 до 32767)
    int16_t right_stick_y;  // Правый стик Y (-32768 до 32767)
    uint8_t reserved[6];    // Зарезервировано
} __attribute__((packed)) xbox_hid_report_t;

// Битовая маска кнопок (buttons1)
#define XBOX_BUTTON_A      (1 << 0)
#define XBOX_BUTTON_B      (1 << 1)
#define XBOX_BUTTON_X      (1 << 2)
#define XBOX_BUTTON_Y      (1 << 3)
#define XBOX_BUTTON_LB     (1 << 4)
#define XBOX_BUTTON_RB     (1 << 5)
#define XBOX_BUTTON_BACK   (1 << 6)
#define XBOX_BUTTON_START  (1 << 7)

// Битовая маска кнопок (buttons2)
#define XBOX_BUTTON_LEFT_STICK  (1 << 0)
#define XBOX_BUTTON_RIGHT_STICK (1 << 1)

// D-Pad направления (buttons2, биты 4-7)
#define XBOX_DPAD_UP      0x00
#define XBOX_DPAD_UP_RIGHT 0x01
#define XBOX_DPAD_RIGHT   0x02
#define XBOX_DPAD_DOWN_RIGHT 0x03
#define XBOX_DPAD_DOWN    0x04
#define XBOX_DPAD_DOWN_LEFT 0x05
#define XBOX_DPAD_LEFT    0x06
#define XBOX_DPAD_UP_LEFT 0x07
#define XBOX_DPAD_NONE    0x08

#endif // XBOX_HID_DESCRIPTOR_H

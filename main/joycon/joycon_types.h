/**
 * @file joycon_types.h
 * @brief Общие типы для работы с Joy-Con контроллерами
 * 
 * Определяет общие типы данных, используемые во всех модулях работы с Joy-Con:
 * - Тип Joy-Con (левый/правый)
 * - Состояние подключения
 * 
 * Эти типы используются как в Bluetooth Classic, так и в BLE модулях.
 */

#ifndef JOYCON_TYPES_H
#define JOYCON_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Тип Joy-Con (левый или правый)
typedef enum {
    JOYCON_TYPE_UNKNOWN = 0,
    JOYCON_TYPE_LEFT,
    JOYCON_TYPE_RIGHT
} joycon_type_t;

// Состояние подключения
typedef enum {
    JOYCON_STATE_DISCONNECTED = 0,
    JOYCON_STATE_SCANNING,
    JOYCON_STATE_CONNECTING,
    JOYCON_STATE_CONNECTED,
    JOYCON_STATE_ERROR
} joycon_state_t;

#endif // JOYCON_TYPES_H

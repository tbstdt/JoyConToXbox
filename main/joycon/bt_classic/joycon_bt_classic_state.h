/**
 * @file joycon_bt_classic_state.h
 * @brief Модуль управления состоянием подключений Joy-Con через Bluetooth Classic
 * 
 * Централизованное управление состоянием подключений:
 * - Хранение состояния обоих Joy-Con
 * - Управление переходами состояний с валидацией
 * - Синхронизация доступа к состоянию
 * - Callbacks на изменение состояния
 */

#ifndef JOYCON_BT_CLASSIC_STATE_H
#define JOYCON_BT_CLASSIC_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "../joycon_types.h"
#include "joycon_bt_classic_internal.h"

// Полное определение структуры в internal.h

// Callback для изменения состояния
typedef void (*joycon_bt_classic_state_callback_t)(joycon_type_t type, joycon_state_t state);

/**
 * @brief Инициализация модуля управления состоянием
 * 
 * Должна вызываться после инициализации мьютексов в joycon_bt_classic_init()
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_state_init(void);

/**
 * @brief Деинициализация модуля управления состоянием
 */
void joycon_bt_classic_state_deinit(void);

/**
 * @brief Получить состояние Joy-Con (потокобезопасно)
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @return joycon_state_t Текущее состояние
 */
joycon_state_t joycon_bt_classic_state_get(joycon_type_t type);

/**
 * @brief Установить состояние с валидацией переходов
 * 
 * Проверяет допустимость перехода перед установкой состояния.
 * 
 * @param type Тип Joy-Con
 * @param new_state Новое состояние
 * @return esp_err_t ESP_OK при успехе, ESP_ERR_INVALID_STATE если переход недопустим
 */
esp_err_t joycon_bt_classic_state_set(joycon_type_t type, joycon_state_t new_state);

/**
 * @brief Проверить допустимость перехода состояния
 * 
 * @param type Тип Joy-Con
 * @param new_state Желаемое новое состояние
 * @return true если переход допустим, false если нет
 */
bool joycon_bt_classic_state_can_transition(joycon_type_t type, joycon_state_t new_state);

/**
 * @brief Получить структуру подключения (для обратной совместимости)
 * 
 * ВНИМАНИЕ: Возвращает указатель на внутреннюю структуру.
 * Мьютекс должен быть захвачен вызывающей стороной.
 * Используется для обратной совместимости с существующим кодом.
 * 
 * @param type Тип Joy-Con
 * @return joycon_bt_classic_connection_t* Указатель на структуру подключения или NULL
 */
joycon_bt_classic_connection_t* joycon_bt_classic_state_get_connection(joycon_type_t type);

/**
 * @brief Подписка на изменения состояния
 * 
 * @param callback Функция обратного вызова (может быть NULL)
 */
void joycon_bt_classic_state_subscribe(joycon_bt_classic_state_callback_t callback);

#endif // JOYCON_BT_CLASSIC_STATE_H

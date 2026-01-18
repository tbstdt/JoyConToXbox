/**
 * @file joycon_bt_classic_l2cap.h
 * @brief Модуль работы с L2CAP каналами для Joy-Con через Bluetooth Classic
 * 
 * Содержит:
 * - Инициализацию L2CAP модуля
 * - Открытие и закрытие L2CAP каналов (Interrupt и Control)
 * - Отправку Output Reports (команды вибрации и настройки)
 * - Обработку Input Reports (данные от Joy-Con)
 * - Управление состоянием L2CAP каналов
 */

#ifndef JOYCON_BT_CLASSIC_L2CAP_H
#define JOYCON_BT_CLASSIC_L2CAP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "../joycon_types.h"

// PSM для HID каналов (стандартные значения)
#define HID_INTERRUPT_PSM 0x13  // HID Interrupt канал для Input Reports
#define HID_CONTROL_PSM 0x11    // HID Control канал для Output Reports

// Максимальный размер L2CAP пакета
#define L2CAP_MAX_PACKET_SIZE 512

/**
 * @brief Инициализация L2CAP модуля
 * 
 * Должна вызываться после esp_bluedroid_enable()
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_l2cap_init(void);

/**
 * @brief Деинициализация L2CAP модуля
 * 
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_l2cap_deinit(void);

/**
 * @brief Открыть L2CAP каналы для Joy-Con
 * 
 * Открывает оба канала (Interrupt и Control) для указанного джойкона.
 * Вызывается после установления ACL соединения.
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_open_l2cap_channels(joycon_type_t type);

/**
 * @brief Закрыть L2CAP каналы для Joy-Con
 * 
 * Закрывает оба канала для указанного джойкона.
 * Вызывается при отключении ACL соединения.
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_close_l2cap_channels(joycon_type_t type);

/**
 * @brief Отправить Output Report через L2CAP Control канал
 * 
 * Используется для отправки команд джойкону:
 * - Set Input Report Mode (Subcommand 0x50)
 * - HD Rumble (Subcommand 0x10)
 * - Player LEDs (Subcommand 0x40)
 * 
 * @param type Тип Joy-Con (левый или правый)
 * @param data Данные для отправки
 * @param len Длина данных
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_send_output_report(joycon_type_t type, const uint8_t *data, size_t len);

/**
 * @brief Отправить Subcommand джойкону
 * 
 * Вспомогательная функция для отправки subcommand через Control канал.
 * 
 * @param type Тип Joy-Con
 * @param subcmd ID subcommand
 * @param data Данные subcommand (может быть NULL)
 * @param data_len Длина данных subcommand
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_send_subcommand(joycon_type_t type, uint8_t subcmd, const uint8_t *data, size_t data_len);

/**
 * @brief Установить режим Input Report Mode
 * 
 * Устанавливает режим отправки данных от джойкона.
 * Вызывается после успешного открытия L2CAP каналов.
 * 
 * @param type Тип Joy-Con
 * @param report_id ID отчета для получения (обычно 0x30)
 * @return esp_err_t ESP_OK при успехе
 */
esp_err_t joycon_bt_classic_set_input_report_mode(joycon_type_t type, uint8_t report_id);

/**
 * @brief Проверить, открыты ли L2CAP каналы для джойкона
 * 
 * @param type Тип Joy-Con
 * @return true если оба канала открыты
 */
bool joycon_bt_classic_l2cap_channels_ready(joycon_type_t type);

#endif // JOYCON_BT_CLASSIC_L2CAP_H

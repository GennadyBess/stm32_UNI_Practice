/**
 * @file           mcp41010.c
 * @brief          Драйвер цифрового потенциометра MCP41010
 */
#include "mcp41010.h"

/**
 * @brief Формирует короткую задержку для соблюдения временных параметров шины SPI.
 * @param None
 */
static inline void POT_Delay(void)
{
    for (volatile uint32_t i = 0; i < 15; i++)
    {
        __NOP();
    }
}

/**
 * @brief Побитовая программная отправка одного байта по интерфейсу SPI.
 * @param[in] byte Байт данных для отправки на устройство.
 */
static void MCP41010_SendByte(uint8_t byte)
{
    for (int8_t i = 7; i >= 0; i--)
    {
        /* SCK низкий перед выставлением бита (Mode 0: данные готовятся на низком уровне) */
        HAL_GPIO_WritePin(POT_SCK_GPIO_Port, POT_SCK_Pin, GPIO_PIN_RESET);

        /* Выставляем бит на MOSI, старший бит первый */
        if (byte & (1 << i))
        {
            HAL_GPIO_WritePin(POT_MOSI_GPIO_Port, POT_MOSI_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(POT_MOSI_GPIO_Port, POT_MOSI_Pin, GPIO_PIN_RESET);
        }

        POT_Delay();

        /* Растущий фронт SCK — MCP41010 защёлкивает бит по фронту вверх */
        HAL_GPIO_WritePin(POT_SCK_GPIO_Port, POT_SCK_Pin, GPIO_PIN_SET);

        POT_Delay();
    }

    /* Возвращаем SCK в состояние покоя */
    HAL_GPIO_WritePin(POT_SCK_GPIO_Port, POT_SCK_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Установка положения цифрового потенциометра MCP41010.
 * @details Функция формирует полную команду управления, состоящую из 16 бит.
 * Первым байтом передается команда записи в канал Potentiometer 0 (0x11),
 * вторым — целевая позиция движка (0-255).
 * @param[in] value Новое положение движка потенциометра (от 0 до 255).
 * 0   — минимальное сопротивление.
 * 255 — максимальное сопротивление (номинал 10 кОм).
 */
void MCP41010_SetValue(uint8_t value)
{
    /* CS активен (низкий уровень) */
    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_RESET);
    POT_Delay();

    MCP41010_SendByte(0x11);  // команда: запись в потенциометр P0 (биты выбора канала + запись)
    MCP41010_SendByte(value); // значение сопротивления, 0..255

    POT_Delay();
    /* CS неактивен (высокий уровень) — фиксирует новое значение */
    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_SET);
}

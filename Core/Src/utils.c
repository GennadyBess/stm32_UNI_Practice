/**
  * @file           utils.c
  * @brief          Вспомогательные функции
  */
#include "utils.h"


/**
 * @brief Конвертирует строку ASCII в неотрицательное целое число (от 0 до 65_535).
 * * @details Функция посимвольно парсит строку, пока не встретит символ,
 * не являющийся цифрой от '0' до '9'.
 * * @note Функция не обрабатывает знак минус ('-')
 * и возвращает ошибку при попытке передать первым символом не цифру.
 * Числа больше допустимого будут приведены к диапазону 0-65_535.
 * * @param[in] str Указатель на исходную нуль-терминальную строку.
 * * @retval >=0 Успешно декодированное число.
 * @retval -1  Ошибка парсинга (строка пуста или первый символ не является цифрой).
 */
int32_t custom_atoi(const char *str) {
    uint16_t res = 0;
    uint8_t is_number = 0;
    while (*str >= '0' && *str <= '9') {
        res = res * 10 + (*str - '0');
        str++;
        is_number = 1;
    }
    if (is_number) return res;
    else return -1;
}

/**
 * @brief Конвертирует строку ASCII в неотрицательное число с плавающей точкой.
 * * @details Парсит целую и дробную части числа. В качестве разделителя
 * допускается только точка ('.').
 * * @note Функция не обрабатывает знак минус ('-')
 * и возвращает ошибку при попытке передать первым символом не цифру.
 * Числа больше допустимого будут приведены к диапазону типа данных float.
 * * @param[in] str Указатель на исходную нуль-терминальную строку.
 * * @retval >=0  Успешно декодированное число.
 * @retval -1.0f  Ошибка парсинга (неверный формат или некорректные символы).
 */
float custom_atof(const char *str) {
    float res = 0.0f;
    float factor = 1.0f;

    uint8_t is_number = 0;
    while (*str >= '0' && *str <= '9') {
        res = res * 10 + (*str - '0');
        str++;
        is_number = 1;
    }

    if (*str == '.') {
		str++;
		while (*str >= '0' && *str <= '9') {
			factor *= 0.1f;
			res += (*str - '0') * factor;
			str++;
		}
	}
    if (is_number) return res;
    else return -1.0f;
}

/**
 * @brief Конвертирует беззнаковое 32-битное число в строку ASCII с форматированием.
 * @details Функция переводит число в текстовый вид, выравнивая его по правому краю
 * и заполняя пустые левые разряды указанным символом-заполнителем (например, нулями или пробелами).
 * Строка записывается в буфер БЕЗ завершающего нуля (нуль-терминатора), что позволяет
 * легко склеивать строки.
 * @param[in]  value    Преобразуемое число (uint32_t).
 * @param[out] buf      Указатель на буфер в памяти, куда будет записан результат.
 * @param[in]  digits   Минимальное количество символов, которое должно быть записано.
 * @param[in]  pad_char Символ для заполнения пустых левых разрядов (например, '0' или ' ').
 * @warning Длина Value должна быть НЕ БОЛЬШЕ значения digits. Соответствующей проверки нет.
 * @return int Количество фактически записанных в буфер символов (всегда равно параметру digits).
 */
int custom_utoa(uint32_t value, char* buf, int digits, char pad_char)
{

    for (int i = 0; i < digits; i++) {
        buf[i] = pad_char;
    }

    int index = digits - 1;
    for (int i = 0; i < digits; i++) {
    	if (index >= 0) {
    		buf[index--] = (value % 10) + '0';
    		value /= 10;
    	}
    }
    return digits;
}

/**
 * @brief Форматирует и добавляет показания датчика в текстовый буфер.
 * @details Функция формирует строку фиксированного формата для вывода физических величин
 * (напряжение, ток)
 * * @param[out] buf       Указатель на рабочий буфер, куда дописываются данные.
 * @param[in]  p         Текущая позиция (индекс) для записи в буфере.
 * @param[in]  label     Нуль-терминальная строка с именем физической величины.
 * @param[in]  is_neg    Флаг знака.
 * @param[in]  main_part Целая часть числа для вывода.
 * @param[in]  sub_part  Дробная часть числа для вывода.
 * @param[in]  unit      Символ единицы измерения.
 * * @return int Новый индекс позиции в буфере после успешной записи всех символов.
 */
int append_sensor_data(char *buf, int p, const char *label, uint8_t is_neg,
                       uint32_t main_part, uint32_t sub_part, char unit)
{
    while (*label) {
        buf[p++] = *label++;
    }

    buf[p++] = ':';
    buf[p++] = ' ';

    buf[p++] = is_neg ? '-' : ' ';

    p += custom_utoa(main_part, &buf[p], 2, '0');

    buf[p++] = '.';

    p += custom_utoa(sub_part, &buf[p], PRECISION_DIGITS, '0');

    buf[p++] = ' ';
    buf[p++] = unit;
    buf[p++] = '\r';
    buf[p++] = '\n';

    return p;
}

/**
 * @brief Вычисляет компоненты float числа и добавляет коэффициент калибровки в текстовый буфер.
 * @details Функция разбивает вещественное число на целую и дробную части,
 * и записывает получившуюся строку в буфер.
 * * @param[out] buf   Указатель на рабочий буфер, куда дописываются данные.
 * @param[in]  p     Текущая позиция (индекс) для записи в буфере.
 * @param[in]  label Нуль-терминальная строка с именем коэффициента.
 * @param[in]  value Вещественное значение коэффициента (float), которое нужно вывести.
 * * @return int Новый индекс позиции в буфере после успешной записи всех символов.
 */
int append_float_coef(char *buf, int p, const char *label, float value) {
    while (*label) {
        buf[p++] = *label++;
    }

    uint8_t is_neg = 0;
    float value_abs = value;
    if (value < 0.0f) {
        is_neg = 1;
        value_abs = -value;
    }

    buf[p++] = is_neg ? '-' : ' ';

    uint32_t integer_part = (uint32_t)value_abs;
    uint32_t decimal_part = (uint32_t)((value_abs - (float)integer_part) * PRECISION);

    p += custom_utoa(integer_part, &buf[p], 2, ' ');

    buf[p++] = '.';

    p += custom_utoa(decimal_part, &buf[p], PRECISION_DIGITS, '0');

    buf[p++] = '\r';
    buf[p++] = '\n';

    return p;
}

/**
 * @brief Очищает строку от лишних пробелов.
 * @details Удаляет все начальные и конечные пробелы, а также заменяет
 * несколько подряд идущих пробелов внутри строки на один пробел.
 * @param[in,out] str Указатель на обрабатываемую нуль-терминальную строку.
 */
void strip_extra_spaces(char *str) {
    if (!str) return;

    char *src = str;
    char *dst = str;
    uint8_t space_found = 0;

    while (*src == ' ' || *src == '\t') {
        src++;
    }

     while (*src != '\0') {
        if (*src == ' ' || *src == '\t') {
            if (!space_found) {
                *dst++ = ' ';
                space_found = 1;
            }
        } else {
            *dst++ = *src;
            space_found = 0;
        }
        src++;
    }

    if (dst > str && *(dst - 1) == ' ') {
        dst--;
    }

    *dst = '\0';
}

/**
  * @file           cli.c
  * @brief          Обработка консольных команд пользователя
  */
#include "main.h"
#include "cli.h"
#include "utils.h"
#include "mcp41010.h"
#include "usbd_cdc_if.h"
#include "ee.h"

#define CAL_TABLE_SIZE 10

typedef struct {
    float measured;
    float real;
} cal_point_t;

static cal_point_t cal_voltage_table[CAL_TABLE_SIZE];
static cal_point_t cal_current_table[CAL_TABLE_SIZE];
static uint8_t cal_voltage_index = 0;
static uint8_t cal_current_index = 0;

/**
 * @brief Расчет калибровочных коэффициентов методом наименьших квадратов (МНК).
 * @details Функция принимает массив накопленных экспериментальных точек и строит линейную
 * зависимость вида: y = k * x + b, минимизируя квадратичное отклонение.
 * @param[in]  table Указатель на таблицу точек.
 * @param[in]  n     Количество точек в таблице.
 * @param[out] out_k Указатель на переменную, где будет храниться коэффициент k.
 * @param[out] out_b Указатель на переменную, где будет храниться коэффициент b.
 * @retval 1 Если расчет успешно выполнен.
 * @retval 0 Ошибка расчета x(недостаточно точек или определитель близок к нулю).
 */
static uint8_t SolveLinearFit(cal_point_t *table, uint8_t n, float *out_k, float *out_b)
{
    if (n < 2) return 0;

    float Sx = 0.0f, Sy = 0.0f, Sxx = 0.0f, Sxy = 0.0f;

    for (uint8_t i = 0; i < n; i++)
    {
    	float meas = table[i].measured;
    	float real = table[i].real;
        Sx  += meas;
        Sy  += real;
        Sxx += meas * meas;
        Sxy += meas * real;
    }

    float det = Sxx * (float)n - Sx * Sx;

    if (det < 0.000001f && det > -0.000001f)
    {
        return 0;
    }

    *out_k = (Sxy * (float)n - Sx * Sy) / det;
    *out_b = (Sxx * Sy - Sx * Sxy) / det;

    return 1;
}

/**
 * @brief Обработчик команды "debug". Управление периодической отправкой данных телеметрии.
 * @details Включает/выключает флаг отладки или задает интервал отправки в мс.
 * @param[in] args Строка аргументов. "off" для отключения; число для установки интервала;
 * пустая строка устанавливает интервал по умолчанию (1000 мс).
 */
void do_debug(char *args) {
	if (!args || *args == '\0') {
			debug_interval = 1000;
			debug_enabled = 1;
			return;
	}

	if (strcmp(args, "off") == 0) {
		debug_enabled = 0;
	    char msg[] = "Debug is off\r\n";
	    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	int32_t interval = custom_atoi(args);
	if (interval <= 0 || interval == -1) {
		debug_interval = 1000;
	} else {
		if (interval >= 50) {
			debug_interval = (uint16_t)(interval / 50 * 50);
		} else {
			debug_interval = 1000;
		}
	}
    debug_enabled = 1;
}

/**
 * @brief Обработчик команды "get". Запрос на однократную отправку состояния.
 * @param None
 */
void do_get(char *args) {
    get_status_enabled = 1;
}

/**
 * @brief Обработчик команды "stop". Остановка автоматического регулирования.
 * @param None
 */
void do_stop(char *args) {
    voltage_regulation_enabled = 0;
    current_regulation_enabled = 0;
    char msg[] = "Regulation stopped\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

/**
 * @brief Обработчик команды "led". Прямое управление отладочным светодиодом.
 * @param[in] args Строка "on" для включения светодиода, "off" — для выключения.
 */
void do_led(char *args) {
	if (strcmp(args, "on") == 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		char msg[] = "LED is now ON\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
	else if (strcmp(args, "off") == 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		char msg[] = "LED is now OFF\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
	else {
		char msg[] = "ERROR: Invalid argument. Use 'on' or 'off'\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
}

/**
 * @brief  Обработчик команды "set". Ручная установка целевых параметров.
 * @details Поддерживает три режима:
 *          - `set v <float>` : Установка целевого напряжения
 *          - `set c <float>` : Установка целевого тока
 *          - `set r <int>`   : Ручная запись кода в цифровой потенциометр (0–255)
 * @param[in] args Строка параметров с токенами типа и значения.
 */
void do_set(char *args) {
	char *type_str = strtok(args, " ");
	char *value_str = strtok(NULL, " ");

	if (strcmp(type_str, "v") == 0)
	{
		float raw = custom_atof(value_str);
		uint8_t ok = Set_Target_Voltage(raw);
		if (ok) {
			char msg[] = "V Value changed\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		} else {
			char msg[] = "ERORR: Invalid Voltage\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		}
	}
	else if (strcmp(type_str, "c") == 0)
	{
		float raw = custom_atof(value_str);
		uint8_t ok = Set_Target_Current(raw);
		if (ok) {
			char msg[] = "C Value changed\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		} else {
			char msg[] = "ERORR: Invalid Current\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		}
	}
	else if (strcmp(type_str, "r") == 0)
	{
		int value = custom_atoi(value_str);
		if (value == -1) {
			char msg[] = "ERROR: Invalid R-value\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		}
		MCP41010_SetValue((uint8_t)value);
		voltage_regulation_enabled = 0;
		current_regulation_enabled = 0;
		char msg[] = "R Value changed\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
	else
	{
		char msg[] = "ERROR: Invalid type\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}
}

/**
 * @brief  Обработчик команды "coeff". Вывод текущих калибровочных и ПИД-коэффициентов.
 * @param[in] args Выбор канала: "v" (калибровка напряжения), "c" (калибровка тока),
 *                 "piv" (ПИ напряжения) или "pic" (ПИ тока).
 */
void do_print_coefficients(char *args) {
	if (strcmp(args, "v") == 0) {
		char tx_buffer[64];
		int p = 0;
		const char v_label[] = "Voltage:\r\n";
		strcpy(&tx_buffer[p], v_label); p += strlen(v_label);
		p = append_float_coef(tx_buffer, p, "K: ", k_voltage);
		p = append_float_coef(tx_buffer, p, "B: ", b_voltage);
		CDC_Transmit_FS((uint8_t*)tx_buffer, p);
	}
	else if (strcmp(args, "c") == 0) {
	    char tx_buffer[64];
		int p = 0;
	    const char c_label[] = "Current:\r\n";
	    strcpy(&tx_buffer[p], c_label); p += strlen(c_label);
	    p = append_float_coef(tx_buffer, p, "K: ", k_current);
	    p = append_float_coef(tx_buffer, p, "B: ", b_current);
	    CDC_Transmit_FS((uint8_t*)tx_buffer, p);
	}
	else if (strcmp(args, "pic") == 0) {
		char tx_buffer[64];
		int p = 0;

		const char c_head[] = "C_PID:\r\n";
		strcpy(&tx_buffer[p], c_head); p += strlen(c_head);
		p = append_float_coef(tx_buffer, p, "Kp: ", c_Kp);
		p = append_float_coef(tx_buffer, p, "Ki: ", c_Ki);
		p = append_float_coef(tx_buffer, p, "Kd: ", c_Kd);
		tx_buffer[p] = '\0';
		CDC_Transmit_FS((uint8_t*)tx_buffer, p);
	}
	else if (strcmp(args, "piv") == 0) {
		char tx_buffer[64];
		int p = 0;

		const char v_head[] = "V_PID:\r\n";
		strcpy(&tx_buffer[p], v_head); p += strlen(v_head);
		p = append_float_coef(tx_buffer, p, "Kp: ", v_Kp);
		p = append_float_coef(tx_buffer, p, "Ki: ", v_Ki);
		p = append_float_coef(tx_buffer, p, "Kd: ", v_Kd);
		CDC_Transmit_FS((uint8_t*)tx_buffer, p);
	}
	else {
		char msg[] = "ERORR: Invalid Argument\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
	}

}

/**
 * @brief  Обработчик команды "cal". Управление процессом калибровки АЦП.
 * @details Поддерживает режимы:
 *          - `cal <v|c> <float>` : Добавление экспериментальной точки (измеренное vs эталонное)
 *          - `cal <v|c> start`   : Выполнение расчета МНК и сохранение параметров в Flash
 *          - `cal <v|c> reset`   : Сброс калибровочных коэффициентов к дефолтным (k=1, b=0)
 * @param[in] args Строка параметров.
 */
void do_calibration(char *args) {
	if (!args || *args == '\0') {
		char msg[] = "ERROR: Usage: cal <v|c> <start|reset> OR cal <v|c> <value>\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	char *type_str = strtok(args, " ");

	uint8_t is_voltage = 0;
	if (strcmp(type_str, "v") == 0) {
		is_voltage = 1;
	} else if (strcmp(type_str, "c") == 0) {
		is_voltage = 0;
	} else {
		char msg[] = "ERROR: Invalid type. Use 'v' for voltage, 'c' for current\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	char *value_str = strtok(NULL, " ");
	if (!value_str) {
		char msg[] = "ERROR: Missing second argument\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	if (strcmp(value_str, "start") == 0) {
		if (is_voltage) {
			uint8_t flag = SolveLinearFit(cal_voltage_table, cal_voltage_index, &k_voltage, &b_voltage);
			if (flag == 0) {
				CDC_Transmit_FS((uint8_t*)"ERROR: Calculation failed\r\n", 28);
				return;
			}
			cal_voltage_index = 0;
			settings.k_volt = k_voltage;
			settings.b_volt = b_voltage;
			EE_Write();
			memset(cal_voltage_table, 0, sizeof(cal_voltage_table));
			CDC_Transmit_FS((uint8_t*)"Voltage fit calculated\r\n", 25);
		} else {
			uint8_t flag = SolveLinearFit(cal_current_table, cal_current_index, &k_current, &b_current);
			if (flag == 0) {
				CDC_Transmit_FS((uint8_t*)"ERROR: Calculation failed\r\n", 28);
				return;
			}
			cal_current_index = 0;
			settings.k_curr = k_current;
			settings.b_curr = b_current;
			EE_Write();
			memset(cal_current_table, 0, sizeof(cal_current_table));
			CDC_Transmit_FS((uint8_t*)"Current fit calculated\r\n", 25);
		}
		return;
	}

	if (strcmp(value_str, "reset") == 0) {
		if (is_voltage) {
			cal_voltage_index = 0;
			k_voltage = 1;
			b_voltage = 0;
			settings.k_volt = 1;
			settings.b_volt = 0;
			EE_Write();
			memset(cal_voltage_table, 0, sizeof(cal_voltage_table));
			CDC_Transmit_FS((uint8_t*)"Voltage reseted\r\n", 17);
		} else {
			cal_current_index = 0;
			k_current = 1;
			b_current = 0;
			settings.k_curr = 1;
			settings.b_curr = 0;
			EE_Write();
			memset(cal_current_table, 0, sizeof(cal_current_table));
			CDC_Transmit_FS((uint8_t*)"Current reseted\r\n", 17);
		}
		return;
	}

	float value = custom_atof(value_str);
	if (value == -1.0f) {
		char msg[] = "ERROR: Invalid float value\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	if (is_voltage) {
		if (cal_voltage_index >= CAL_TABLE_SIZE) {
			char msg[] = "ERROR: Voltage table is full\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
			return;
		}
		cal_voltage_table[cal_voltage_index].measured = voltage.whole_num;
		cal_voltage_table[cal_voltage_index].real = value;
		cal_voltage_index++;
		char msg[] = "V Value submitted\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

	} else {
		if (cal_current_index >= CAL_TABLE_SIZE) {
			char msg[] = "ERROR: Current table is full\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
			return;
		}
		cal_current_table[cal_current_index].measured = current.whole_num;
		cal_current_table[cal_current_index].real = value;
		cal_current_index++;
		char msg[] = "C Value submitted\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

	}
}

/**
 * @brief  Обработчик команды "pi". Настройка и сохранение коэффициентов PI-регулятора.
 * @details Форматы команды:
 *          - `pi <v|c> <kp|ki|kd> <value>` : Оперативное изменение коэффициента
 *          - `pi <v|c> save`              : Сохранение настроек в Flash
 * @param[in] args Строка параметров.
 */
void do_pi_change(char *args) {
    if (!args || *args == '\0') {
        char msg[] = "ERROR: Usage: pi <v|c> <kp|ki|kd> <value>\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        return;
    }

    char *type_str = strtok(args, " ");
    if (!type_str) {
        char msg[] = "ERROR: Missing second argument>\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        return;
    }

    uint8_t is_voltage = 0;
    if (strcmp(type_str, "v") == 0) {
        is_voltage = 1;
    } else if (strcmp(type_str, "c") == 0) {
        is_voltage = 0;
    } else {
        char msg[] = "ERROR: Invalid type. Use 'v' for voltage, 'c' for current\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        return;
    }

    char *coeff_str = strtok(NULL, " ");
    if (!coeff_str) {
		char msg[] = "ERROR: Missing third argument>\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	if (strcmp(coeff_str, "save") == 0) {
	    if (is_voltage) {
			settings.volt_Kp = v_Kp;
			settings.volt_Ki = v_Ki;
			settings.volt_Kd = v_Kd;
			char msg[] = "Pi coeffs for V saved\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		} else {
			settings.curr_Kp = c_Kp;
			settings.curr_Ki = c_Ki;
			settings.curr_Kd = c_Kd;
			char msg[] = "Pi coeffs for C saves\r\n";
			CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		}
	    EE_Write();
	    return;
	}

    uint8_t channel = 1;
	if (strcmp(coeff_str, "kp") == 0) {
		channel = 1;
	} else if (strcmp(coeff_str, "ki") == 0) {
		channel = 2;
	} else if (strcmp(coeff_str, "kd") == 0) {
		channel = 3;
	} else {
		char msg[] = "ERROR: Invalid channel. Use kp | ki | kd\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

	char *value_str = strtok(NULL, " ");
	if (!value_str) {
		char msg[] = "ERROR: Missing forth argument>\r\n";
		CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
		return;
	}

    float pi_coeff = custom_atof(value_str);
    if (pi_coeff == -1.0f) {
        char msg[] = "ERROR: Invalid float value in PID coefficients\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        return;
    }


    if (is_voltage) {
		if (channel == 1)      v_Kp = pi_coeff;
		else if (channel == 2) v_Ki = pi_coeff;
		else if (channel == 3) v_Kd = pi_coeff;
	} else {
		if (channel == 1)     c_Kp = pi_coeff;
		else if (channel == 2) c_Ki = pi_coeff;
		else if (channel == 3) c_Kd = pi_coeff;
	}
	char msg[] = "PID coefficient updated\r\n";
	CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

/**
 * @brief Таблица сопоставления текстовых команд и функций-обработчиков CLI.
 */
const command_t cmd_table[] = {
    {"debug", do_debug},
    {"get",   do_get},
    {"stop",  do_stop},
	{"led",   do_led},
	{"set",   do_set},
	{"coeff", do_print_coefficients},
	{"cal",   do_calibration},
	{"pi",    do_pi_change}
};

#define CMD_COUNT (sizeof(cmd_table) / sizeof(command_t))

/**
 * @brief Функция парсинга и распределения входящих строк команд.
 * @details Очищает строку от символов переноса и лишних пробелов, выделяет первый токен как имя команды,
 * ищет совпадение в cmd_table и вызывает соответствующий обработчик.
 * @param[in] cmd Указатель на строку.
 */
void process_command(char *cmd) {
    cmd[strcspn(cmd, "\r\n")] = 0;
    strip_extra_spaces(cmd);

    char *command = strtok(cmd, " ");
    char *args = strtok(NULL, "");

    if (!command) return;

    for (size_t i = 0; i < CMD_COUNT; i++) {
        if (strcmp(command, cmd_table[i].name) == 0) {
            cmd_table[i].handler(args);
            return;
        }
    }


    char msg[] = "ERROR: Unknown command\r\n";
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

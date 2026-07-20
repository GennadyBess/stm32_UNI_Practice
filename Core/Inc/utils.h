#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define PRECISION_DIGITS 4
#if PRECISION_DIGITS == 1
    #define PRECISION 10.0f
#elif PRECISION_DIGITS == 2
    #define PRECISION 100.0f
#elif PRECISION_DIGITS == 3
    #define PRECISION 1000.0f
#elif PRECISION_DIGITS == 4
    #define PRECISION 10000.0f
#elif PRECISION_DIGITS == 5
    #define PRECISION 100000.0f
#else
    #define PRECISION 100.0f
#endif

int32_t custom_atoi(const char *str);
float custom_atof(const char *str);
int custom_utoa(uint32_t value, char* buf, int digits, char pad_char);
int append_sensor_data(char *buf, int p, const char *label, uint8_t is_neg,
                       uint32_t main_part, uint32_t sub_part, char unit);
int append_float_coef(char *buf, int p, const char *label, float value);
void strip_extra_spaces(char *str);
#endif

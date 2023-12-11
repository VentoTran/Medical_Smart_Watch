#ifndef ALGORITHM_H
#define ALGORITHM_H

// #include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define BUFFER_SIZE (128)
#define DELAY_AMOSTRAGEM (19.2)
#define CORRELATION_RATIO (85.0)


void filter_n(uint32_t *ir_buffer, uint32_t *red_buffer, int filter);

void remove_dc_part(uint32_t *ir_buffer, uint32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean);
void calculate_linear_regression(double *angular_coef, double *linear_coef, uint32_t *data);
double correlation_datay_datax(uint32_t *data_red, uint32_t *data_ir);
void remove_trend_line(uint32_t *buffer);

// Para calcular a regressão linear.
double sum_of_xy_elements(uint32_t *data);
uint64_t sum_of_elements(uint32_t *data);
double sum_of_squared_elements(uint32_t *data);
double somatoria_x2() __attribute__((optimize(2)));
void init_time_array();
int calculate_heart_rate(uint32_t *ir_data, double *r0, double *auto_correlationated_data);
double spo2_measurement(uint32_t *ir_data, uint32_t *red_data, uint64_t ir_mean, uint64_t red_mean);
double rms_value(uint32_t *data);
double auto_correlation_function(uint32_t *data, uint32_t lag);

#endif

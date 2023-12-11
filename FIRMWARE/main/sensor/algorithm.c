#include "algorithm.h"
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"

double time_array[BUFFER_SIZE];

#define DEBUG true
#define MINIMUM_RATIO 0.3

static int32_t avr_n_elem_in_array(int32_t* array, uint32_t start, uint32_t end)
{
	int64_t sum = 0;
	uint32_t len = end - start + 1;
	for (int i = start; i <= end; i++)
	{
		sum += array[i];
	}
	sum /= len;
	return sum;
}

void filter_n(int32_t *ir_buffer, int32_t *red_buffer, int filter)
{
	filter /= 2;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		int lower = i - filter;
		if (lower <= 0)	lower = 0;

		int upper = i + filter;
		if (upper >= 127) upper = 127;

		ir_buffer[i] = avr_n_elem_in_array(ir_buffer, lower, upper);
		red_buffer[i] = avr_n_elem_in_array(red_buffer, lower, upper);
	}
}

void init_time_array()
{
	double time = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		time_array[i] = time;
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
}

void remove_dc_part(int32_t *ir_buffer, int32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean)
{
	*ir_mean = 0;
	*red_mean = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		*ir_mean += ir_buffer[i];
		*red_mean += red_buffer[i];
	}

	*ir_mean = *ir_mean / (BUFFER_SIZE);
	*red_mean = *red_mean / (BUFFER_SIZE);

	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		red_buffer[i] = red_buffer[i] - *red_mean;
		ir_buffer[i] = ir_buffer[i] - *ir_mean;
	}
}

void remove_trend_line(int32_t *buffer)
{
	double a = 0;
	double b = 0;

	calculate_linear_regression(&a, &b, buffer);

/*
	printf("angualar coef = %f\n", a);
	printf("linear coef = %f\n", b);
*/
	double time = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		buffer[i] = ((buffer[i] + (-a * time)) - b);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
}

void calculate_linear_regression(double *angular_coef, double *linear_coef, int32_t *data)
{
	int64_t sum_of_y = sum_of_elements(data);
	double sum_of_x = 325.12; // Automatizar...
	double sum_of_x2 = somatoria_x2();
	double sum_of_xy = sum_of_xy_elements(data);
	double sum_of_x_squared = (sum_of_x * sum_of_x);

	double temp = (sum_of_xy - (sum_of_x * sum_of_y) / BUFFER_SIZE);
	double temp2 = (sum_of_x2 - (sum_of_x_squared / BUFFER_SIZE));

	*angular_coef = temp / temp2;
	*linear_coef = ((sum_of_y / BUFFER_SIZE) - (*angular_coef * (sum_of_x / BUFFER_SIZE)));
}

double correlation_datay_datax(int32_t *data_red, int32_t *data_ir)
{

	double correlation = 0;
	double x_mean = 0;
	double y_mean = 0;
	double volatile sum_of_x = 0;
	double volatile sum_of_y = 0;
	double sum_of_x_minus_xmean2 = 0;
	double sum_of_y_minus_ymean2 = 0;
	double covar_xy = 0;
	double sx = 0;
	double sy = 0;

	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum_of_x += data_red[i];
		sum_of_y += data_ir[i];
	}
	x_mean = sum_of_x / BUFFER_SIZE;
	y_mean = sum_of_y / BUFFER_SIZE;

	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum_of_x_minus_xmean2 += ((data_red[i] - x_mean) * (data_red[i] - x_mean));
		sum_of_y_minus_ymean2 += ((data_ir[i] - y_mean) * (data_ir[i] - y_mean));
		covar_xy += ((data_red[i] - x_mean) * (data_ir[i] - y_mean));
	}
	sx = sqrt(sum_of_x_minus_xmean2 / (BUFFER_SIZE));
	sy = sqrt(sum_of_y_minus_ymean2 / (BUFFER_SIZE));
	covar_xy = (covar_xy / (BUFFER_SIZE));

	correlation = (covar_xy / (sx * sy));

	return correlation;
}

double spo2_measurement(int32_t *ir_data, int32_t *red_data, uint64_t ir_mean, uint64_t red_mean)
{
	double Z = 0;
	double SpO2;
	double ir_rms = rms_value(ir_data);
	double red_rms = rms_value(red_data);

	Z = (red_rms / red_mean) / (ir_rms / ir_mean);
#if SHOW_ALL == 1
	ESP_LOGI("PROCESS", "red_rms = %f", red_rms);
	ESP_LOGI("PROCESS", "ir_rms = %f", ir_rms);
	ESP_LOGI("PROCESS", "Z = %f", Z);
#endif

	// SpO2 = (49.7 * Z);
	SpO2 = (-45.06*Z + 30.354)*Z + 94.845;
	return SpO2;
}

int calculate_heart_rate(int32_t *ir_data, double *r0, double *auto_correlationated_data)
{
	double auto_correlation_result;
	double resultado = 333;
	double auto_coorelation_0 = auto_correlation_function(ir_data, 0);
	*r0 = auto_coorelation_0;
#if SHOW_ALL == 1
	ESP_LOGI("PROCESS", "R0 = %f", *r0);
#endif
	double biggest_value = 0;
	int biggest_value_index = 0;
	double division;

	for (float i = 0; i < 125; i++)
	{
		auto_correlation_result = auto_correlation_function(ir_data, i);
		division = auto_correlation_result / auto_coorelation_0;
		auto_correlationated_data[(int)i] = division;

		if (i > 10)
		{
			if (division > MINIMUM_RATIO)
			{
				if (biggest_value < division)
				{
					biggest_value = division;
					biggest_value_index = i;
					continue;
				}

				resultado = ((1 / (biggest_value_index * (DELAY_AMOSTRAGEM / 1000.0))) * 60);
#if !DEBUG
				return ((int)resultado); // Não retorna daqui seestiver debugando.
#endif
			}
		}
	}
	return (int)resultado;
}

double auto_correlation_function(int32_t *data, int32_t lag)
{
	double soma = 0;
	double resultado = 0;
	for (int i = 0; i < (BUFFER_SIZE - lag); i++)
	{
		soma += ((data[i]) * (data[i + lag]));
	}
	resultado = soma / BUFFER_SIZE;
	return resultado;
}

int64_t sum_of_elements(int32_t *data)
{
	int64_t sum = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum += data[i];
	}
	return sum;
}

double sum_of_xy_elements(int32_t *data)
{
	double sum_xy = 0;
	double time = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum_xy += (data[i] * time);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
	return sum_xy;
}

double sum_of_squared_elements(int32_t *data)
{
	double sum_squared = 0;
	int time = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		sum_squared += (data[i] * data[i]);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
	return sum_squared;
}

double somatoria_x2()
{
	float incremento = (DELAY_AMOSTRAGEM / 1000.0);
	double resultado = 0.0;
	double squared_values = 0;
	float temp = 0;

	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		squared_values = temp * temp;
		temp += incremento;
		resultado += squared_values;
	}
	return resultado;
}

double rms_value(int32_t *data)
{
	double result = 0;
	int32_t somatoria = 0;
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		somatoria += (data[i] * data[i]);
	}
	result = sqrt(somatoria / BUFFER_SIZE);
	return result;
}

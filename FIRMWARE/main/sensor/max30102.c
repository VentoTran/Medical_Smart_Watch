#include <stdio.h>

#include "max30102.h"
#include "math.h"

#include "driver/i2c.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief Built-in plotting function. Called during an interrupt to print/plot the current sample.
 * @note Override this in your main.c if you do not use printf() for printing.
 * @param ir_sample
 * @param red_sample
 */
void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
    // ESP_LOGI("Sensor", " %ld|%ld ", ir_sample, red_sample);
    printf("%ld|%ld\n", ir_sample, red_sample);
}

//===================================================================================================================================

/**
 * @brief 
 * 
 * @param data_rd 
 * @param size 
 * @return esp_err_t 
 */
static esp_err_t i2c_sensor_read(max30102_t *obj, uint8_t *data_rd, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(obj->_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief 
 * 
 * @param data_wr 
 * @param size 
 * @return esp_err_t 
 */
static esp_err_t i2c_sensor_write(max30102_t *obj, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(obj->_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief 
 * 
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_process(max30102_t *obj)
{
// #if PLOT_FINE == 1
//     for (uint8_t i = 0; i < MAX30102_SAMPLE_LEN_MAX; i++)
//     {
//         // ESP_LOGI("Sensor", " %ld|%ld ", obj->_ir_ready[i], obj->_red_ready[i]);
//         printf("%ld|%ld\n", obj->_ir_ready[i], obj->_red_ready[i]);
//     }
// #endif

    filter_n((int32_t*)obj->_ir_ready, (int32_t*)obj->_red_ready, 5U);
    remove_dc_part((int32_t*)obj->_ir_ready, (int32_t*)obj->_red_ready, &obj->_ir_mean, &obj->_red_mean);
#if SHOW_ALL == 1
    ESP_LOGI("PROCESS", "red_mean = %lld", obj->_red_mean);
	ESP_LOGI("PROCESS", "ir_mean = %lld", obj->_ir_mean);
#endif
    // remove_trend_line((int32_t*)obj->_ir_ready);
    // remove_trend_line((int32_t*)obj->_red_ready);
    obj->pearson_correlation = correlation_datay_datax((int32_t*)obj->_red_ready, (int32_t*)obj->_ir_ready);

    ESP_LOGI("PROCESS", "pearson_correlation = %2.3f", obj->pearson_correlation);

    if (obj->_ir_mean <= 200000)
    {
        ESP_LOGI("Sensor", "Unmount -> Nothing to Process");
        obj->HeartRate = 0;
        obj->SpO2 = 0.0;
        obj->status_flag = max30102_gather_data;
    }

#if PLOT_FINE == 1
    for (uint8_t i = 0; i < MAX30102_SAMPLE_LEN_MAX; i++)
    {
        // ESP_LOGI("Sensor", " %ld|%ld ", obj->_ir_ready[i], obj->_red_ready[i]);
        printf("%ld|%ld\n", obj->_ir_ready[i], obj->_red_ready[i]);
    }
#endif
}

/**
 * @brief 
 * 
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_get_heart_rate(max30102_t *obj)
{
    uint8_t temp = calculate_heart_rate((int32_t*)obj->_ir_ready, &obj->_r0_autocorrelation, obj->_auto_correlationated_data);

    if (temp >= 50 && temp <= 180)
    {
        obj->HeartRate = temp;
        ESP_LOGI("Sensor", "Heart Rate = %d", obj->HeartRate);
    }
    else
    {
        obj->HeartRate = 0;
        ESP_LOGI("Sensor", "Invalid HR -> Calculation Fail!");
    }
}

/**
 * @brief 
 * 
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_get_spo2(max30102_t *obj)
{
    double temp = spo2_measurement((int32_t*)obj->_ir_ready, (int32_t*)obj->_red_ready, obj->_ir_mean, obj->_red_mean);

    if (temp >= 90.0 && temp <= 100.0)
    {
        obj->SpO2 = temp;
        ESP_LOGI("Sensor", "SpO2 = %2.1f", obj->SpO2);
    }
    else
    {
        obj->SpO2 = 0.0;
        ESP_LOGI("Sensor", "Invalid SpO2 -> Calculation Fail!");
    }
}

//===================================================================================================================================

/**
 * @brief MAX30102 initiation function.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param hi2c Pointer to I2C object handle
 */
void max30102_init(max30102_t *obj, int i2c_port, int sda, int scl, int speed)
{
    obj->_i2c_port = i2c_port;
    i2c_config_t i2c_configuration = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = 1,
        .scl_io_num = scl,
        .scl_pullup_en = 1,
        .master.clk_speed = speed
    };
    esp_err_t err;
    err = i2c_param_config(i2c_port, &i2c_configuration);
    if (err != ESP_OK)
        ESP_LOGE("Sensor", "I2C Config FAIL");
    err = i2c_driver_install(i2c_port, i2c_configuration.mode, 0, 0, 0);
    if (err != ESP_OK)
        ESP_LOGE("Sensor", "I2C Driver Install FAIL");

    obj->_interrupt_flag = 0;
    obj->_sample_index = 0;
    memset(obj->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(obj->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(obj->_ir_ready, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(obj->_red_ready, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
}

/**
 * @brief Write buffer of buflen bytes to a register of the MAX30102.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to write to.
 * @param buf Pointer containing the bytes to write.
 * @param buflen Number of bytes to write.
 */
void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    if (buflen >= 20)    return;
    uint8_t data[21] = {0};
	data[0] = reg;
	memcpy(&data[1], buf, buflen * sizeof(uint8_t));
	i2c_sensor_write(obj, data, buflen+1);
}

/**
 * @brief Read buflen bytes from a register of the MAX30102 and store to buffer.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to read from.
 * @param buf Pointer to the array to write to.
 * @param buflen Number of bytes to read.
 */
void max30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    i2c_sensor_write(obj, &reg, 1);
	i2c_sensor_read(obj, buf, buflen);
}

/**
 * @brief Reset the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_reset(max30102_t *obj)
{
    uint8_t val = 0x40;
    max30102_write(obj, MAX30102_MODE_CONFIG, &val, 1);
}

/**
 * @brief Enable A_FULL interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_a_full(max30102_t *obj, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_A_FULL);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_A_FULL);
    max30102_write(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
}

/**
 * @brief Enable PPG_RDY interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_ppg_rdy(max30102_t *obj, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_PPG_RDY);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_PPG_RDY);
    max30102_write(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
}

/**
 * @brief Enable ALC_OVF interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_alc_ovf(max30102_t *obj, uint8_t enable)
{
    uint8_t reg = 0;
    max30102_read(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
    reg &= ~(0x01 << MAX30102_INTERRUPT_ALC_OVF);
    reg |= ((enable & 0x01) << MAX30102_INTERRUPT_ALC_OVF);
    max30102_write(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
}

/**
 * @brief Enable DIE_TEMP_RDY interrupt.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_die_temp_rdy(max30102_t *obj, uint8_t enable)
{
    uint8_t reg = (enable & 0x01) << MAX30102_INTERRUPT_DIE_TEMP_RDY;
    max30102_write(obj, MAX30102_INTERRUPT_ENABLE_2, &reg, 1);
}

/**
 * @brief Enable temperature measurement.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_die_temp_en(max30102_t *obj, uint8_t enable)
{
    uint8_t reg = (enable & 0x01) << MAX30102_DIE_TEMP_EN;
    max30102_write(obj, MAX30102_DIE_TEMP_CONFIG, &reg, 1);
}

/**
 * @brief Set interrupt flag on interrupt. To be called in the corresponding external interrupt handler.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_on_interrupt(max30102_t *obj)
{
    obj->_interrupt_flag = 1;
}

/**
 * @brief Check whether the interrupt flag is active.
 *
 * @param obj Pointer to max30102_t object instance.
 * @return uint8_t Active (1) or inactive (0).
 */
uint8_t max30102_has_interrupt(max30102_t *obj)
{
    return obj->_interrupt_flag;
}

/**
 * @brief Read interrupt status registers (0x00 and 0x01) and perform corresponding tasks.
 *
 * @param obj Pointer to max30102_t object instance.
 */
bool max30102_interrupt_handler(max30102_t *obj)
{
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30102_read(obj, MAX30102_INTERRUPT_STATUS_1, reg, 2);

    if ((reg[0] >> MAX30102_INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
        max30102_read_fifo(obj);
        return true;
    }

    if ((reg[0] >> MAX30102_INTERRUPT_PPG_RDY) & 0x01)
    {
        // New FIFO data ready
        return false;
    }

    if ((reg[0] >> MAX30102_INTERRUPT_ALC_OVF) & 0x01)
    {
        // Ambient light overflow
        return false;
    }

    if ((reg[1] >> MAX30102_INTERRUPT_DIE_TEMP_RDY) & 0x01)
    {
        // Temperature data ready
        int8_t temp_int;
        uint8_t temp_frac;
        max30102_read_temp(obj, &temp_int, &temp_frac);
        // float temp = temp_int + 0.0625f * temp_frac;
        return false;
    }

    // Reset interrupt flag
    obj->_interrupt_flag = 0;
    return false;
}

/**
 * @brief Shutdown the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param shdn Shutdown bit.
 */
void max30102_shutdown(max30102_t *obj, uint8_t shdn)
{
    uint8_t config;
    max30102_read(obj, MAX30102_MODE_CONFIG, &config, 1);
    config = (config & 0x7f) | (shdn << MAX30102_MODE_SHDN);
    max30102_write(obj, MAX30102_MODE_CONFIG, &config, 1);
}

/**
 * @brief Set measurement mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param mode Measurement mode enum (max30102_mode_t).
 */
void max30102_set_mode(max30102_t *obj, max30102_mode_t mode)
{
    uint8_t config;
    max30102_read(obj, MAX30102_MODE_CONFIG, &config, 1);
    config = (config & 0xf8) | mode;
    max30102_write(obj, MAX30102_MODE_CONFIG, &config, 1);
    max30102_clear_fifo(obj);
}

/**
 * @brief Set sampling rate.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param sr Sampling rate enum (max30102_spo2_st_t).
 */
void max30102_set_sampling_rate(max30102_t *obj, max30102_sr_t sr)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x63) << MAX30102_SPO2_SR;
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

/**
 * @brief Set led pulse width.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param pw Pulse width enum (max30102_led_pw_t).
 */
void max30102_set_led_pulse_width(max30102_t *obj, max30102_led_pw_t pw)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x7c) | (pw << MAX30102_SPO2_LEW_PW);
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

/**
 * @brief Set ADC resolution.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param adc ADC resolution enum (max30102_adc_t).
 */
void max30102_set_adc_resolution(max30102_t *obj, max30102_adc_t adc)
{
    uint8_t config;
    max30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x1f) | (adc << MAX30102_SPO2_ADC_RGE);
    max30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);
}

/**
 * @brief Set LED current.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_1(max30102_t *obj, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write(obj, MAX30102_LED_IR_PA1, &pa, 1);
}

/**
 * @brief Set LED current.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_2(max30102_t *obj, float ma)
{
    uint8_t pa = ma / 0.2;
    max30102_write(obj, MAX30102_LED_RED_PA2, &pa, 1);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_1_2(max30102_t *obj, max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2)
{
    uint8_t val = 0;
    val |= ((slot1 << MAX30102_MULTI_LED_CTRL_SLOT1) | (slot2 << MAX30102_MULTI_LED_CTRL_SLOT2));
    max30102_write(obj, MAX30102_MULTI_LED_CTRL_1, &val, 1);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_3_4(max30102_t *obj, max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4)
{
    uint8_t val = 0;
    val |= ((slot3 << MAX30102_MULTI_LED_CTRL_SLOT3) | (slot4 << MAX30102_MULTI_LED_CTRL_SLOT4));
    max30102_write(obj, MAX30102_MULTI_LED_CTRL_2, &val, 1);
}

/**
 * @brief
 *
 * @param obj Pointer to max30102_t object instance.
 * @param smp_ave
 * @param roll_over_en Roll over enabled(1) or disabled(0).
 * @param fifo_a_full Number of empty samples when A_FULL interrupt issued (0 < fifo_a_full < 15).
 */
void max30102_set_fifo_config(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full)
{
    uint8_t config = 0x00;
    config |= smp_ave << MAX30102_FIFO_CONFIG_SMP_AVE;
    config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIG_ROLL_OVER_EN);
    config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIG_FIFO_A_FULL);
    max30102_write(obj, MAX30102_FIFO_CONFIG, &config, 1);
}

/**
 * @brief Clear all FIFO pointers in the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_clear_fifo(max30102_t *obj)
{
    uint8_t val = 0x00;
    max30102_write(obj, MAX30102_FIFO_WR_PTR, &val, 1);
    max30102_write(obj, MAX30102_FIFO_RD_PTR, &val, 1);
    max30102_write(obj, MAX30102_OVF_COUNTER, &val, 1);
}

/**
 * @brief Read FIFO content and store to buffer in max30102_t object instance.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_read_fifo(max30102_t *obj)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wr_ptr = 0, rd_ptr = 0;
    max30102_read(obj, MAX30102_FIFO_WR_PTR, &wr_ptr, 1);
    max30102_read(obj, MAX30102_FIFO_RD_PTR, &rd_ptr, 1);

    int8_t num_samples;

    num_samples = (int8_t)wr_ptr - (int8_t)rd_ptr;
    if (num_samples < 1)
    {
        num_samples += 32;
    }

#if SHOW_NUM_SAMPLE == 1
    ESP_LOGI("Sensor", "Got %d samples", num_samples);
#endif

    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
    for (uint8_t i = 0; i < num_samples; i++)
    {
        uint8_t sample[6] = {0};
        max30102_read(obj, MAX30102_FIFO_DATA, sample, 6);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
        uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;

        obj->_ir_samples[obj->_sample_index] = ir_sample;
        obj->_red_samples[obj->_sample_index] = red_sample;

        obj->_sample_index++;
        if (obj->_sample_index >= MAX30102_SAMPLE_LEN_MAX)
        {
            obj->_sample_index = 0;
            obj->status_flag = max30102_data_ready;
            memcpy(obj->_ir_ready, obj->_ir_samples, MAX30102_SAMPLE_LEN_MAX * sizeof(int32_t));
            memcpy(obj->_red_ready, obj->_red_samples, MAX30102_SAMPLE_LEN_MAX * sizeof(int32_t));
        }

#if PLOT_RAW == 1
        max30102_plot(ir_sample, red_sample);
#endif
    }
}

/**
 * @brief Read die temperature.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param temp_int Pointer to store the integer part of temperature. Stored in 2's complement format.
 * @param temp_frac Pointer to store the fractional part of temperature. Increments of 0.0625 deg C.
 */
void max30102_read_temp(max30102_t *obj, int8_t *temp_int, uint8_t *temp_frac)
{
    max30102_read(obj, MAX30102_DIE_TINT, (uint8_t *)temp_int, 1);
    max30102_read(obj, MAX30102_DIE_TFRAC, temp_frac, 1);
}





#ifdef __cplusplus
}
#endif

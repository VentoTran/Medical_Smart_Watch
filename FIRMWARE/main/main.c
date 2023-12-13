#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs.h"
#include "time.h"

#include "button/button.h"
#include "display/ssd1306.h"
#include "sensor/max30102.h"
#include "bluetooth/bluetooth.h"

#define BLINK_GPIO (4)
#define CHRG_GPIO (5)
#define STBY_GPIO (12)
#define BUTTON_GPIO (14)

#define FREE1_GPIO (16)
#define FREE2_GPIO (17)
#define FREE3_IGPIO (32)
#define FREE4_IGPIO (33)

#define DISPLAY_SDA (18)
#define DISPLAY_SCL (19)

#define MAX_SDA (21)
#define MAX_SCL (22)
#define MAX_INT (23)

#define MPU_SDA (25)
#define MPU_SCL (26)
#define MPU_INT (27)

#define TOTAL_MINS_IN_A_DAY (24 * 60)
#define DATA_BASE_SAMPLE_PERIOD_MINS (2)

#define STORAGE_NAMESPACE "MSW_Data_Base"
#define DATA_BASE_KEY "Data_Base_Key"

#define QUEUE_LENGTH (10)
#define FILTER_AVR_SAMPLE (4)
#define ITEM_SIZE_HR sizeof(uint8_t)
#define ITEM_SIZE_SPO2 sizeof(double)

#define ALLOW_VALUE_0 (1)

typedef enum
{
    IDLE = 0,
    SINGLE = 1,
    CONTINUOUSLY = 2,
    SLEEP = 3
} Measure_Status_t;

typedef struct
{
    uint8_t HR;
    uint16_t SpO2;
} Data_Storage_t;

static const char whoIam[11] = "IamVTsMSW:\0";

static bool s_led_state = false;
static bool is_Time_Sync = false;
static bool is_Data_Saved = false;
static bool is_HR_Data_Point_Valid = false;
static bool is_SpO2_Data_Point_Valid = false;
static uint8_t HR_Now = 0;
static double SpO2_Now = 0.0;
static uint8_t HR_continuous_invalid_sample_count = 0;
static uint8_t SpO2_continuous_invalid_sample_count = 0;
static Measure_Status_t Device_Status = IDLE;
static uint16_t led_blink_periodMS = 3000;
static struct spp_write_evt_param write;

static button_t button;
static SSD1306_t display;
static max30102_t sensor;

static TaskHandle_t sensor_read_task_handle = NULL;
static TaskHandle_t sensor_process_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;
static TaskHandle_t tranfer_all_task_handle = NULL;
static nvs_handle_t data_base_handle;
static struct tm myLocalTime = {
    .tm_year = 2000 - 1900,
    .tm_mon = 0,
    .tm_mday = 1,
    .tm_hour = 0,
    .tm_min = 0,
    .tm_sec = 0};
static StaticQueue_t xStaticQueue_HR;
static StaticQueue_t xStaticQueue_SpO2;
static QueueHandle_t xQueue_HR;
static QueueHandle_t xQueue_SpO2;

static uint8_t ucQueueStorageArea_HR[QUEUE_LENGTH * ITEM_SIZE_HR];
static uint8_t ucQueueStorageArea_SpO2[QUEUE_LENGTH * ITEM_SIZE_SPO2];
static Data_Storage_t myDataBase[(uint32_t)(TOTAL_MINS_IN_A_DAY / DATA_BASE_SAMPLE_PERIOD_MINS)] = {0};

void sensorReadTask(void *pvParameters);
void sensorProcessTask(void *pvParameters);
void displayTask(void *pvParameters);
void tranferAllTask(void *pvParameters);

// ===========================================================================================================================================================

void button_cb_press_once(void *arg)
{
    ESP_LOGI("Button", "Single Click");

    if ((Device_Status == SINGLE) && (is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true))
    {
        Device_Status = IDLE;
        ESP_LOGI("MODE", "Enter IDLE Mode");
    }
    else if (Device_Status == SLEEP)
    {
        Device_Status = IDLE;
        ESP_LOGI("MODE", "Enter IDLE Mode");
        ssd1306_contrast(&display, 0xFF);
    }
    // else if (Device_Status == CONTINUOUSLY)
    // {
    //     Device_Status = IDLE;
    //     ESP_LOGI("MODE", "Enter IDLE Mode");
    //     if (sensor_read_task_handle != NULL)
    //         vTaskDelete(sensor_read_task_handle);
    //     max30102_reset(&sensor);
    // }
}

void button_cb_press_twice(void *arg)
{
    ESP_LOGI("Button", "Double Click");
    if ((Device_Status == IDLE) || ((Device_Status == SINGLE) && (is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true)))
    {
        Device_Status = SINGLE;
        ESP_LOGI("MODE", "Enter SINGLE Mode");
        BaseType_t ret;
        ret = xTaskCreatePinnedToCore(sensorReadTask, "Sensor Read", 8192, NULL, tskIDLE_PRIORITY + 10, &sensor_read_task_handle, 0U);
        if (ret != pdPASS)
            ESP_LOGE("Task", "FAIL Sensor Task");
    }
    else if ((Device_Status == SINGLE) && ((is_HR_Data_Point_Valid != true) || (is_SpO2_Data_Point_Valid != true)))
    {
        // Abort
        Device_Status = IDLE;
        ESP_LOGI("MODE", "Enter IDLE Mode");
        if (sensor_read_task_handle != NULL)
            vTaskDelete(sensor_read_task_handle);
        max30102_reset(&sensor);
    }
}

void button_cb_hold(void *arg)
{
    ESP_LOGI("Button", "Hold");
    if ((Device_Status == IDLE) || ((Device_Status == SINGLE) && (is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true)))
    {
        Device_Status = CONTINUOUSLY;
        ESP_LOGI("MODE", "Enter CONTINUOUS Mode");
        BaseType_t ret;
        ret = xTaskCreatePinnedToCore(sensorReadTask, "Sensor Read", 8192, NULL, tskIDLE_PRIORITY + 10, &sensor_read_task_handle, 0U);
        if (ret != pdPASS)
            ESP_LOGE("Task", "FAIL Sensor Task");
    }
    else if ((Device_Status == SINGLE) && ((is_HR_Data_Point_Valid != true) || (is_SpO2_Data_Point_Valid != true)))
    {
        Device_Status = CONTINUOUSLY;
        ESP_LOGI("MODE", "Enter CONTINUOUS Mode");
    }
    else if (Device_Status == CONTINUOUSLY)
    {
        Device_Status = IDLE;
        ESP_LOGI("MODE", "Enter IDLE Mode");
        if (sensor_read_task_handle != NULL)
            vTaskDelete(sensor_read_task_handle);
        max30102_reset(&sensor);
    }
}

void printIntroduction(void)
{
    ESP_LOGI("Main", "Enter MAIN");
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI("CHIP", "This is %s chip with %d CPU core(s), %s%s%s%s",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT/" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI("CHIP", "Silicon revision v%d.%d", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGE("CHIP", "Get flash size failed");
        return;
    }

    ESP_LOGI("CHIP", "Got %ld MB %s flash", flash_size / (uint32_t)(1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI("CHIP", "Minimum free heap size: %ld bytes", esp_get_minimum_free_heap_size());
}

void NVS_StoreDataBase(void)
{
    ESP_LOGI("NVS", "NVS SAVING DATA...");

    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &data_base_handle);
    if (err != ESP_OK)
        ESP_LOGE("NVS", "Error at nvs_open: %s", esp_err_to_name(err));

    size_t required_size = 0;
    err = nvs_get_blob(data_base_handle, DATA_BASE_KEY, NULL, &required_size);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND))
        ESP_LOGE("NVS", "Error at nvs_get_blob: %s", esp_err_to_name(err));

    ESP_LOGI("NVS", "Check NVS Data Base, found blob of size %i bytes", required_size);

    if ((err == ESP_OK) && (required_size == sizeof(myDataBase)))
    {
        err = nvs_set_blob(data_base_handle, DATA_BASE_KEY, &myDataBase, sizeof(myDataBase));
        if (err != ESP_OK)
            ESP_LOGE("NVS", "Error at nvs_set_blob trying to save data: %s", esp_err_to_name(err));
    }

    // if ((err == ESP_ERR_NVS_NOT_FOUND) || (required_size == 0) || (required_size != sizeof(myDataBase)))
    // {
    //     ESP_LOGW("NVS", "First Time Initialize Data Base");
    //     err = nvs_set_blob(data_base_handle, DATA_BASE_KEY, &myDataBase, sizeof(myDataBase));
    //     if (err != ESP_OK)
    //         ESP_LOGE("NVS", "Error at nvs_set_blob: %s", esp_err_to_name(err));
    // }
    // if ((err == ESP_OK) && (required_size == sizeof(myDataBase)))
    // {
    //     ESP_LOGI("NVS", "Getting Data Base...");
    //     err = nvs_get_blob(data_base_handle, DATA_BASE_KEY, &myDataBase, &required_size);
    //     if (err == ESP_OK)
    //         ESP_LOGI("NVS", "Success!");
    //     else
    //         ESP_LOGE("NVS", "Error at nvs_set_blob to load data base: %s", esp_err_to_name(err));
    // }

    err = nvs_commit(data_base_handle);
    if (err != ESP_OK)
        ESP_LOGE("NVS", "Error at nvs_commit: %s", esp_err_to_name(err));

    nvs_close(data_base_handle);

    ESP_LOGI("NVS", "NVS DONE!");
}

// ===========================================================================================================================================================

void app_main(void)
{

    printIntroduction();

    // =========================================================================================================================================================================

    // ESP_ERROR_CHECK(bluetooth_init());

    // vTaskDelay(2000);

    // =========================================================================================================================================================================

    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(button_init(&button, BUTTON_GPIO, BUTTON_EDGE_FALLING, tskIDLE_PRIORITY + 5, configMINIMAL_STACK_SIZE * 4));

    button_add_cb(&button, BUTTON_CLICK_SINGLE, button_cb_press_once, NULL);
    button_add_cb(&button, BUTTON_CLICK_MEDIUM, button_cb_hold, NULL);
    button_add_cb(&button, BUTTON_CLICK_LONG, button_cb_hold, NULL);
    button_add_cb(&button, BUTTON_CLICK_DOUBLE, button_cb_press_twice, NULL);

    max30102_init(&sensor, 0, MAX_SDA, MAX_SCL, 400000);
    max30102_reset(&sensor);

    i2c_master_init(&display, DISPLAY_SDA, DISPLAY_SCL, GPIO_NUM_NC);
    ssd1306_init(&display, 128, 64);

    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
    ssd1306_display_text(&display, 0, "     MEDICAL    ", 16, false);
    ssd1306_display_text(&display, 1, "   SMART WATCH  ", 16, false);
    ssd1306_display_text_x3(&display, 4, "HELLO", 5, false);

    vTaskDelay(2000);

    ESP_ERROR_CHECK(bluetooth_init());

    vTaskDelay(2000);

    {
        esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &data_base_handle);
        if (err != ESP_OK)
            ESP_LOGE("NVS", "Error at nvs_open: %s", esp_err_to_name(err));
        size_t required_size = 0;
        err = nvs_get_blob(data_base_handle, DATA_BASE_KEY, NULL, &required_size);
        if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND))
            ESP_LOGE("NVS", "Error at nvs_get_blob: %s", esp_err_to_name(err));
        ESP_LOGI("NVS", "Check NVS Data Base, found blob of size %i bytes", required_size);
        if ((err == ESP_ERR_NVS_NOT_FOUND) || (required_size == 0) || (required_size != sizeof(myDataBase)))
        {
            ESP_LOGW("NVS", "First Time Initialize Data Base");
            err = nvs_set_blob(data_base_handle, DATA_BASE_KEY, &myDataBase, sizeof(myDataBase));
            if (err != ESP_OK)
                ESP_LOGE("NVS", "Error at nvs_set_blob: %s", esp_err_to_name(err));
        }
        if ((err == ESP_OK) && (required_size == sizeof(myDataBase)))
        {
            ESP_LOGI("NVS", "Getting Data Base...");
            err = nvs_get_blob(data_base_handle, DATA_BASE_KEY, &myDataBase, &required_size);
            if (err == ESP_OK)
                ESP_LOGI("NVS", "Success!");
            else
                ESP_LOGE("NVS", "Error at nvs_set_blob to load data base: %s", esp_err_to_name(err));
        }

        err = nvs_commit(data_base_handle);
        if (err != ESP_OK)
            ESP_LOGE("NVS", "Error at nvs_commit: %s", esp_err_to_name(err));

        nvs_close(data_base_handle);

        ESP_LOGI("NVS", "NVS DONE!");
    }

    {
        time_t t = mktime(&myLocalTime);
        // ESP_LOGI("TIME", "Setting time to %s", asctime(&myLocalTime));
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);
        setenv("TZ", "ICT", 1);
        tzset();
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &myLocalTime);
        ESP_LOGI("TIME", "The current date/time is: %s", strftime_buf);
    }

    {
        xQueue_HR = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE_HR, ucQueueStorageArea_HR, &xStaticQueue_HR);
        configASSERT(xQueue_HR);
        xQueue_SpO2 = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE_SPO2, ucQueueStorageArea_SpO2, &xStaticQueue_SpO2);
        configASSERT(xQueue_SpO2);
    }

    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(displayTask, "Display", 8192, NULL, tskIDLE_PRIORITY + 15, &display_task_handle, 0U);
    if (ret != pdPASS)
        ESP_LOGE("Task", "FAIL Display Task");

    ESP_LOGI("MAIN", "Device Running Norminal");

    while (1)
    {
        gpio_set_level(BLINK_GPIO, (uint32_t)s_led_state);
        // ESP_LOGI("Blink", "Blinking!");
        if (bluetooth_connected == true)
            led_blink_periodMS = 1000;
        else
            led_blink_periodMS = 3000;
        if (s_led_state == false)
            vTaskDelay(100);
        else
            vTaskDelay(led_blink_periodMS);
        s_led_state = !s_led_state;
        time_t now;
        time(&now);
        localtime_r(&now, &myLocalTime);
    }
}

void sensorReadTask(void *pvParameters)
{
    ESP_LOGI("Task", "Enter read task MAX30102");
    BaseType_t ret;

    sensor.HeartRate = 0U;
    sensor.SpO2 = 0.0;
    sensor.Temperature = 0.0;

    uint8_t device_id = 0x00U;
    while (device_id != 0x15U)
    {
        max30102_reset(&sensor);
        vTaskDelay(500);
        max30102_read(&sensor, 0xFFU, &device_id, 1U);
        ESP_LOGI("Sensor", "MAX30102 Device ID = 0x%2X", device_id);
    };
    // ESP_LOGI("Sensor", "MAX30102 Device ID = 0x%2X", device_id);

    max30102_clear_fifo(&sensor);
    // Enter SpO2 mode
    max30102_set_mode(&sensor, max30102_spo2);
    max30102_set_fifo_config(&sensor, max30102_smp_ave_4, 1, 12);
    // max30102_set_a_full(&sensor, 1);

    // Sensor settings
    max30102_set_led_pulse_width(&sensor, max30102_pw_17_bit);
    max30102_set_adc_resolution(&sensor, max30102_adc_2048);
    max30102_set_sampling_rate(&sensor, max30102_sr_100);
    // RED
    max30102_set_led_current_1(&sensor, 6.2);
    // IR
    max30102_set_led_current_2(&sensor, 7.2);

    ESP_LOGI("Sensor", "MAX30102 Configured Successfully");

    max30102_clear_fifo(&sensor);
    // max30102_interrupt_handler(&sensor);

    is_HR_Data_Point_Valid = false;
    is_SpO2_Data_Point_Valid = false;

    while (1)
    {
        vTaskDelay(200);
        // max30102_interrupt_handler(&sensor);
        max30102_read_fifo(&sensor);
        if (sensor.status_flag == max30102_data_ready)
        {
            ESP_LOGI("Sensor", "Gather enough data -> Processing");
            sensor.status_flag = max30102_processing;
            ret = xTaskCreatePinnedToCore(sensorProcessTask, "Sensor Process", 12288, NULL, tskIDLE_PRIORITY + 5, &sensor_process_task_handle, 1U);
            if (ret != pdPASS)
                ESP_LOGE("Task", "FAIL Sensor Task");
        }
    }
}

void sensorProcessTask(void *pvParameters)
{
    // PROCESSING START
    max30102_process(&sensor);
    if (sensor.status_flag == max30102_processing)
    {
        max30102_get_heart_rate(&sensor);
        if (sensor.pearson_correlation >= CORRELATION_RATIO)
            max30102_get_spo2(&sensor);
        else
        {
            ESP_LOGW("PROCESS", "Dirty Data -> Not Process");
            sensor.SpO2 = 0.0;
        }
    }
    sensor.status_flag = max30102_process_done;

    // PROCESSING DONE

    if ((sensor.HeartRate >= 50) && (sensor.HeartRate <= 180))
    {
        ESP_LOGI("PROCESS", "Valid -> Send to HR Queue");
        xQueueSend(xQueue_HR, (void *)&sensor.HeartRate, (TickType_t)10);
        HR_continuous_invalid_sample_count = 0;
    }
    else
    {
        HR_continuous_invalid_sample_count++;
        if (HR_continuous_invalid_sample_count >= 3)
        {
            HR_Now = 0;
            is_HR_Data_Point_Valid = false;
        }
    }

    if ((sensor.SpO2 >= 90.0) && (sensor.SpO2 <= 100.0))
    {
        ESP_LOGI("PROCESS", "Valid -> Send to SpO2 Queue");
        xQueueSend(xQueue_SpO2, (void *)&sensor.SpO2, (TickType_t)10);
        SpO2_continuous_invalid_sample_count = 0;
    }
    else
    {
        SpO2_continuous_invalid_sample_count++;
        if (SpO2_continuous_invalid_sample_count >= 3)
        {
            SpO2_Now = 0.0;
            is_SpO2_Data_Point_Valid = false;
        }
    }

    if ((uxQueueSpacesAvailable(xQueue_HR) <= (QUEUE_LENGTH - FILTER_AVR_SAMPLE)) && ((is_HR_Data_Point_Valid == false) || (Device_Status == CONTINUOUSLY)))
    {
        uint8_t num = QUEUE_LENGTH - uxQueueSpacesAvailable(xQueue_HR);
        uint32_t sum = 0;
        for (uint8_t i = 0; i < num; i++)
        {
            uint8_t temp = 0;
            xQueueReceive(xQueue_HR, &temp, (TickType_t)10);
            sum += temp;
        }
        sum /= num;
        HR_Now = (uint8_t)sum;
        if (uxQueueSpacesAvailable(xQueue_HR) != QUEUE_LENGTH)
        {
            ESP_LOGW("PROCESS", "HR Queue not empty!");
            is_HR_Data_Point_Valid = false;
        }
        else
        {
            ESP_LOGI("PROCESS", "HR Data Point!");
            is_HR_Data_Point_Valid = true;
            if (is_Time_Sync == true)
            {
                uint32_t reqTimeStamp = myLocalTime.tm_hour * 60 + myLocalTime.tm_min;
                reqTimeStamp /= DATA_BASE_SAMPLE_PERIOD_MINS;
                Data_Storage_t dataStored = {
                    .HR = (myDataBase[reqTimeStamp].HR == 0) ? (HR_Now) : ((myDataBase[reqTimeStamp].HR + HR_Now) / 2),
                    .SpO2 = myDataBase[reqTimeStamp].SpO2};
                myDataBase[reqTimeStamp] = dataStored;
            }
        }
        xQueueReset(xQueue_HR);
    }

    if ((uxQueueSpacesAvailable(xQueue_SpO2) <= (QUEUE_LENGTH - FILTER_AVR_SAMPLE)) && ((is_SpO2_Data_Point_Valid == false) || (Device_Status == CONTINUOUSLY)))
    {
        uint8_t num = QUEUE_LENGTH - uxQueueSpacesAvailable(xQueue_SpO2);
        double sum = 0.0;
        for (uint8_t i = 0; i < num; i++)
        {
            double temp = 0;
            xQueueReceive(xQueue_SpO2, &temp, (TickType_t)10);
            sum += temp;
        }
        sum /= num;
        SpO2_Now = sum;
        if (uxQueueSpacesAvailable(xQueue_SpO2) != QUEUE_LENGTH)
        {
            ESP_LOGW("PROCESS", "SpO2 Queue not empty!");
            is_SpO2_Data_Point_Valid = false;
        }
        else
        {
            ESP_LOGI("PROCESS", "SpO2 Data Point!");
            is_SpO2_Data_Point_Valid = true;
            if (is_Time_Sync == true)
            {
                uint32_t reqTimeStamp = myLocalTime.tm_hour * 60 + myLocalTime.tm_min;
                reqTimeStamp /= DATA_BASE_SAMPLE_PERIOD_MINS;
                Data_Storage_t dataStored = {
                    .HR = myDataBase[reqTimeStamp].HR,
                    .SpO2 = (myDataBase[reqTimeStamp].SpO2 == 0.0) ? ((uint16_t)(SpO2_Now * 10.0)) : (((uint16_t)(SpO2_Now * 10.0) + myDataBase[reqTimeStamp].SpO2) / 2)};
                myDataBase[reqTimeStamp] = dataStored;
            }
        }
        xQueueReset(xQueue_SpO2);
    }

    if ((is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true) && (Device_Status == SINGLE))
    {
        ESP_LOGI("PROCESS", "SINGLE DONE!");
        sensor.status_flag = max30102_gather_data;

        if (is_Time_Sync == true)
        {
            uint32_t reqTimeStamp = myLocalTime.tm_hour * 60 + myLocalTime.tm_min;
            if (reqTimeStamp % 2 == 1)
                reqTimeStamp++;
            reqTimeStamp /= DATA_BASE_SAMPLE_PERIOD_MINS;
            Data_Storage_t dataStored = {
                .HR = HR_Now,
                .SpO2 = (uint16_t)(SpO2_Now * 10.0)};
            myDataBase[reqTimeStamp] = dataStored;
            NVS_StoreDataBase();
            ESP_LOGI("PROCESS", "Stored data at timestamp %02d:%02d, store at index %ld", myLocalTime.tm_hour, myLocalTime.tm_min, reqTimeStamp);
            if (sensor_read_task_handle != NULL)
                vTaskDelete(sensor_read_task_handle);
            max30102_reset(&sensor);
            if (sensor_process_task_handle != NULL)
                vTaskDelete(sensor_process_task_handle);
        }
    }

    if ((is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true) && (Device_Status == CONTINUOUSLY) && (bluetooth_connected == true) && (is_Time_Sync == true))
    {
        ESP_LOGI("BLUETOOTH", "Update New Data to App");
        char tempString[30] = {0};
        sprintf(tempString, "UPDATE,%d,%.1f:%c", HR_Now, SpO2_Now, '\0');
        ESP_LOGI("BLUETOOTH", "Send Update to Phone %s", tempString);
        esp_spp_write(write.handle, strlen(tempString), (uint8_t *)tempString);
    }

    if (Device_Status == CONTINUOUSLY)
    {
        ESP_LOGI("PROCESS", "CONTINUOUS!");
        if (is_Time_Sync == true)
        {
            if (((myLocalTime.tm_min % 2) == 0) && (is_Data_Saved == false))
            {
                ESP_LOGI("TIME", "Time to save data");
                uint32_t reqTimeStamp = myLocalTime.tm_hour * 60 + myLocalTime.tm_min;
                reqTimeStamp /= DATA_BASE_SAMPLE_PERIOD_MINS;
                Data_Storage_t dataStored = {
                    .HR = HR_Now,
                    .SpO2 = (uint16_t)(SpO2_Now * 10.0)};
                myDataBase[reqTimeStamp] = dataStored;
                NVS_StoreDataBase();
                ESP_LOGI("PROCESS", "Stored data at timestamp %02d:%02d, store at index %ld", myLocalTime.tm_hour, myLocalTime.tm_min, reqTimeStamp);
                is_Data_Saved = true;
            }
            else if ((myLocalTime.tm_min % 2) == 1)
                is_Data_Saved = false;
        }
    }

    sensor.status_flag = max30102_gather_data;

    if (sensor_process_task_handle != NULL)
        vTaskDelete(sensor_process_task_handle);
}

void displayTask(void *pvParameters)
{
    ssd1306_clear_screen(&display, false);
    ssd1306_display_text(&display, 0, "     MEDICAL    ", 16, false);
    ssd1306_display_text(&display, 1, "   SMART WATCH  ", 16, false);
    // ssd1306_display_text(&display, 2, "                ", 16, false);
    ssd1306_display_text(&display, 3, "  DOUBLE CLICK  ", 16, false);
    ssd1306_display_text(&display, 4, "   -> SINGLE    ", 16, false);
    // ssd1306_display_text(&display, 5, "                ", 16, false);
    ssd1306_display_text(&display, 6, "      HOLD      ", 16, false);
    ssd1306_display_text(&display, 7, " -> CONTINUOUS  ", 16, false);

    while (1)
    {
        switch (Device_Status)
        {
        case IDLE:
        {
            ssd1306_display_text(&display, 0, "     MEDICAL    ", 16, false);
            ssd1306_display_text(&display, 1, "   SMART WATCH  ", 16, false);
            ssd1306_display_text(&display, 2, "                ", 16, false);
            ssd1306_display_text(&display, 3, "  DOUBLE CLICK  ", 16, false);
            ssd1306_display_text(&display, 4, "   -> SINGLE    ", 16, false);
            ssd1306_display_text(&display, 5, "                ", 16, false);
            ssd1306_display_text(&display, 6, "      HOLD      ", 16, false);
            ssd1306_display_text(&display, 7, " -> CONTINUOUS  ", 16, false);
            break;
        }
        case SINGLE:
        {
            char tempString0[40] = "";
            char tempString1[40] = "";
            char tempString2[40] = "";
            // char tempString3[30] = "";

            if (is_Time_Sync == true)
                sprintf(tempString0, "%02d:%02d %02d/%02d/%4d", myLocalTime.tm_hour, myLocalTime.tm_min, myLocalTime.tm_mday, myLocalTime.tm_mon + 1, myLocalTime.tm_year + 1900);
            else
                sprintf(tempString0, "      ----      ");

            if (is_HR_Data_Point_Valid == false)
                sprintf(tempString1, "    --- BPM     ");
            else
                sprintf(tempString1, "    %3d BPM     ", HR_Now);

            if (is_SpO2_Data_Point_Valid == false)
                sprintf(tempString2, "     ---- %%     ");
            else
                sprintf(tempString2, "     %2.1f %%     ", SpO2_Now);

            // if ((is_HR_Data_Point_Valid == true) && (is_SpO2_Data_Point_Valid == true))
            //     sprintf(tempString3, "Single    Return");
            // else
            //     sprintf(tempString3, "Double     Abort");

            ssd1306_display_text(&display, 0, "     SINGLE     ", 16, false);
            ssd1306_display_text(&display, 1, tempString0, 16, false);
            ssd1306_display_text(&display, 2, "                ", 16, false);
            ssd1306_display_text(&display, 3, "   HEART RATE   ", 16, false);
            ssd1306_display_text(&display, 4, tempString1, 16, false);
            ssd1306_display_text(&display, 5, "                ", 16, false);
            ssd1306_display_text(&display, 6, "      SPO2      ", 16, false);
            ssd1306_display_text(&display, 7, tempString2, 16, false);
            // ssd1306_display_text(&display, 6, tempString3, 16, false);
            // ssd1306_display_text(&display, 7, "Hold  Continuous", 16, false);

            break;
        }
        case CONTINUOUSLY:
        {
            char tempString0[40] = "";
            char tempString1[40] = "";
            char tempString2[40] = "";

            if (is_Time_Sync == true)
                sprintf(tempString0, "%02d:%02d %02d/%02d/%4d", myLocalTime.tm_hour, myLocalTime.tm_min, myLocalTime.tm_mday, myLocalTime.tm_mon + 1, myLocalTime.tm_year + 1900);
            else
                sprintf(tempString0, "      ----      ");

            if (is_HR_Data_Point_Valid == false)
                sprintf(tempString1, "    --- BPM     ");
            else
                sprintf(tempString1, "    %3d BPM     ", HR_Now);

            if (is_SpO2_Data_Point_Valid == false)
                sprintf(tempString2, "     ---- %%     ");
            else
                sprintf(tempString2, "     %2.1f %%     ", SpO2_Now);

            ssd1306_display_text(&display, 0, "   CONTINUOUS   ", 16, false);
            ssd1306_display_text(&display, 1, tempString0, 16, false);
            ssd1306_display_text(&display, 2, "   HEART RATE   ", 16, false);
            ssd1306_display_text(&display, 3, tempString1, 16, false);
            ssd1306_display_text(&display, 4, "                ", 16, false);
            ssd1306_display_text(&display, 5, "      SPO2      ", 16, false);
            ssd1306_display_text(&display, 6, tempString2, 16, false);
            ssd1306_display_text(&display, 7, "                ", 16, false);
            // ssd1306_display_text(&display, 7, "Single    Return", 16, false);
            break;
        }
        case SLEEP:
        {
            ssd1306_contrast(&display, 0U);
            ssd1306_clear_screen(&display, false);
            vTaskSuspend(display_task_handle);
            break;
        }
        default:
        {
            break;
        }
        }
        vTaskDelay(100);
    }
}

void tranferAllTask(void *pvParameters)
{
    int num = sizeof(myDataBase) / sizeof(Data_Storage_t);
    for (int i = 0; i < num; i++)
    {
        char tempString[20] = {0};
        Data_Storage_t dataStored = myDataBase[i];
        sprintf(tempString, "DATA,%d,%d,%.1f:%c", i, dataStored.HR, (double)dataStored.SpO2 / 10.0, '\0');
        bluetooth_sent = false;
        esp_spp_write(write.handle, strlen(tempString), (uint8_t *)tempString);
        while (bluetooth_sent == false)
            vTaskDelay(1);
        vTaskDelay(5);
    }
    ESP_LOGI("BLUETOOTH", "DONE tranfer ALL the data!");
    vTaskDelete(tranfer_all_task_handle);
}

// ===========================================================================================================================================================

void bluetooth_data_recv_cb(esp_spp_cb_param_t *param, char *data, int len)
{
    memcpy(&write, param, sizeof(struct spp_write_evt_param));

    if (strstr((const char *)data, (const char *)"WhoAreYou?") != NULL)
    {
        esp_spp_write(param->write.handle, 10U, (uint8_t *)whoIam);
        ESP_LOGI("BLUETOOTH", "Response to Phone %s", whoIam);
    }

    if (strstr((const char *)data, (const char *)"TIME") != NULL)
    {
        sscanf(data, "TIME,%d,%d,%d,%d,%d,%d!", &myLocalTime.tm_year, &myLocalTime.tm_mon, &myLocalTime.tm_mday, &myLocalTime.tm_hour, &myLocalTime.tm_min, &myLocalTime.tm_sec);
        myLocalTime.tm_mon--;
        myLocalTime.tm_year -= 1900;
        {
            time_t t = mktime(&myLocalTime);
            char strftime_buf[64];
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &myLocalTime);
            ESP_LOGI("TIME", "The current date/time is: %s", strftime_buf);
            struct timeval now = {.tv_sec = t};
            settimeofday(&now, NULL);
            is_Time_Sync = true;
        }
    }

    if (strstr((const char *)data, (const char *)"DATA") != NULL)
    {
        uint32_t reqTimeStamp = 0;
        sscanf(data, "DATA,%ld?", &reqTimeStamp);
        uint8_t hour = (uint8_t)(reqTimeStamp / 100);
        uint8_t min = (uint8_t)(reqTimeStamp % 100);

        reqTimeStamp = (reqTimeStamp / 100) * 60 + (reqTimeStamp % 100);
        reqTimeStamp /= DATA_BASE_SAMPLE_PERIOD_MINS;

        ESP_LOGI("BLUETOOTH", "Requested data at timestamp %02d:%02d, store at index %ld", hour, min, reqTimeStamp);

        if (reqTimeStamp < (sizeof(myDataBase) / sizeof(Data_Storage_t)))
        {
            Data_Storage_t dataStored = myDataBase[reqTimeStamp];
#if ALLOW_VALUE_0 == 0
            if ((dataStored.HR > 0) || (dataStored.SpO2 > 0))
            {
#endif
                char tempString[20] = {0};
                sprintf(tempString, "DATA,%ld,%d,%.1f:%c", reqTimeStamp, dataStored.HR, (double)dataStored.SpO2 / 10.0, '\0');
                ESP_LOGI("BLUETOOTH", "Response to Phone %s", tempString);
                esp_spp_write(param->write.handle, strlen(tempString), (uint8_t *)tempString);
#if ALLOW_VALUE_0 == 0
            }
#endif
        }
    }

    if (strstr((const char *)data, (const char *)"ALL!") != NULL)
    {
        ESP_LOGI("BLUETOOTH", "Response to Phone ALL the data!");
        BaseType_t ret = xTaskCreatePinnedToCore(tranferAllTask, "Sensor Process", 4096, NULL, tskIDLE_PRIORITY + 5, &tranfer_all_task_handle, 1U);
        if (ret != pdPASS)
            ESP_LOGE("Task", "FAIL Tranfer All Task");
    }
}

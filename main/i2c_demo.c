#include <stdbool.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "lcd_display.h"

#include "i2c_demo.h"

#define TEST_I2C_PORT -1 // Autoselect (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html#_CPPv4N23i2c_master_bus_config_t8i2c_portE)
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21

#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define ACC_LOG_TAG "Accelerometer"
#define ACC_ADDRESS 0x4C

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t accelerometer_handle;

QueueHandle_t xAccelerometerData;

typedef struct {
    float x, y, z;
} coordinates_t;

void app_main(void) {
    add_i2c_master();
    add_accelerometer();
    add_lcd_display();

    xAccelerometerData = xQueueCreate(
        10,
        sizeof(coordinates_t)
    );

    if (xAccelerometerData == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    }

    TaskHandle_t readLoopHandle = NULL;
    TaskHandle_t displayLoopHandle = NULL;
    
    //x = get_x();
    //y = get_y();
    //z = get_z();
    //printf("X: %d, Y: %d, Z: %d\n", x, y, z);

    xTaskCreate(
        read_loop,
        "Accelerometer data read",
        2048,
        (void*)1000,
        tskIDLE_PRIORITY,
        &readLoopHandle
    );

    xTaskCreate(
       display_loop,
       "Display updates",
       2048,
       (void*)1000,
       tskIDLE_PRIORITY,
       &displayLoopHandle
    );
}

void add_i2c_master(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_LOGI("I2C Master", "Successfully added I2C master");
}

void add_lcd_display(void)
{
    lcd_init(bus_handle, LCD_ADDR, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, LCD_COLS, LCD_ROWS);
    ESP_LOGI("LCD Display", "Added display");
    lcd_clear();
    lcd_send_string("Ready to use");
    lcd_set_cur(1, 0);
    lcd_send_string("================");
}

void add_accelerometer(void)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ACC_ADDRESS,
        .scl_speed_hz = 100000,
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &accelerometer_handle));
    ESP_LOGI(ACC_LOG_TAG, "Successfully added I2C accelerometer");

    uint8_t samples_register = 0x08;
    uint8_t sample_selection = 0b00110100;
    uint8_t data_to_transmit[2] = {samples_register, sample_selection};
    ESP_ERROR_CHECK(i2c_master_transmit(accelerometer_handle, data_to_transmit, 2, -1));
    ESP_LOGI(ACC_LOG_TAG, "Successfully setup sample rate");

    uint8_t set_mode_register = 0x07;
    uint8_t enable_active_mode = 0x01;
    data_to_transmit[0] = set_mode_register;
    data_to_transmit[1] = enable_active_mode;
    ESP_ERROR_CHECK(i2c_master_transmit(accelerometer_handle, data_to_transmit, 2, -1));
    ESP_LOGI(ACC_LOG_TAG, "Successfully set accelerometer to active mode");

}

float receive_coordinate(uint8_t* read_register) {
    uint8_t raw_byte;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(accelerometer_handle, read_register, 1, &raw_byte, 1, -1));
    int8_t parsed_coordinate = raw_byte & 0x3F;
    if (parsed_coordinate >> 5) {
        parsed_coordinate = parsed_coordinate | -64;
    }
    //return ((raw_byte & 0x3F) >> 4) ? -(int16_t)parsed_coordinate : (int16_t)parsed_coordinate;
    return parsed_coordinate / 9.8;
}

float get_x(void) {
    uint8_t read_register = 0x00;
    return receive_coordinate(&read_register);
}

float get_y(void) {
    uint8_t read_register = 0x01;
    return receive_coordinate(&read_register);
}

float get_z(void) {
    uint8_t read_register = 0x02;
    return receive_coordinate(&read_register);
}

void read_loop(void *params)
{
    float x, y, z;
    uint32_t delay_in_milis = (uint32_t)params;
    ESP_LOGI(ACC_LOG_TAG, "Initiated read loop");

    while (1) {
        vTaskDelay(delay_in_milis / portTICK_PERIOD_MS);
        x = get_x();
        y = get_y();
        z = get_z();
        coordinates_t xyz = {.x = x, .y = y, .z = z};
        ESP_LOGI(ACC_LOG_TAG, "X: %f, Y: %f, Z: %f\n", xyz.x, xyz.y, xyz.z);
        xQueueSend(xAccelerometerData,
            (void*)&xyz, (TickType_t)0);
    }
}

void display_loop(void *params)
{
    uint32_t delay_in_milis = (uint32_t)params;
    lcd_clear();
    char text_buffer[6];
    coordinates_t read_coords = {0};
    while (1) {
        if (xQueueReceive(xAccelerometerData, &read_coords, (TickType_t)10) == pdPASS) {
            lcd_set_cur(0, 1);
            sprintf(text_buffer, "X:%1.1f", read_coords.x);
            lcd_send_string(text_buffer);

            lcd_set_cur(0, 8);
            sprintf(text_buffer, "Y:%1.1f", read_coords.y);
            lcd_send_string(text_buffer);

            lcd_set_cur(1, 1);
            sprintf(text_buffer, "Z:%1.1f", read_coords.z);
            lcd_send_string(text_buffer);

            vTaskDelay(delay_in_milis / portTICK_PERIOD_MS);
        }
    }
}
#include <stdio.h>
#include <unistd.h>

#include "esp_check.h"
#include "driver/i2c_master.h"

#define A2 1
#define A1 1
#define A0 1
#define ADDRESS (A2 << 2 | A1 << 1 | A0)
#define LCD_ADDRESS (0b01000000 >> 1) | ADDRESS

// LCD instructions
#define LCD_CLEAR 0x01             // replace all characters with ASCII 'space'
#define LCD_HOME 0x02              // return cursor to first position on first line
#define LCD_ENTRY_MODE 0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF 0x08       // turn display off
#define LCD_DISPLAY_ON 0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET 0x30    // reset the LCD
#define LCD_FUNCTION_SET_4BIT 0x28 // 4-bit data, 2-line display, 5 x 7 font

#define TAG "LCD Display"

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

static i2c_master_dev_handle_t dev_handle;

static esp_err_t i2c_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_ADDRESS,
        .scl_speed_hz = 100000,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

esp_err_t lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data[0] = data_u | 0x0C;
    data[1] = data_u | 0x08;
    data[2] = data_l | 0x0C;
    data[3] = data_l | 0x08;

    return i2c_master_transmit(dev_handle, data, 4, 1000);
    //if (err != 0) ESP_LOGI(TAG, "Error no. %d in command", err);
}

esp_err_t lcd_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_transmit[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_transmit[0] = data_u|0x0D;  //en=1, rs=0
	data_transmit[1] = data_u|0x09;  //en=0, rs=0
	data_transmit[2] = data_l|0x0D;  //en=1, rs=0
	data_transmit[3] = data_l|0x09;  //en=0, rs=0
	return i2c_master_transmit(dev_handle, data_transmit, 4, 1000);
	//if (err!=0) ESP_LOGI(TAG, "Error in sending data");
}

esp_err_t lcd_init(i2c_master_bus_handle_t bus_handle,
    uint8_t addr,
    uint8_t dataPin,
    uint8_t clockPin,
    uint8_t cols,
    uint8_t rows)
{
    // I2C Initialization
    ESP_RETURN_ON_ERROR(i2c_init(bus_handle), TAG, "I2C Initialization failed");

    // 4 bit initialisation
	usleep(50000);  // wait for >40ms
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_FUNCTION_RESET), TAG, "Command sending failed");
	usleep(4500);  // wait for >4.1ms
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_FUNCTION_RESET), TAG, "Command sending failed");
	usleep(200);  // wait for >100us
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_FUNCTION_RESET), TAG, "Command sending failed");
	usleep(200);
	ESP_RETURN_ON_ERROR(lcd_send_cmd (0x20), TAG, "Command sending failed");  // 4bit mode
	usleep(200);

    // dislay initialisation
    ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_FUNCTION_SET_4BIT), TAG, "Command sending failed"); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	usleep(1000);
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_DISPLAY_OFF), TAG, "Command sending failed"); //Display on/off control --> D=0,C=0, B=0  ---> display off
	usleep(1000);
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_CLEAR), TAG, "Command sending failed");  // clear display
	usleep(1000);
	usleep(1000);
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_ENTRY_MODE), TAG, "Command sending failed"); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	usleep(1000);
	ESP_RETURN_ON_ERROR(lcd_send_cmd (LCD_DISPLAY_ON), TAG, "Command sending failed"); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	usleep(2000);

    return ESP_OK;
}

esp_err_t lcd_set_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    ESP_RETURN_ON_ERROR(lcd_send_cmd(col), TAG, "Putting cursor failed");
    return ESP_OK;
}

esp_err_t lcd_send_string (char *str)
{
	while (*str) ESP_RETURN_ON_ERROR(lcd_send_data(*str++), TAG, "Sending string failed");
    return ESP_OK;
}

esp_err_t lcd_clear (void)
{
	ESP_RETURN_ON_ERROR(lcd_send_cmd(LCD_CLEAR), TAG, "Clear failed");
	usleep(5000);
    return ESP_OK;
}
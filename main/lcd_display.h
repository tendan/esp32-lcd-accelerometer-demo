#ifndef HD44780_H
#define HD44780_H

esp_err_t lcd_send_cmd(char cmd);

esp_err_t lcd_send_data(char data);

esp_err_t lcd_init(i2c_master_bus_handle_t bus_handle,
    uint8_t addr,
    uint8_t dataPin,
    uint8_t clockPin,
    uint8_t cols,
    uint8_t rows);

esp_err_t lcd_set_cur(int row, int col);

esp_err_t lcd_send_string (char *str);

esp_err_t lcd_clear (void);

#endif /* HD44780_H */
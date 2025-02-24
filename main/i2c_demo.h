#ifndef I2C_DEMO_H
#define I2C_DEMO_H

void add_i2c_master(void);
void add_lcd_display(void);
void add_accelerometer(void);

float receive_coordinate(uint8_t* read_register);
float get_x(void);
float get_y(void);
float get_z(void);

void read_loop(void *params);
void display_loop(void *params);

#endif /* I2C_DEMO_H */
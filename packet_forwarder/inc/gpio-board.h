#ifndef __GPIO_BOARD_H_
#define __GPIO_BOARD_H_

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

#define POUT            4   /* P1-07 */
#define BUFFER_MAX      3
#define DIRECTION_MAX   48



int gpio_export(int pin);
int gpio_unexport(int pin);
int gpio_direction(int pin, int dir);
int gpio_write(int pin, int value);
int gpio_read(int pin);

#endif

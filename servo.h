#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_PIN PB2

// Timer1 ticks at 4 us per count (prescaler 64 @ 16 MHz)
#define SERVO_RIGHT   150   // 600 us pulse (~ -90 deg, looking right)
#define SERVO_FRONT   350   // 1400 us pulse (   0 deg, looking forward)
#define SERVO_LEFT    625   // 2500 us pulse (~ +90 deg, looking left)

void setup_servo(void);
uint8_t move_servo(int pulseWidth);
uint8_t move_servo_smart(int pulseWidth, uint8_t *wait_ms);
uint8_t servo_at_position(int pulseWidth);
int get_servo_position(void);

#endif

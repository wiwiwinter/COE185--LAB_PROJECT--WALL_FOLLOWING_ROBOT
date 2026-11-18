#ifndef MOTORS_H
#define MOTORS_H

#include <avr/io.h>

extern const double original_speed;
extern const double distance_threshold; // in inches (20.0 cm / 2.54)
extern double       percent_speed;

#define FORWARD   1
#define BACKWARD -1

#define MOTOR_LEFT   0
#define MOTOR_RIGHT  1

// Scales normalized speed [0.0, 1.0] to Timer0 8-bit PWM duty cycle [0, MOTOR_TOP_SPEED].
#define MOTOR_TOP_SPEED 200

#define LEFT_CONTROL  PD2  // Left motor direction
#define LEFT_PWM      PD5  // Left motor PWM (OC0B)
#define RIGHT_CONTROL PD4  // Right motor direction
#define RIGHT_PWM     PD6  // Right motor PWM (OC0A)

void enable_motors(void);
void setup_motors(void);
void set_motor_speed(int motor, double speed, int direction);
void follow_wall(double power_percentage, double maintain_distance, int side);

#endif

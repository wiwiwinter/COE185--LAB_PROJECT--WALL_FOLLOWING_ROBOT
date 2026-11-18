#include "motors.h"
#include "ultrasonic.h"
#include "servo.h"
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

const double original_speed = 0.30;
const double distance_threshold = (20.0 / 2.54);
double percent_speed = 0.30;

void enable_motors(void)
{
    DDRD |= (1 << LEFT_CONTROL);   // PD2
    DDRD |= (1 << LEFT_PWM);       // PD5
    DDRD |= (1 << RIGHT_CONTROL);  // PD4
    DDRD |= (1 << RIGHT_PWM);      // PD6
}

// Configure Timer0 for Fast PWM (prescaler 64, ~976.5 Hz at 16 MHz) on OC0A (PD6) and OC0B (PD5).
void setup_motors(void)
{
    enable_motors();

    TCCR0A = (1 << WGM01) | (1 << WGM00);
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS01) | (1 << CS00);

    OCR0A = 0; // Right motor (PD6)
    OCR0B = 0; // Left motor (PD5)
}

// Left and right direction pins have opposite polarity due to physical motor mounting.
void set_motor_speed(int motor, double speed, int direction)
{
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;

    uint8_t pwm_value = (uint8_t)(speed * MOTOR_TOP_SPEED);
    if (pwm_value > MOTOR_TOP_SPEED) {
        pwm_value = MOTOR_TOP_SPEED;
    }

    if (motor == MOTOR_LEFT) {
        OCR0B = pwm_value; // PD5
        if (direction == FORWARD) {
            PORTD |= (1 << LEFT_CONTROL);   // PD2 HIGH = forward
        } else {
            PORTD &= ~(1 << LEFT_CONTROL);  // PD2 LOW = backward
        }
    }
    else if (motor == MOTOR_RIGHT) {
        OCR0A = pwm_value; // PD6
        if (direction == FORWARD) {
            PORTD &= ~(1 << RIGHT_CONTROL); // PD4 LOW = forward
        } else {
            PORTD |= (1 << RIGHT_CONTROL);  // PD4 HIGH = backward
        }
    }
}

void follow_wall(double power_percentage, double maintain_distance, int side)
{
    move_servo(side);
    _delay_ms(80);

    (void)measure_distance();
    _delay_ms(30);

    double distance = measure_distance();
    // Ignore out-of-range readings (>30 cm / ~11.81 in)
    if (distance <= 0.0 || distance > (30.0 / 2.54)) {
        distance = maintain_distance;
    }

    double left_speed, right_speed;
    if (side == MOTOR_RIGHT) {
        left_speed  = power_percentage * (distance / maintain_distance);
        right_speed = power_percentage * (maintain_distance / distance);
    }
    else {
        left_speed  = power_percentage * (distance / maintain_distance);
        right_speed = power_percentage * (maintain_distance / distance);
    }

    if (left_speed > 1.0) left_speed = 1.0;
    if (right_speed > 1.0) right_speed = 1.0;

    set_motor_speed(MOTOR_LEFT,  left_speed,  FORWARD);
    set_motor_speed(MOTOR_RIGHT, right_speed, FORWARD);

    _delay_ms(40);
}

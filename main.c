/*
 * main.c
 * Autonomous right-wall-following state machine using ultrasonic sensing,
 * servo scanning, and PD motor control on ATmega328P. Distances are in inches.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "motors.h"
#include "ultrasonic.h"
#include "servo.h"
#include "time.h"
#include "uart.h"

#define RIGHT_POS   SERVO_RIGHT
#define CENTER_POS  SERVO_FRONT
#define CM_TO_IN    2.54

// Distance thresholds (in inches)
#define DESIRED_DISTANCE_IN  9.0
#define TOO_CLOSE_IN         4.0
#define OBSTACLE_TH_IN       14.0
#define WALL_LOST_TH_IN      18.0
#define DISTANCE_TOLERANCE   2.5

// Speed settings (0.0 to 1.0 duty cycle ratio)
#define CRUISE_SPEED     0.25
#define MAX_SPEED        0.32    // Used when aligned within DISTANCE_TOLERANCE
#define MIN_SPEED        0.15    // Used during tight turns or front scans
#define TURN_SPEED       0.22

// PD controller gains for right-wall tracking
#define KP               0.035
#define KD               0.015
#define MAX_STEERING     0.15

#define SERVO_STABILIZE_MS  100
#define MEASUREMENT_SAMPLES 3

#define MIN_VALID_DISTANCE_IN 1.5
#define MAX_VALID_DISTANCE_IN 100.0
#define FILTER_WINDOW         3

// Loop timing and safety thresholds
#define FRONT_CHECK_INTERVAL_MS  150
#define RIGHT_READ_DELAY_MS      20
#define TURN_DURATION_MS         200
#define EMERGENCY_STOP_DIST      2.0

typedef enum {
    STATE_STARTUP,
    STATE_RUNNING,          // Continuous PD wall-following
    STATE_OBSTACLE_AVOID,   // Front blocked, pivoting left
    STATE_FIND_WALL,        // Right wall lost, curving right
    STATE_EMERGENCY_STOP    // Too close to wall, backing up and turning away
} robot_state_t;

static void motor_stop_all(void) {
    set_motor_speed(MOTOR_LEFT,  0.0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0.0, FORWARD);
}

static void motor_set_speeds(double left_pct, double right_pct) {
    if (left_pct < 0.0) left_pct = 0.0; if (left_pct > 1.0) left_pct = 1.0;
    if (right_pct < 0.0) right_pct = 0.0; if (right_pct > 1.0) right_pct = 1.0;
    set_motor_speed(MOTOR_LEFT, left_pct, FORWARD);
    set_motor_speed(MOTOR_RIGHT, right_pct, FORWARD);
}

static void motor_turn_left_inplace(double pct) {
    if (pct < 0.0) pct = 0.0; if (pct > 1.0) pct = 1.0;
    set_motor_speed(MOTOR_LEFT, pct, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, pct, FORWARD);
}

static double right_distance_history[FILTER_WINDOW] = {0};
static uint8_t history_index = 0;
static double previous_error = 0.0;

// Average two consecutive ultrasonic readings, discarding out-of-range samples.
static double take_measurement(void) {
    double d1 = measure_distance();
    _delay_ms(10);
    double d2 = measure_distance();
    
    uint8_t v1 = (d1 >= MIN_VALID_DISTANCE_IN && d1 <= MAX_VALID_DISTANCE_IN);
    uint8_t v2 = (d2 >= MIN_VALID_DISTANCE_IN && d2 <= MAX_VALID_DISTANCE_IN);
    
    if (v1 && v2) {
        return (d1 + d2) / 2.0;
    } else if (v1) {
        return d1;
    } else if (v2) {
        return d2;
    }
    return 0.0;
}

// Single ultrasonic sample used during brief front checks.
static double take_quick_measurement(void) {
    double d = measure_distance();
    if (d >= MIN_VALID_DISTANCE_IN && d <= MAX_VALID_DISTANCE_IN) {
        return d;
    }
    return 0.0;
}

// Smooth right-wall readings over FILTER_WINDOW samples; fall back to last valid average on 0.0.
static double apply_filter(double new_value) {
    if (new_value == 0.0) {
        double sum = 0.0;
        uint8_t count = 0;
        for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
            if (right_distance_history[i] > 0.0) {
                sum += right_distance_history[i];
                count++;
            }
        }
        return (count > 0) ? (sum / count) : DESIRED_DISTANCE_IN;
    }
    
    right_distance_history[history_index] = new_value;
    history_index = (history_index + 1) % FILTER_WINDOW;
    
    double sum = 0.0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
        if (right_distance_history[i] > 0.0) {
            sum += right_distance_history[i];
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : new_value;
}

// Positive output steers right (toward wall); negative steers left (away from wall).
static double calculate_steering(double right_distance) {
    double error = right_distance - DESIRED_DISTANCE_IN;
    double derivative = error - previous_error;
    double steering = (KP * error) + (KD * derivative);
    previous_error = error;
    
    if (steering > MAX_STEERING) steering = MAX_STEERING;
    if (steering < -MAX_STEERING) steering = -MAX_STEERING;
    
    return steering;
}

// Positive steering increases left wheel speed and reduces right wheel speed to curve right.
static void update_continuous_drive(double steering, double base_speed) {
    double left_speed = base_speed + steering;
    double right_speed = base_speed - steering;
    
    if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    
    motor_set_speeds(left_speed, right_speed);
}

int main(void) {
    setup_motors();
    setup_ultrasonic();
    setup_servo();
    init_time();
    uart_init(9600);

    sei();

    move_servo(RIGHT_POS);
    _delay_ms(500);

    uart_println("Starting continuous wall-following mode");
    uart_println("Target: 9 inches from right wall");

    robot_state_t state = STATE_STARTUP;
    double right_distance = DESIRED_DISTANCE_IN;
    double front_distance = 0.0;
    uint32_t last_front_check = 0;
    uint32_t now = 0;
    uint8_t bad_reading_count = 0;
    char debug_buf[64];

    motor_stop_all();
    _delay_ms(500);

    for (;;) {
        now = millis();
        
        switch (state) {
            case STATE_STARTUP:
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                right_distance = apply_filter(take_measurement());
                
                snprintf(debug_buf, sizeof(debug_buf), "Startup Right: %.2fin", right_distance);
                uart_println(debug_buf);
                
                if (right_distance > 0.0) {
                    uart_println("Initial reading OK, starting...");
                    state = STATE_RUNNING;
                    last_front_check = now;
                    motor_set_speeds(CRUISE_SPEED, CRUISE_SPEED);
                } else {
                    uart_println("Waiting for valid reading...");
                    _delay_ms(100);
                }
                break;

            case STATE_RUNNING:
                if (!servo_at_position(RIGHT_POS)) {
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                }
                
                {
                    double raw = take_measurement();
                    right_distance = apply_filter(raw);
                    
                    snprintf(debug_buf, sizeof(debug_buf), "R: %.2fin", right_distance);
                    uart_println(debug_buf);
                    
                    if (raw == 0.0) {
                        bad_reading_count++;
                        uart_println("Bad right reading!");
                        if (bad_reading_count > 3) {
                            uart_println("Wall lost - too many bad readings!");
                            state = STATE_FIND_WALL;
                            break;
                        }
                    } else {
                        bad_reading_count = 0;
                    }
                }
                
                if (right_distance > 0.0 && right_distance < EMERGENCY_STOP_DIST) {
                    uart_println("EMERGENCY: Too close to wall!");
                    state = STATE_EMERGENCY_STOP;
                    break;
                }
                
                if (right_distance > WALL_LOST_TH_IN) {
                    uart_println("Wall too far, seeking...");
                    state = STATE_FIND_WALL;
                    break;
                }
                
                // Periodic front obstacle scan every FRONT_CHECK_INTERVAL_MS
                if ((now - last_front_check) > FRONT_CHECK_INTERVAL_MS) {
                    motor_set_speeds(MIN_SPEED, MIN_SPEED);
                    
                    move_servo(CENTER_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                    
                    front_distance = take_quick_measurement();
                    snprintf(debug_buf, sizeof(debug_buf), "F: %.2fin", front_distance);
                    uart_println(debug_buf);
                    
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                    
                    last_front_check = now;
                    
                    if (front_distance > 0.0 && front_distance < OBSTACLE_TH_IN) {
                        snprintf(debug_buf, sizeof(debug_buf), "OBSTACLE at %.2fin!", front_distance);
                        uart_println(debug_buf);
                        state = STATE_OBSTACLE_AVOID;
                        break;
                    }
                }
                
                {
                    double steering = calculate_steering(right_distance);
                    double speed = CRUISE_SPEED;
                    double error_mag = right_distance - DESIRED_DISTANCE_IN;
                    if (error_mag < 0) error_mag = -error_mag;
                    
                    if (error_mag < DISTANCE_TOLERANCE) {
                        speed = MAX_SPEED;
                    } else if (error_mag > 3.0) {
                        speed = MIN_SPEED;
                    }
                    
                    update_continuous_drive(steering, speed);
                }
                
                _delay_ms(RIGHT_READ_DELAY_MS);
                break;

            case STATE_OBSTACLE_AVOID:
                motor_stop_all();
                _delay_ms(100);
                
                uart_println("Avoiding obstacle - turning left");
                
                motor_turn_left_inplace(TURN_SPEED);
                _delay_ms(TURN_DURATION_MS);
                motor_stop_all();
                _delay_ms(50);
                
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                front_distance = take_quick_measurement();
                snprintf(debug_buf, sizeof(debug_buf), "After turn F: %.2fin", front_distance);
                uart_println(debug_buf);
                
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                
                if (front_distance > 0.0 && front_distance < OBSTACLE_TH_IN) {
                    uart_println("Still blocked, turning more...");
                    break;
                }
                
                motor_set_speeds(CRUISE_SPEED, CRUISE_SPEED);
                _delay_ms(150);
                
                last_front_check = millis();
                uart_println("Resuming wall follow");
                state = STATE_RUNNING;
                break;

            case STATE_FIND_WALL:
                if (!servo_at_position(RIGHT_POS)) {
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                }
                
                uart_println("Seeking wall - curving right");
                
                motor_set_speeds(CRUISE_SPEED, MIN_SPEED);
                _delay_ms(200);
                
                right_distance = apply_filter(take_measurement());
                snprintf(debug_buf, sizeof(debug_buf), "Seeking R: %.2fin", right_distance);
                uart_println(debug_buf);
                
                if (right_distance > 0.0 && right_distance < WALL_LOST_TH_IN) {
                    uart_println("Wall found!");
                    bad_reading_count = 0;
                    last_front_check = millis();
                    state = STATE_RUNNING;
                }
                break;

            case STATE_EMERGENCY_STOP:
                motor_stop_all();
                uart_println("EMERGENCY - backing up");
                _delay_ms(100);
                
                set_motor_speed(MOTOR_LEFT, 0.18, BACKWARD);
                set_motor_speed(MOTOR_RIGHT, 0.18, BACKWARD);
                _delay_ms(200);
                motor_stop_all();
                _delay_ms(50);
                
                uart_println("Turning away from wall");
                motor_turn_left_inplace(TURN_SPEED);
                _delay_ms(150);
                motor_stop_all();
                _delay_ms(50);
                
                right_distance = apply_filter(take_measurement());
                snprintf(debug_buf, sizeof(debug_buf), "After correction R: %.2fin", right_distance);
                uart_println(debug_buf);
                
                last_front_check = millis();
                state = STATE_RUNNING;
                break;

            default:
                state = STATE_STARTUP;
                break;
        }
        
        _delay_ms(10);
    }

    return 0;
}

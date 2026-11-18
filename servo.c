#include "servo.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

static volatile int current_position = SERVO_FRONT;
static volatile uint8_t servo_moving = 0;

// Configure Timer1 in Fast PWM mode 15 (TOP = OCR1A = 4999) for 50 Hz servo control on OC1B (PB2).
void setup_servo(void)
{
    DDRB |= (1 << SERVO_PIN);

    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64 -> 250 kHz (4 us per tick)

    OCR1A = 4999; // 20 ms period (5000 ticks)
    OCR1B = SERVO_FRONT;
    current_position = SERVO_FRONT;
}

// Match target position within +/-5 ticks (20 us tolerance).
uint8_t servo_at_position(int pulseWidth)
{
    int diff = current_position - pulseWidth;
    if (diff < 0) diff = -diff;
    return (diff <= 5);
}

int get_servo_position(void)
{
    return current_position;
}

// Returns 1 if a move was initiated, 0 if already within tolerance.
uint8_t move_servo(int pulseWidth)
{
    if (pulseWidth < SERVO_RIGHT) pulseWidth = SERVO_RIGHT;
    if (pulseWidth > SERVO_LEFT)  pulseWidth = SERVO_LEFT;
    
    if (servo_at_position(pulseWidth)) {
        return 0;
    }
    
    OCR1B = pulseWidth;
    current_position = pulseWidth;
    return 1;
}

// Move servo and estimate settling time in ms (~0.15 ms per tick, clamped to 20-80 ms).
uint8_t move_servo_smart(int pulseWidth, uint8_t *wait_ms)
{
    if (pulseWidth < SERVO_RIGHT) pulseWidth = SERVO_RIGHT;
    if (pulseWidth > SERVO_LEFT)  pulseWidth = SERVO_LEFT;
    
    if (servo_at_position(pulseWidth)) {
        *wait_ms = 0;
        return 0;
    }
    
    int distance = current_position - pulseWidth;
    if (distance < 0) distance = -distance;
    
    uint8_t estimated_wait = (uint8_t)((distance * 15) / 100);
    if (estimated_wait < 20) estimated_wait = 20;
    if (estimated_wait > 80) estimated_wait = 80;
    
    *wait_ms = estimated_wait;
    OCR1B = pulseWidth;
    current_position = pulseWidth;
    return 1;
}

#include "ultrasonic.h"
#include "servo.h"
#include "uart.h"
#include "time.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_end_us = 0;
static volatile uint8_t  echo_ready = 0;
static volatile uint8_t  echo_started = 0;

// Speed of sound conversion: (0.0343 cm/us / 2) / 2.54 cm/in = 0.00675197 in/us
#define US_TO_INCHES 0.00675197

void setup_ultrasonic(void) {
    DDRB |= (1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIGGER_PIN);

    // Enable Pin Change Interrupt 0 for ECHO_PIN (PB5 / PCINT5)
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << ECHO_PIN);
}

// Capture rising and falling edge timestamps on ECHO_PIN using Timer2 micros().
ISR(PCINT0_vect) {
    uint32_t now = micros();
    if (PINB & (1 << ECHO_PIN)) {
        echo_start_us = now;
        echo_started = 1;
    } else if (echo_started) {
        echo_end_us = now;
        echo_ready = 1;
        echo_started = 0;
    }
}

// Trigger a 10 us pulse and return measured distance in inches (0.0 on timeout or out of [1.5, 80.0] in).
double measure_distance(void) {
    echo_ready = 0;
    echo_started = 0;
    
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);

    // 15 ms timeout corresponds to ~101 inches max range
    uint32_t start = micros();
    const uint32_t timeout_us = 15000UL;
    
    while (!echo_ready) {
        if ((micros() - start) > timeout_us) {
            return 0.0;
        }
    }

    uint32_t duration_us = echo_end_us - echo_start_us;
    double distance_in = duration_us * US_TO_INCHES;

    if (distance_in < 1.5 || distance_in > 80.0) {
        return 0.0;
    }
    return distance_in;
}

double measure_distance_fast(void) {
    return measure_distance();
}

// Average two readings separated by 500 us; return whichever is valid if one fails.
double measure_distance_avg(void) {
    double d1 = measure_distance();
    _delay_us(500);
    double d2 = measure_distance();
    
    if (d1 == 0.0) return d2;
    if (d2 == 0.0) return d1;
    
    return (d1 + d2) / 2.0;
}

double proximity_read(int direction) {
    move_servo(direction);
    _delay_ms(80);

    (void)measure_distance();
    _delay_ms(20);

    return measure_distance();
}

void debug_ultrasonic(void) {
    char buffer[64];

    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);

    _delay_ms(30);

    double test_distance = measure_distance();
    uint32_t distance_inch = (uint32_t)(test_distance * 100);

    snprintf(buffer, sizeof(buffer), "Raw Distance: %lu.%02lu in",
             distance_inch / 100, distance_inch % 100);
    uart_println(buffer);
}

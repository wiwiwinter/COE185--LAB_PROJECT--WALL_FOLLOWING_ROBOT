# Hardware, Calibration, and Tuning Guide

This guide covers wiring verification, timer calculations, PD controller tuning, and serial diagnostics for the ATmega328P wall-following robot.

---

## 1. Hardware Setup & Wiring Checklist

Before applying motor power, verify the following connections against the firmware definitions:

1. **Motor Driver (PD2, PD4, PD5, PD6)**
   - Left motor PWM connects to `PD5` (`OC0B`, Arduino pin `D5`) and direction to `PD2` (`D2`).
   - Right motor PWM connects to `PD6` (`OC0A`, Arduino pin `D6`) and direction to `PD4` (`D4`).
   - Note that [`motors.c`](motors.c) sets `PD2 HIGH` for left-motor forward and `PD4 LOW` for right-motor forward because the two motors are mounted facing opposite sides of the chassis.
2. **Scanning Servo (PB2)**
   - Signal wire connects to `PB2` (`OC1B`, Arduino pin `D10`).
   - Power the servo from a regulated 5V rail capable of handling stall current spikes rather than relying on a weak logic trace, and share a common `GND` with the ATmega328P.
3. **HC-SR04 Ultrasonic Sensor (PB4, PB5)**
   - `TRIG` connects to `PB4` (Arduino pin `D12`).
   - `ECHO` connects to `PB5` (Arduino pin `D13` / `PCINT5`).

---

## 2. How the Peripherals Work

### Timer2 Timebase (`time.c`)
`Timer2` runs in Clear Timer on Compare Match (CTC) mode with prescaler `64`:
- Tick frequency: `16 MHz / 64 = 250 kHz` (`4 us` per tick).
- `OCR2A = 249` fires `TIMER2_COMPA_vect` every `250 ticks` (`1.0 ms`), incrementing `g_ms_count`.
- `micros()` combines `g_ms_count * 1000` with the live counter `TCNT2 * 4` inside an `ATOMIC_BLOCK` to provide `4 us` timing resolution without disabling interrupts globally for long periods.

### Ultrasonic Echo Timing (`ultrasonic.c`)
Instead of blocking inside a busy-wait loop from the start of the pulse, `PCINT0_vect` captures `micros()` on the rising and falling edges of `PB5`:
- Distance conversion:
  $$\text{distance (in)} = \Delta t_{\mu\text{s}} \times \frac{0.0343\text{ cm}/\mu\text{s}}{2 \times 2.54\text{ cm/in}} = \Delta t_{\mu\text{s}} \times 0.00675197$$
- Readings outside `[1.5, 80.0]` inches or exceeding the `15 ms` timeout return `0.0`.
- In [`main.c`](main.c), `take_measurement()` averages two valid consecutive readings separated by `10 ms` and feeds them into a 3-sample moving average buffer (`right_distance_history`).

### Servo PWM (`servo.c`)
`Timer1` runs in Fast PWM Mode 15 (`WGM13:0 = 1111`) with prescaler `64` and `OCR1A = 4999`:
- Period: `5000 ticks * 4 us = 20,000 us` (`50 Hz`).
- Pulse width positions in [`servo.h`](servo.h):
  - `SERVO_RIGHT = 150` (`600 us`)
  - `SERVO_FRONT = 350` (`1400 us`)
  - `SERVO_LEFT  = 625` (`2500 us`)
- `servo_at_position()` checks whether the servo is already within `5 ticks` (`20 us`) of the requested position so the main loop does not incur a `100 ms` settling delay (`SERVO_STABILIZE_MS`) when the servo is already facing the right wall.

---

## 3. PD Controller & Speed Scheduling

While in `STATE_RUNNING`, the robot computes the distance error relative to `DESIRED_DISTANCE_IN` (`9.0 in`):

```text
error      = right_distance - DESIRED_DISTANCE_IN
derivative = error - previous_error
steering   = clamp((KP * error) + (KD * derivative), -MAX_STEERING, +MAX_STEERING)
```

Wheel speeds are updated continuously around `base_speed`:
- `left_speed  = clamp(base_speed + steering, MIN_SPEED, MAX_SPEED)`
- `right_speed = clamp(base_speed - steering, MIN_SPEED, MAX_SPEED)`

**How it reacts:**
- **Too far from the right wall (`error > 0`):** `steering > 0` increases `left_speed` and decreases `right_speed`, curving the robot rightward toward the wall.
- **Too close to the right wall (`error < 0`):** `steering < 0` decreases `left_speed` and increases `right_speed`, curving the robot leftward away from the wall.
- **Speed scheduling:**
  - When `|error| < 2.5 in` (`DISTANCE_TOLERANCE`), `base_speed` rises to `MAX_SPEED` (`0.32`).
  - When `2.5 in <= |error| <= 3.0 in`, `base_speed` stays at `CRUISE_SPEED` (`0.25`).
  - When `|error| > 3.0 in`, `base_speed` drops to `MIN_SPEED` (`0.15`) so sharp corrections happen at low forward velocity.

---

## 4. Calibration & Tuning Procedure

### Step 1: Verify Servo Alignment
1. Flash the firmware and observe the servo during the first `500 ms` after reset.
2. It should point perpendicular to the right side of the chassis (`SERVO_RIGHT = 150`).
3. Every `150 ms` during `STATE_RUNNING`, it sweeps to `SERVO_FRONT = 350`. If `350` is not dead-ahead on your horn mounting, reseat the servo horn or adjust `SERVO_FRONT` and `SERVO_RIGHT` in [`servo.h`](servo.h).

### Step 2: Calibrate Wheel Polarity and Top Speed
1. Lift the wheels off the ground and check that both wheels spin forward during `STATE_RUNNING`.
2. If one wheel spins backward when commanded `FORWARD`, invert its `PORTD` bit logic in `set_motor_speed()` inside [`motors.c`](motors.c).
3. If the robot moves too fast or stalls on high-friction floors, adjust `MOTOR_TOP_SPEED` (default `200` out of `255`) in [`motors.h`](motors.h) or the speed ratios (`CRUISE_SPEED`, `MAX_SPEED`, `MIN_SPEED`) in [`main.c`](main.c).

### Step 3: Tune `KP` and `KD`
1. **Oscillation (fishtailing along the wall):** Lower `KP` (default `0.035`) in steps of `0.005`, or increase `KD` (default `0.015`) to dampen rapid error changes.
2. **Sluggish recovery (drifts into or away from the wall):** Increase `KP` slightly or raise `MAX_STEERING` (default `0.15`).
3. **Corner turning (`STATE_OBSTACLE_AVOID`):** Adjust `TURN_DURATION_MS` (`200 ms`) and `TURN_SPEED` (`0.22`) in [`main.c`](main.c) so a single left pivot rotates roughly 45-90 degrees on your surface.

---

## 5. Serial Telemetry & Troubleshooting

Connect a serial monitor at **9600 baud, 8 data bits, no parity, 2 stop bits (8N2)** (`UCSR0C = (1<<USBS0)|(3<<UCSZ00)` in [`uart.c`](uart.c); standard 8N1 monitors also read it cleanly).

### Sample Serial Output
```text
Starting continuous wall-following mode
Target: 9 inches from right wall
Startup Right: 8.74in
Initial reading OK, starting...
R: 8.82in
R: 8.91in
F: 31.40in
R: 9.05in
OBSTACLE at 11.20in!
Avoiding obstacle - turning left
After turn F: 28.50in
Resuming wall follow
```

### Common Issues

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| Serial prints `?` instead of numbers (e.g., `R: ?in`) | AVR `vfprintf` linked without float support | Build using the provided [`Makefile`](Makefile), which passes `-Wl,-u,vfprintf -lprintf_flt -lm`. |
| Always prints `Waiting for valid reading...` | Ultrasonic sensor not receiving echo on `PB5` | Check 5V power to HC-SR04, verify `TRIG = PB4` (`D12`) and `ECHO = PB5` (`D13`), and ensure an object is within `1.5` to `80.0` inches. |
| Robot stutters every `150 ms` | Front scan interval (`FRONT_CHECK_INTERVAL_MS`) + `200 ms` total servo settle time | Increase `FRONT_CHECK_INTERVAL_MS` (e.g., to `250` or `300 ms`) or reduce `SERVO_STABILIZE_MS` if using a high-speed servo. |
| `avrdude: ser_open(): can't open device` | Wrong COM port in [`Makefile`](Makefile) | Check Device Manager for your Arduino/USB-UART port and pass `make flash PROGRAMMER="-c arduino -b 115200 -P COMx"`. |

/*
 * EENG350 Demo 2 - Stand-in for the Arduino core, so beacon.ino can be
 * compiled and run on a PC by sim_main.cpp (see run_sim.py).
 *
 * Only what beacon.ino uses is here. The functions are implemented in
 * sim_main.cpp, where the pins are wired to a simulated robot: the PWM and
 * direction pins drive two motor models, the encoder pins toggle as the
 * simulated wheels turn, and Wire delivers the simulated Pi's messages.
 */

#ifndef HOST_SIM_ARDUINO_H
#define HOST_SIM_ARDUINO_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.1415926535897932
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define SDA 18
#define SCL 19
#define F(text) (text)                  // on the Uno this keeps a string in flash

// Macros on the Uno too, which is why min(PWM, 255.0) can mix types
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

unsigned long millis();                 // each call also moves the simulation on one step
unsigned long micros();
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
uint8_t readPortD();                    // pins 0 to 7 as one byte, as the Uno's PIND register
#define PIND (readPortD())
void analogWrite(int pin, int value);   // a float argument is cut to an int, as on the Uno
void attachInterrupt(int pin, void (*isr)(), int mode);
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void noInterrupts() {}
inline void interrupts() {}
inline void delayMicroseconds(unsigned int) {}

struct SerialSim {
    void begin(long) {}
    void print(const char *text);
    void print(char c);
    void print(int value);
    void print(unsigned int value);
    void print(long value);
    void print(unsigned long value);
    void print(double value, int digits = 2);
    template <class T> void println(T value) { print(value); print("\n"); }
    void println(double value, int digits) { print(value, digits); print("\n"); }
};
extern SerialSim Serial;

struct WireSim {
    void begin(int) {}
    void onReceive(void (*callback)(int));
    int available();
    int read();
};
extern WireSim Wire;

#endif

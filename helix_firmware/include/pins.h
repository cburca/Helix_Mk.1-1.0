// Teensy Inputs/Devices
// - LED strips (x2 digital outputs)
// - E-stop (x1 digital inputs) - Interrupts
// - Motor 1 Encoder (x2 digital inputs) - Interrupts
// - Motor 2 Encoder (x2 digital inputs) - Interrupts
// - Motor 1 Driver (x2 digital outputs, x1 ADC input) - Polling
// - Motor 2 Driver (x2 digital outputs, x1 ADC input) - Polling
// - Ultrasonic Sensor (x4 digial outpuot, x4 ADC input) - Polling
// - UART TX (-> Pi)
// - UART RX (<- Pi)
// - IMU (x1 UART TX to Teensy) - Polling



//* LED Strip *//

#define LED_STRIP_F 2
#define LED_STRIP_R 3
#define NUM_LEDS 22  // Adjust this according to the number of LEDs per strip

#define WHITE 255, 255, 255
#define BLUE 0, 0, 255
#define RED 255, 0, 0
#define YELLOW 255, 255, 0
#define OFF 0, 0, 0

//* E Stop *//

#define E_STOP 4

//* Motor Encoders *//

#define LEFT_ENC_A 5
#define LEFT_ENC_B 6

#define RIGHT_ENC_A 7
#define RIGHT_ENC_B 8

//* Motor Drivers *//

#define BTS7960_1_R_PWM
#define BTS7960_1_L_PWM
#define BTS7960_1_R_EN
#define BTS7960_1_L_EN
#define BTS7960_1_R_IS // May share both drive side current alarms
#define BTS7960_1_L_IS

#define BTS7960_2_R_PWM
#define BTS7960_2_L_PWM
#define BTS7960_2_R_EN
#define BTS7960_2_L_EN
#define BTS7960_2_R_IS // May share both drive side current alarms
#define BTS7960_2_L_IS

//* Ultrasonic Sensors *//

#define ME007YS_1_TX
#define ME007YS_1_ECHO

// #define ULT_2_TX
// #define ULT_2_ECHO

// #define ULT_3_TX
// #define ULT_3_ECHO

// #define ULT_4_TX
// #define ULT_4_ECHO

//* Comms *//

#define TEENSY2PI_TX
#define TEENSY2PI_RX

#define WT901_UART_RX

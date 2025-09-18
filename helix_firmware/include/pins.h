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

#define LED_STRIP_F
#define LED_STRIP_R

#define E_STOP

#define MOTER_ENC_1_HIGH
#define MOTER_ENC_1_LOW

#define MOTER_ENC_2_HIGH
#define MOTER_ENC_2_LOW

#define BTS7960_1_R_PWM
#define BTS7960_1_L_PWM
#define BTS7960_1_R_EN
#define BTS7960_1_L_EN
#define BTS7960_1_R_IS
#define BTS7960_1_L_IS

#define BTS7960_2_R_PWM
#define BTS7960_2_L_PWM
#define BTS7960_2_R_EN
#define BTS7960_2_L_EN
#define BTS7960_2_R_IS
#define BTS7960_2_L_IS

#define ME007YS_1_TX
#define ME007YS_1_ECHO

// #define ULT_2_TX
// #define ULT_2_ECHO

// #define ULT_3_TX
// #define ULT_3_ECHO

// #define ULT_4_TX
// #define ULT_4_ECHO

#define TEENSY2PI_TX
#define TEENSY2PI_RX

#define WT901_UART_RX

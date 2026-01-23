#include <Arduino.h>
#include "Odometry.h"
#include "MotorDriver.h"
#include "LEDs.h"
#include "pins.h"
#include "PIDController.h"
#include "uart.h"


#define TRACK_WIDTH        0.35f   // Meters (Distance between wheels)
#define PID_INTERVAL_MS    100      // 100Hz Control Loop
#define DIAG_INTERVAL_MS   300      // 100Hz Control Loop
#define ODOM_INTERVAL_MS   50      // 20Hz Telemetry Loop


UARTInterface uart(Serial1);

Odometry leftOdom(LEFT_ENC_A, LEFT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, 1);   // wheel diameter = 160 mm
Odometry rightOdom(RIGHT_ENC_A, RIGHT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, -1);

MotorDriver motorL(BTS7960_1_R_PWM, BTS7960_1_L_PWM, BTS7960_1_R_EN, BTS7960_1_L_EN);
MotorDriver motorR(BTS7960_2_R_PWM, BTS7960_2_L_PWM, BTS7960_2_R_EN, BTS7960_2_L_EN);

// Instantiate PID control objects (left/right wheels) after setting up odom & motors
WheelController leftPID(leftOdom, motorL);
WheelController rightPID(rightOdom, motorR);


LedController ledStrips(NUM_LEDS, LED_STRIP_F, LED_STRIP_R);

// E-Stop Stuff
bool estopActive = false;
int estop = E_STOP;

void estop_press(void){
  estopActive = (digitalRead(estop) == HIGH);
}

// ================== GLOBAL VARIABLES ==================
unsigned long last_pid_time = 0;
unsigned long last_odom_time = 0;
unsigned long last_diag_time = 0;
uint8_t payload_buffer[64]; 

// ================== HELPER FUNCTIONS ==================

void handleCmdVel(uint8_t *data) {
  float lin_x, ang_z;
  
  // 1. Unpack Float data (Little Endian)
  memcpy(&lin_x, &data[0], 4);
  memcpy(&ang_z, &data[4], 4);

  // 2. Inverse Kinematics (Differential Drive)
  // V_left = V_lin - (V_ang * Width / 2)
  // V_right = V_lin + (V_ang * Width / 2)
  float vel_l = lin_x - (ang_z * (TRACK_WIDTH / 2.0f));
  float vel_r = lin_x + (ang_z * (TRACK_WIDTH / 2.0f));

  // 3. Set PID Setpoints
  pidLeft.setTargetVelocity(vel_l);
  pidRight.setTargetVelocity(vel_r);
}

void sendOdometry() {
  // 1. Get latest data
  // NOTE: Velocities are already calculated in the main loop via odom.update()
  int32_t l_ticks = leftOdom.getTicks();
  int32_t r_ticks = rightOdom.getTicks();
  float v_l = leftOdom.getStoredLinearVelocity();
  float v_r = rightOdom.getStoredLinearVelocity();

  // 2. Forward Kinematics (Wheel Vels -> Robot Vel)
  float lin_vel = (v_l + v_r) / 2.0f;
  float ang_vel = (v_r - v_l) / TRACK_WIDTH;
  uint32_t timestamp = millis();

  // 3. Pack Data: [Time(4), L_Tick(4), R_Tick(4), Lin(4), Ang(4)]
  uint8_t odom_payload[20];
  memcpy(&odom_payload[0], &timestamp, 4);
  memcpy(&odom_payload[4], &l_ticks, 4);
  memcpy(&odom_payload[8], &r_ticks, 4);
  memcpy(&odom_payload[12], &lin_vel, 4);
  memcpy(&odom_payload[16], &ang_vel, 4);

  // 4. Send
  uart.sendPacket(PKT_ODOM, odom_payload, 20);
}


// --- Setup ---
void setup() {
    Serial.begin(9600);
    uart.begin(115200);

    // E-STOP //
    pinMode(estop,INPUT_PULLUP);
    attachInterrupt(estop, estop_press, CHANGE);

    // Encoders //
    // MAYBE DO THIS IN ODOM FILES?
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftOdom.leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftOdom.leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightOdom.rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightOdom.rightEncoderISR, CHANGE);

    leftOdom.begin();
    rightOdom.begin();

    // Motors Init. //
    motorL.begin();
    motorL.enable();
    motorR.begin();
    motorR.enable();

    // PID Control Init. //
    leftPID.begin();
    rightPID.begin();

    // LED Strips //
    ledStrips.begin();
    ledStrips.initialStartup();
    ledStrips.normalOperation();

    Serial.println("==== ODOMETRY TEST START ====");
    Serial.println("Encoders ready. Rotate wheels or run motors.");
    Serial.println();

    // OPEN-LOOP MOTOR CONTROL TESTING //
    // motorL.setSpeed(80);
    // motorR.setSpeed(80);

    // CLOSED-LOOP MOTOR CONTROL TESTING //
    // leftPID.setTargetVelocity(0.3);
    // rightPID.setTargetVelocity(0.3);
}

// --- Loop ---
void loop() {
    unsigned long now = millis();

//////// Single-Wheel Rotation Test ///////////
    // if(leftOdom.getTicks() > 1440){
    //     motorL.disable();
    // }
    // else if(rightOdom.getTicks() > 1440){
    //     motorR.disable();
    // }
///////////////////////////////////////////////
    
    // 1. READ SERIAL (Poll for commands)
    uart.update();
    if (uart.isPacketAvailable()) {
        PacketType type = uart.getPacketType();
        uint8_t len = uart.getPacketLength();
        uart.getPacketPayload(payload_buffer);

        if (type == PKT_CMDVEL && len == 8) {
        handleCmdVel(payload_buffer);
        }
    }

    // 2. CONTROL LOOP (10Hz)
    if (now - last_pid_time >= PID_INTERVAL_MS) {  // 10 Hz
        float dt = (now - last_pid_time) / 1000.0f;
        last_pid_time = now;

        leftOdom.update();
        rightOdom.update();

        leftPID.update(dt);
        rightPID.update(dt);
    }

    // 3. TELEMETRY LOOP (20Hz)
    if (now - last_odom_time >= ODOM_INTERVAL_MS) {
        last_odom_time = now;
        sendOdometry();
    }

    // E-Stop check
    if(estopActive == true){
        motorL.disable();
        motorR.disable();

        leftPID.disablePID();
        rightPID.disablePID();

        while(estopActive){
            ledStrips.handleObstacle();
        }

        motorL.enable();
        motorR.enable();
        // motorL.setSpeed(50);     // <--- Open Loop
        // motorR.setSpeed(-50);
        leftPID.enablePID();        // <--- Closed Loop - need to make sure no integral windup
        rightPID.enablePID();

        ledStrips.normalOperation();
    }

    // ODOMETRY DIAGNOSTIC STATS
    if (now - last_diag_time >= DIAG_INTERVAL_MS) { // every 0.3 seconds
        lastDiag = now;

    //     // // 1. Distance test
        float leftDist = leftOdom.getDistance();
        float rightDist = rightOdom.getDistance();

    //     // // 2. Velocity tests
        float leftAngVel = leftOdom.getStoredAngularVelocity();
        float rightAngVel = rightOdom.getStoredAngularVelocity();

        float leftLinVel = leftOdom.getStoredLinearVelocity();
        float rightLinVel = rightOdom.getStoredLinearVelocity();

    //     // 3. Print results
        Serial.println("------ Encoder Report ------");
        Serial.print("Ticks [L,R]: ");
        Serial.print(leftOdom.getTicks()); Serial.print(", ");
        Serial.println(rightOdom.getTicks());

        Serial.print("Distance [m] [L,R]: ");
        Serial.print(leftDist, 5); Serial.print(", ");
        Serial.println(rightDist, 5);

        Serial.print("Angular Vel [rad/s] [L,R]: ");
        Serial.print(leftAngVel, 3); Serial.print(", ");
        Serial.println(rightAngVel, 3);

        Serial.print("Linear Vel [m/s] [L,R]: ");
        Serial.print(leftLinVel, 3); Serial.print(", ");
        Serial.println(rightLinVel, 3);

        Serial.println("-----------------------------\n");
    }
}
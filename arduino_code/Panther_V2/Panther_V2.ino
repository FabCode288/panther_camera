// ============================================================
// Panther - Safe Communication + Motor Control
// ============================================================
// - Binary serial communication @ 115200 Baud
// - Connection state machine
// - Fixed 32-byte packet structure
// - Packet synchronization via 0xAA 0x55
// - XOR checksum
// - Command timeout failsafe
// - ESTOP / RESET
// - DC motor PID control
// - Encoder feedback
// - Motor feedback transmission @ 30 Hz
// ============================================================

#include <stdint.h>
#include <string.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ============================================================
// PINS
// ============================================================

// Left motor
const int encoderA_L = 3;
const int encoderB_L = 5;
const int ENA_L      = 11;
const int IN1_L      = 12;
const int IN2_L      = 13;

// Right motor
const int encoderA_R = 2;
const int encoderB_R = 4;
const int ENA_R      = 9;
const int IN1_R      = 8;
const int IN2_R      = 7;


// ============================================================
// SERIAL / COMMUNICATION
// ============================================================

#define BAUDRATE 115200

const unsigned long COMMUNICATION_TIMEOUT = 500;
const unsigned long FEEDBACK_INTERVAL     = 33;

//====================
// IMU
//====================
Adafruit_MPU6050 mpu;
const unsigned long IMU_INTERVAL = 33;
unsigned long lastImuSend = 0;

// ============================================================
// MOTOR / ENCODER
// ============================================================

volatile long ticks_L = 0;
volatile long ticks_R = 0;

long lastTicks_L = 0;
long lastTicks_R = 0;

unsigned long lastControlTime = 0;

const float ticksPerRevolution = 44.0;
const float gearRatio          = 22.2;

const float ticksPerWheelRev =
  ticksPerRevolution * gearRatio;

const float radPerTick =
  (2.0 * PI) / ticksPerWheelRev;


// ============================================================
// MOTOR POSITION
// ============================================================

float position_L = 0.0;
float position_R = 0.0;


// ============================================================
// MOTOR TARGETS
// ============================================================

float targetOmega_L = 0.0;
float targetOmega_R = 0.0;


// ============================================================
// PID
// ============================================================

float Kp = 1.3;
float Ki = 0.5;
float Kd = 0.005;

float integral_L = 0.0;
float integral_R = 0.0;

float lastError_L = 0.0;
float lastError_R = 0.0;

int pwm_L = 50;
int pwm_R = 50;

const int minPWM = 30;


// ============================================================
// STATE MACHINE
// ============================================================

enum State
{
  DISCONNECTED,
  WAITING,
  READY,
  ACTIVE,
  ESTOP
};

State state = WAITING;


// ============================================================
// PACKET TYPES
// ============================================================

enum PacketType : uint8_t
{
  PKT_PING           = 1,
  PKT_READY          = 2,
  PKT_HB             = 3,

  PKT_CMD            = 10,

  PKT_ESTOP          = 20,
  PKT_RESET          = 21,

  PKT_MOTOR_FEEDBACK = 30,
  PKT_IMU_DATA       = 31,

  PKT_DEBUG_CMD      = 40
};


// ============================================================
// DATA STRUCTURES
// ============================================================

#pragma pack(push, 1)

struct Command
{
  int16_t left;
  int16_t right;
};

struct MotorFeedback
{
  uint32_t timestamp_us;

  float position_L;
  float position_R;

  float omega_L;
  float omega_R;
};

struct DebugCmd
{
  int16_t left;
  int16_t right;
};

struct ImuData
{
  float accel_x;
  float gyro_z;
};

// ============================================================
// FIXED PACKET
// ============================================================
//
// Total size:
//
// header1      1 byte
// header2      1 byte
// type         1 byte
// payload     27 byte
// checksum     2 byte
//
// TOTAL       32 byte
//
// ============================================================

struct Packet
{
  uint8_t header1;
  uint8_t header2;

  uint8_t type;

  uint8_t payload[27];

  uint16_t checksum;
};

#pragma pack(pop)


// ============================================================
// PACKET BUFFER
// ============================================================

union PacketBuffer
{
  Packet packet;
  uint8_t bytes[sizeof(Packet)];
};


// ============================================================
// SIZE CHECK
// ============================================================

static_assert(
  sizeof(Packet) == 32,
  "Packet size must be exactly 32 bytes");


// ============================================================
// RX BUFFER
// ============================================================

PacketBuffer rx_buffer;

size_t rx_index = 0;


// ============================================================
// COMMUNICATION TIMERS
// ============================================================

unsigned long lastMessageTime = 0;
unsigned long lastFeedbackTime = 0;


// ============================================================
// HELPER
// ============================================================

void stopMotors()
{
  targetOmega_L = 0.0;
  targetOmega_R = 0.0;

  integral_L = 0.0;
  integral_R = 0.0;

  lastError_L = 0.0;
  lastError_R = 0.0;
}


// ============================================================
// CHECKSUM
// ============================================================
//
// XOR over TYPE and all payload bytes.
//
// This is independent of the actual data structure inside
// the payload and therefore works for all packet types.
//
// ============================================================

uint16_t calculateChecksum(
  const Packet& packet)
{
  uint16_t checksum = 0;

  checksum ^= packet.type;

  for (uint8_t i = 0; i < sizeof(packet.payload); i++)
  {
    checksum ^= packet.payload[i];
  }

  return checksum;
}


// ============================================================
// SEND PACKET
// ============================================================
//
// Every packet sent by the Arduino is exactly 32 bytes.
//
// ============================================================

void sendPacket(
  uint8_t type,
  const uint8_t* payload = nullptr,
  uint8_t payloadSize = 0)
{
  if (payloadSize > sizeof(Packet::payload))
    return;

  PacketBuffer tx;

  memset(
    &tx,
    0,
    sizeof(tx));

  tx.packet.header1 = 0xAA;
  tx.packet.header2 = 0x55;

  tx.packet.type = type;

  if (payload != nullptr && payloadSize > 0)
  {
    memcpy(
      tx.packet.payload,
      payload,
      payloadSize);
  }

  tx.packet.checksum =
    calculateChecksum(tx.packet);

  Serial.write(
    tx.bytes,
    sizeof(tx.bytes));
}


// ============================================================
// SEND READY
// ============================================================

void sendReady()
{
  sendPacket(PKT_READY);
}

// ============================================================
// SEND DEBUG CMD
// ============================================================
//
// Sends back exactly the command values received from ROS.
//
// ============================================================

void sendDebugCmd(
  int16_t left,
  int16_t right)
{
  DebugCmd debug{};

  debug.left = left;
  debug.right = right;

  sendPacket(
    PKT_DEBUG_CMD,
    reinterpret_cast<uint8_t*>(&debug),
    sizeof(debug));
}


// ============================================================
// SEND MOTOR FEEDBACK
// ============================================================

void sendMotorFeedback(
  float omegaWheel_L,
  float omegaWheel_R)
{
  MotorFeedback feedback{};

  feedback.timestamp_us = micros();

  feedback.position_L = position_L;
  feedback.position_R = position_R;

  feedback.omega_L = omegaWheel_L;
  feedback.omega_R = omegaWheel_R;

  sendPacket(
    PKT_MOTOR_FEEDBACK,
    reinterpret_cast<uint8_t*>(&feedback),
    sizeof(feedback));
}


// ============================================================
// PACKET PROCESSING
// ============================================================

void processPacket(
  const Packet& packet)
{

  // ----------------------------------------------------------
  // PING
  // ----------------------------------------------------------

  if (packet.type == PKT_PING)
  {
    sendReady();

    state = READY;
    lastMessageTime = millis();
    return;
  }


  // ----------------------------------------------------------
  // WAITING
  // ----------------------------------------------------------

  if (state == WAITING)
  {
    return;
  }


  // ----------------------------------------------------------
  // READY
  // ----------------------------------------------------------

  if (state == READY)
  {
    if (packet.type == PKT_ESTOP)
    {
      state = ESTOP;

      stopMotors();
      lastMessageTime = millis();
      return;
    }


    if (packet.type == PKT_CMD)
    {
      Command command{};
      lastMessageTime = millis();
      memcpy(
        &command,
        packet.payload,
        sizeof(command));


      // Send received values back for debugging
      //sendDebugCmd(command.left, command.right);


      // FIXED POINT -> FLOAT

      float v_left =
        command.left / 100.0f;

      float v_right =
        command.right / 100.0f;


      // SAFETY LIMIT

      if (abs(v_left) > 50.0f ||
          abs(v_right) > 50.0f)
      {
        return;
      }


      // MOTOR TARGETS

      targetOmega_L = v_left;
      targetOmega_R = v_right;

      state = ACTIVE;

      return;
    }

    return;
  }


  // ----------------------------------------------------------
  // ACTIVE
  // ----------------------------------------------------------

  if (state == ACTIVE)
  {
    if (packet.type == PKT_ESTOP)
    {
      state = ESTOP;

      stopMotors();
      lastMessageTime = millis();
      return;
    }


    if (packet.type == PKT_CMD)
    {
      Command command{};
      lastMessageTime = millis();
      memcpy(
        &command,
        packet.payload,
        sizeof(command));


      // Send received values back for debugging
      //sendDebugCmd(command.left, command.right);


      // FIXED POINT -> FLOAT

      float v_left =
        command.left / 100.0f;

      float v_right =
        command.right / 100.0f;


      // SAFETY LIMIT

      if (abs(v_left) > 50.0f ||
          abs(v_right) > 50.0f)
      {
        return;
      }


      // MOTOR TARGETS

      targetOmega_L = v_left;
      targetOmega_R = v_right;

      return;
    }

    return;
  }


  // ----------------------------------------------------------
  // ESTOP
  // ----------------------------------------------------------

  if (state == ESTOP)
  {
    if (packet.type == PKT_RESET)
    {
      stopMotors();

      state = WAITING;
      lastMessageTime = millis();
      return;
    }

    return;
  }
}


// ============================================================
// SERIAL RECEIVE
// ============================================================
//
// This follows the same basic structure as the working
// communication from the old project.
//
// The receiver waits for:
//
// 0xAA
// 0x55
//
// and then reads exactly sizeof(Packet) bytes.
//
// ============================================================

void handleSerial()
{
  while (Serial.available())
  {
    uint8_t byte = Serial.read();


    switch (rx_index)
    {
      // ------------------------------------------------------
      // HEADER 1
      // ------------------------------------------------------

      case 0:

        if (byte != 0xAA)
        {
          continue;
        }

        rx_buffer.bytes[rx_index++] = byte;

        break;


      // ------------------------------------------------------
      // HEADER 2
      // ------------------------------------------------------

      case 1:

        if (byte != 0x55)
        {
          rx_index = 0;

          continue;
        }

        rx_buffer.bytes[rx_index++] = byte;

        break;


      // ------------------------------------------------------
      // REST OF PACKET
      // ------------------------------------------------------

      default:

        rx_buffer.bytes[rx_index++] = byte;


        // Complete 32-byte packet received

        if (rx_index >= sizeof(Packet))
        {
          rx_index = 0;


          uint16_t checksum =
            calculateChecksum(
              rx_buffer.packet);


          if (checksum ==
              rx_buffer.packet.checksum)
          {
            processPacket(
              rx_buffer.packet);
          }
        }

        break;
    }
  }
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void applyMotor(
  int pwm,
  int ENA,
  int IN1,
  int IN2,
  float target)
{
  if (pwm > 0 && pwm < minPWM)
    pwm = minPWM;

  if (pwm < 0 && pwm > -minPWM)
    pwm = -minPWM;


  // ----------------------------------------------------------
  // TARGET ZERO
  // ----------------------------------------------------------

  if (abs(target) < 0.1)
  {
    analogWrite(ENA, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    return;
  }


  // ----------------------------------------------------------
  // FORWARD
  // ----------------------------------------------------------

  if (pwm >= 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENA, pwm);
  }


  // ----------------------------------------------------------
  // REVERSE
  // ----------------------------------------------------------

  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(ENA, -pwm);
  }
}


// ============================================================
// ENCODER LEFT
// ============================================================

void readEncoderL()
{
  int A = digitalRead(encoderA_L);
  int B = digitalRead(encoderB_L);

  if (A == B)
    ticks_L++;
  else
    ticks_L--;
}


// ============================================================
// ENCODER RIGHT
// ============================================================

void readEncoderR()
{
  int A = digitalRead(encoderA_R);
  int B = digitalRead(encoderB_R);

  if (A == B)
    ticks_R++;
  else
    ticks_R--;
}

void sendImuData()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ImuData imu{};

  imu.accel_x = a.acceleration.x;
  imu.gyro_z = g.gyro.z;

  sendPacket(
    PKT_IMU_DATA,
    reinterpret_cast<uint8_t*>(&imu),
    sizeof(imu)
  );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(BAUDRATE);


  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  pinMode(ENA_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(ENA_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);


  // ----------------------------------------------------------
  // ENCODER PINS
  // ----------------------------------------------------------

  pinMode(
    encoderA_L,
    INPUT_PULLUP);

  pinMode(
    encoderB_L,
    INPUT_PULLUP);

  pinMode(
    encoderA_R,
    INPUT_PULLUP);

  pinMode(
    encoderB_R,
    INPUT_PULLUP);


  // ----------------------------------------------------------
  // SAFE STARTUP
  // ----------------------------------------------------------

  analogWrite(
    ENA_L,
    0);

  analogWrite(
    ENA_R,
    0);


  digitalWrite(
    IN1_L,
    LOW);

  digitalWrite(
    IN2_L,
    LOW);


  digitalWrite(
    IN1_R,
    LOW);

  digitalWrite(
    IN2_R,
    LOW);


  stopMotors();


  // ----------------------------------------------------------
  // ENCODER INTERRUPTS
  // ----------------------------------------------------------

  attachInterrupt(
    digitalPinToInterrupt(encoderA_L),
    readEncoderL,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(encoderA_R),
    readEncoderR,
    CHANGE);


  // ----------------------------------------------------------
  // TIMERS
  // ----------------------------------------------------------

  lastControlTime = millis();

  lastMessageTime = millis();

  lastFeedbackTime = millis();


  // ----------------------------------------------------------
  // INITIAL STATE
  // ----------------------------------------------------------

  state = WAITING;

  // -----------------------------
  // IMU
  // -----------------------------
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // SERIAL COMMUNICATION
  // ----------------------------------------------------------

  handleSerial();


  unsigned long now = millis();


  // ----------------------------------------------------------
  // COMMUNICATION FAILSAFE
  // ----------------------------------------------------------

  if (state == ACTIVE ||
      state == ESTOP)
  {
    if (now - lastMessageTime >
        COMMUNICATION_TIMEOUT)
    {
      stopMotors();

      state = WAITING;
    }
  }


  // ----------------------------------------------------------
  // MOTOR CONTROL
  // ----------------------------------------------------------

  if (now - lastControlTime >= 100)
  {
    float dt =
      (now - lastControlTime) /
      1000.0f;


    // --------------------------------------------------------
    // ENCODER DELTAS
    // --------------------------------------------------------

    long currentTicks_L = ticks_L;
    long currentTicks_R = ticks_R;


    long deltaTicks_L =
      currentTicks_L - lastTicks_L;

    long deltaTicks_R =
      currentTicks_R - lastTicks_R;


    // --------------------------------------------------------
    // POSITION
    // --------------------------------------------------------

    position_L +=
      deltaTicks_L * radPerTick;

    position_R +=
      deltaTicks_R * radPerTick;


    // --------------------------------------------------------
    // MOTOR ANGULAR VELOCITY
    // --------------------------------------------------------

    float omegaMotor_L =
      ((float)deltaTicks_L /
       ticksPerRevolution) *
      (2.0f * PI / dt);

    float omegaMotor_R =
      ((float)deltaTicks_R /
       ticksPerRevolution) *
      (2.0f * PI / dt);


    // --------------------------------------------------------
    // WHEEL ANGULAR VELOCITY
    // --------------------------------------------------------

    float omegaWheel_L =
      omegaMotor_L / gearRatio;

    float omegaWheel_R =
      omegaMotor_R / gearRatio;


    // --------------------------------------------------------
    // TARGET MOTOR VELOCITY
    // --------------------------------------------------------

    float targetMotor_L =
      targetOmega_L * gearRatio;

    float targetMotor_R =
      targetOmega_R * gearRatio;


    // --------------------------------------------------------
    // PID LEFT
    // --------------------------------------------------------

    float error_L =
      targetMotor_L -
      omegaMotor_L;


    if (abs(targetOmega_L) < 0.01)
      integral_L = 0;


    integral_L +=
      error_L * dt;


    float maxIntegral =
      255.0f / Ki;


    integral_L =
      constrain(
        integral_L,
        -maxIntegral,
        maxIntegral);


    float derivative_L =
      (error_L - lastError_L) /
      dt;


    float pid_L =
      Kp * error_L +
      Ki * integral_L +
      Kd * derivative_L;


    pwm_L = pid_L;


    pwm_L =
      constrain(
        pwm_L,
        -255,
        255);


    applyMotor(
      pwm_L,
      ENA_L,
      IN1_L,
      IN2_L,
      targetOmega_L);


    lastError_L = error_L;


    // --------------------------------------------------------
    // PID RIGHT
    // --------------------------------------------------------

    float error_R =
      targetMotor_R -
      omegaMotor_R;


    if (abs(targetOmega_R) < 0.01)
      integral_R = 0;


    integral_R +=
      error_R * dt;


    integral_R =
      constrain(
        integral_R,
        -maxIntegral,
        maxIntegral);


    float derivative_R =
      (error_R - lastError_R) /
      dt;


    float pid_R =
      Kp * error_R +
      Ki * integral_R +
      Kd * derivative_R;


    pwm_R = pid_R;


    pwm_R =
      constrain(
        pwm_R,
        -255,
        255);


    applyMotor(
      pwm_R,
      ENA_R,
      IN1_R,
      IN2_R,
      targetOmega_R);


    lastError_R = error_R;


    // --------------------------------------------------------
    // SAVE ENCODER VALUES
    // --------------------------------------------------------

    lastTicks_L = currentTicks_L;

    lastTicks_R = currentTicks_R;

    lastControlTime = now;


    // --------------------------------------------------------
    // MOTOR FEEDBACK
    // --------------------------------------------------------

    if ((now - lastFeedbackTime >= FEEDBACK_INTERVAL) && state == ACTIVE)
    {
      sendMotorFeedback(
        omegaWheel_L,
        omegaWheel_R);

      lastFeedbackTime = now;
    }
  }
  // IMU
  if ((millis() - lastImuSend >= IMU_INTERVAL) && state == ACTIVE)
  {
    lastImuSend = millis();
    sendImuData();
  }
}


// ============================================================
// COMMUNICATION PROTOCOL
// ============================================================
//
// Every packet is exactly 32 bytes:
//
// +--------+--------+------+----------------------+----------+
// | 0xAA   | 0x55   | TYPE | PAYLOAD (27 bytes)   | CHECKSUM |
// +--------+--------+------+----------------------+----------+
// |  1 B   |  1 B   | 1 B  |       27 B           |   2 B    |
// +--------+--------+------+----------------------+----------+
//
// TOTAL = 32 bytes
//
//
//
// ROS -> Arduino
//
// PKT_PING
//   Establishes connection.
//
// PKT_CMD
//   Payload:
//
//     Command.left
//     Command.right
//
//   Both values are int16_t.
//
//   Value = angular velocity [rad/s] * 100
//
// PKT_ESTOP
//   Stops motors and enters ESTOP.
//
// PKT_RESET
//   Leaves ESTOP and returns to WAITING.
//
//
//
// Arduino -> ROS
//
// PKT_READY
//   Connection established.
//
// PKT_HB
//   Heartbeat.
//
// PKT_MOTOR_FEEDBACK
//   Payload:
//
//     MotorFeedback.timestamp_us
//     MotorFeedback.position_L
//     MotorFeedback.position_R
//     MotorFeedback.omega_L
//     MotorFeedback.omega_R
//
// PKT_DEBUG_CMD
//   Payload:
//
//     DebugCmd.left
//     DebugCmd.right
//
//   Contains the CMD values exactly as received
//   by the Arduino.
//
// STATE MACHINE
//
// WAITING
//   Waits for PKT_PING.
//
// READY
//   Connection established.
//   Waits for first PKT_CMD.
//
// ACTIVE
//   Processes PKT_CMD.
//   Sends motor feedback.
//
// ESTOP
//   Motors stopped.
//   Only PKT_RESET can leave this state.
//
//
//
// COMMUNICATION FAILSAFE
//
// If no packet is received for COMMUNICATION_TIMEOUT:
//
//   ACTIVE -> WAITING
//   ESTOP  -> WAITING
//
// Motors are stopped.
//
//
//
// CHECKSUM
//
// XOR over:
//
//   TYPE
//   PAYLOAD[0]
//   PAYLOAD[1]
//   ...
//   PAYLOAD[26]
//
//
//
// IMPORTANT
//
// No Serial.print() or Serial.println() is used.
//
// The serial interface is exclusively used for the
// binary communication protocol.
//
// ============================================================

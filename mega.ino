#include <REG.h>
#include <wit_c_sdk.h>
#include <PS2X_lib.h>
#include <Thread.h>
#include "imu_functions.h"
#include "Motor.h"
#include "DifferentialDrive.h"
#include "variables.h"

int i;
float fAcc[3], fGyro[3], fAngle[3];
float init_x = 0;
float init_y = 0;
float xAngle = 0;
float yAngle = 0;
#define MAX_TILT 60

#define leftMotorPwmPin 9
#define leftMotorDirectionPin 8
#define rightMotorPwmPin 7
#define rightMotorDirectionPin 6

const int maxSpeed = 120;
const int rampSteps = 50;

#define E_STOP 2
bool e_stop = false;
bool auto_mode = true;

unsigned long moveStartTime = 0;
bool moveForwardInProgress = false;

int error = 0;
byte type = 0;
byte vibrate = 0;

unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 100;

PS2X ps2x;
static Thread thread1, thread2, thread3;
DifferentialDrive loco(leftMotorPwmPin, leftMotorDirectionPin, rightMotorPwmPin, rightMotorDirectionPin, maxSpeed, rampSteps);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(E_STOP, OUTPUT);
  digitalWrite(E_STOP, HIGH);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  if (auto_mode) {
    Serial.println("AUTO");
  }

  error = ps2x.config_gamepad(13, 11, 10, 12, false, false);
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  } else if (error == 1) {
    Serial.println("No controller found, check wiring or reset the Arduino");
  } else if (error == 2) {
    Serial.println("Controller found but not accepting commands");
  } else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");
  }
  type = ps2x.readType();
  switch (type) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
  }

  if (Serial1.available()) {
    WitSerialDataIn(Serial1.read());
  }
  if (Serial.available()) {
    CopeCmdData(Serial.read());
  }
  CmdProcess();

  if (s_cDataUpdate) {
    for (i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
    }

    if (s_cDataUpdate & ANGLE_UPDATE) {
      init_x = fAngle[0];
      init_y = fAngle[1];
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    s_cDataUpdate = 0;
  }

  thread1.onRun([]() {
    while (Serial1.available()) {
      WitSerialDataIn(Serial1.read());
    }
    while (Serial.available()) {
      CopeCmdData(Serial.read());
    }
    CmdProcess();

    if (s_cDataUpdate) {
      for (i = 0; i < 2; i++) {
        fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }
      xAngle = fAngle[0] - init_x;
      yAngle = fAngle[1] - init_y;

      if (abs(xAngle) > MAX_TILT || abs(xAngle) > MAX_TILT) {
        e_stop = true;
        Serial.println("ESTOP");
        digitalWrite(E_STOP, LOW);
        delay(100);
      } else {
        e_stop = false;
        digitalWrite(E_STOP, HIGH);
      }

      if (s_cDataUpdate & ANGLE_UPDATE) {
        Serial.print("X:");
        Serial.print(xAngle, 2);
        Serial.print(" Y: ");
        Serial.print(yAngle, 2);
        Serial.print("\r\n");
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }
      s_cDataUpdate = 0;
    }
  });

  thread2.onRun([]() {
    ps2x.read_gamepad(false, vibrate);
    int rightstickry = ps2x.Analog(PSS_RY);
    int rightstickrx = ps2x.Analog(PSS_RX);
    unsigned long currentMillis = millis();

    if (!e_stop) {
      vibrate = 0;
      if (currentMillis - lastButtonPressTime >= debounceDelay) {
        if (ps2x.Button(PSB_PAD_UP)) {
          Serial.println("FORWARD");
          loco.moveForward(0.5, 0.5);
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_LEFT)) {
          Serial.println("LEFT");
          if (!ps2x.Button(PSB_L2)) {
            loco.turnLeft(0.25, 0.75, true);
          } else {
            loco.turnLeft(0.25, 0.75, false);
          }
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_RIGHT)) {
          Serial.println("RIGHT");
          if (!ps2x.Button(PSB_L2)) {
            loco.turnRight(0.75, 0.25, true);
          } else {
            loco.turnRight(0.75, 0.25, false);
          }
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_DOWN)) {
          Serial.println("BACK");
          loco.moveBackward(0.5, 0.5);
          lastButtonPressTime = currentMillis;
        } else if (!ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_PAD_LEFT)) {
          if (ps2x.Button(PSB_L1)) {
            loco.stop();
            lastButtonPressTime = currentMillis;
          }
        } else if (ps2x.Button(PSB_GREEN)) {
          if (auto_mode == true) {
            auto_mode = false;
            Serial.println("MANUAL");
          } else {
            auto_mode = true;
            Serial.println("AUTO");
          }
        }
      }
    }
    delay(50);
  });

  thread3.onRun([]() {
    if (auto_mode) {
      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        if (input.equals("OK")) {
          moveStartTime = millis();
          moveForwardInProgress = true;
          loco.moveForward(0.5, 0.5);
        }
      }

      if (moveForwardInProgress && millis() - moveStartTime >= 3000) {
        loco.stop();
        moveForwardInProgress = false;
        Serial.println("OK");
      }
    } else if (ps2x.Button(PSB_RED)) {
      moveStartTime = millis();
      moveForwardInProgress = true;
      loco.moveForward(0.5, 0.5);
    }
    if (moveForwardInProgress && millis() - moveStartTime >= 3000) {
      loco.stop();
      moveForwardInProgress = false;
      Serial.println("OK_MANUAL");
    }
  });

  thread1.setInterval(50);
  thread2.setInterval(50);
  thread3.setInterval(50);
  thread1.enabled = true;
  thread2.enabled = true;
  thread3.enabled = true;
}

void loop() {
  thread1.run();
  thread2.run();
  thread3.run();
}
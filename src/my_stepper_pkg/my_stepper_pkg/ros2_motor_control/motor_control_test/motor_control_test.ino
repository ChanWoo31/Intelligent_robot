#include <AccelStepper.h>


#define ENABLE_PIN_1 30
#define DIR_PIN_1 31
#define STEP_PIN_1 32

#define ENABLE_PIN_2 34
#define DIR_PIN_2 35
#define STEP_PIN_2 36

#define ENABLE_PIN_3 38
#define DIR_PIN_3 39
#define STEP_PIN_3 40

#define HOME_SWITCH1 22
#define HOME_SWITCH2 23
#define HOME_SWITCH3 24
#define HOME_SWITCH4 25
#define HOME_SWITCH5 26
#define HOME_SWITCH6 27

AccelStepper stepper_1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper_3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

long endSteps1 = 0, endSteps2 = 0, endSteps3 = 0;
long returnSteps1 = 0, returnSteps2 = 0, returnSteps3 = 0;
long stepPerMmX = 0, stepPerMmY = 0;

const float HOMING_SPEED = 2000;

const int MAX_X_MM = 750;
const int MAX_Y_MM = 500;
const int MAX_Z_MM = 430;

void homeEndstop(AccelStepper &mA, int swA, AccelStepper &mB, int swB, float speed) {
  bool doneA = false, doneB = false;
  mA.setSpeed(speed);
  mB.setSpeed(speed);

  while(!(doneA && doneB)) {
    if(!doneA) {
      if (digitalRead(swA) == HIGH) {
        mA.runSpeed();
      } else {
        mA.stop();
        doneA = true;
        Serial.print("motor");
        Serial.print(&mA == &stepper_1 ? "1" : "2");
        Serial.println("homing success");
      }
    }
    if (!doneB) {
      if (digitalRead(swB) == HIGH) {
        mB.runSpeed();
      } else {
        mB.stop();
        doneB = true;
        Serial.print("motor");
        Serial.print(&mB == &stepper_1 ? "1" : "2");
        Serial.println("homing success");
      }
    }
  }
//  while (mA.isRunning() || mB.isRunning()) {
//    mA.run();
//    mB.run();
//  }
}

void homeEndstop2(AccelStepper &mA, int swA, int swB, float speed) {
  mA.setSpeed(-speed);
  // 처음 위치로 와서 0점 맞추기.
  if (digitalRead(swA) == HIGH) {
    mA.runSpeed();
  }
  while (digitalRead(swA) == HIGH) {
    mA.runSpeed();
    delayMicroseconds(50);
  }
  mA.stop();
  mA.setCurrentPosition(0);
  Serial.println("motor homing start position");
  
  while (mA.isRunning()){
    mA.run();
  }

  // 최종 위치로 가기
  mA.setSpeed(speed);
  if (digitalRead(swB) == HIGH) {
    mA.runSpeed();
  }
  while (digitalRead(swB) == HIGH) {
    mA.runSpeed();
    delayMicroseconds(50);
  }
  mA.stop();
  mA.setCurrentPosition(0);
  Serial.println("motor3 homing start position");
  
  while (mA.isRunning()){
    mA.run();
  }
  
  endSteps3 = abs(stepper_3.currentPosition());
  Serial.print("from start to end using steps 3 : "); Serial.println(endSteps3);
  stepper_3.moveTo(0);
  returnSteps3 = abs(stepper_3.distanceToGo());
  while (stepper_3.distanceToGo() != 0) {
    stepper_3.run();
  }

  
  
}


void setup() {
  Serial.begin(115200);
  // 모터1
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);

  // 모터2
  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  // 모터3
  pinMode(ENABLE_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT);

  digitalWrite(ENABLE_PIN_1, HIGH);
  digitalWrite(ENABLE_PIN_2, HIGH);
  digitalWrite(ENABLE_PIN_3, HIGH);

  //스위치 풀업
  pinMode(HOME_SWITCH1, INPUT_PULLUP);
  pinMode(HOME_SWITCH2, INPUT_PULLUP);
  pinMode(HOME_SWITCH3, INPUT_PULLUP);
  pinMode(HOME_SWITCH4, INPUT_PULLUP);
  pinMode(HOME_SWITCH5, INPUT_PULLUP);
  pinMode(HOME_SWITCH6, INPUT_PULLUP);

  // AccelStepper 설정
  stepper_1.setMaxSpeed(4000);
  stepper_1.setAcceleration(2000);

  stepper_2.setMaxSpeed(4000);
  stepper_2.setAcceleration(2000);

  stepper_3.setMaxSpeed(4000);
  stepper_3.setAcceleration(2000);

  //초기 위치 호밍

  Serial.println("setup started");

  homeEndstop(stepper_1, HOME_SWITCH1, stepper_2, HOME_SWITCH3, HOMING_SPEED);
  stepper_1.setCurrentPosition(0);
  stepper_2.setCurrentPosition(0);
  Serial.println("homing success. start line.");

  // 종단 위치 호밍
  homeEndstop(stepper_1, HOME_SWITCH2, stepper_2, HOME_SWITCH4, -HOMING_SPEED);

  endSteps1 = abs(stepper_1.currentPosition());
  endSteps2 = abs(stepper_2.currentPosition());
  Serial.print("from start to end using steps 1 : "); Serial.println(endSteps1);
  Serial.print("from start to end using steps 2 : "); Serial.println(endSteps2);

  // 다시 초기 위치 복귀
  stepper_1.moveTo(0);
  stepper_2.moveTo(0);

  returnSteps1 = abs(stepper_1.distanceToGo());
  returnSteps2 = abs(stepper_2.distanceToGo());
  while (stepper_1.distanceToGo() != 0 || stepper_2.distanceToGo() != 0) {
    stepper_1.run();
    stepper_2.run();
  }

  homeEndstop2(stepper_3, HOME_SWITCH5, HOME_SWITCH6, HOMING_SPEED);

  stepPerMmX = endSteps1 / MAX_X_MM;
  stepPerMmY = endSteps2 / MAX_Y_MM;
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    //
    Serial.print("Recv >");
    Serial.println(line);
    //
    int comma = line.indexOf(',');
    if (comma < 0) return;

    float x_mm = line.substring(0, comma).toFloat();
    float y_mm = line.substring(comma + 1).toFloat();

    long target1 = lround(x_mm * MAX_X_MM);
    long target2 = lround(y_mm * MAX_Y_MM);

    stepper_1.moveTo(target1);
    stepper_2.moveTo(target2);
  }

  if (stepper_1.distanceToGo() != 0 || stepper_2.distanceToGo() != 0) {
    stepper_1.run();
    stepper_2.run();
  }

}

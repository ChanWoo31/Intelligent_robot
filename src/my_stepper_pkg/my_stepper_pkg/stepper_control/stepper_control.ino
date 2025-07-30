#include <AccelStepper.h>

// 파라미터
const long STEPS_PER_REV = 3200; //모터 회전당 스텝 수
const float LEADSCREW_PITCH = 5.0; // mm/회전

const float STEP_MM = LEADSCREW_PITCH / STEPS_PER_REV;

const float HOMING_BACKOFF_MM = 1.0;
const long BACKOFF_STEPS = lround(HOMING_BACKOFF_MM / STEP_MM);

const long MAX_STEPS = 2000000;

#define DIR_PIN_1 33
#define STEP_PIN_1 34
#define ENABLE_PIN_1 32

#define DIR_PIN_2 37
#define STEP_PIN_2 38
#define ENABLE_PIN_2 36

#define HOME_SWITCH 40

AccelStepper stepper_1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

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

  digitalWrite(ENABLE_PIN_1, HIGH);
  digitalWrite(ENABLE_PIN_2, HIGH);

  //스위치 풀업
  pinMode(HOME_SWITCH, INPUT_PULLUP);

  // AccelStepper 설정
  stepper_1.setMaxSpeed(4000);
  stepper_1.setAcceleration(2000);

  stepper_2.setMaxSpeed(4000);
  stepper_2.setAcceleration(2000);

  // 호밍속도
  stepper_1.setSpeed(-1000);
  stepper_2.setSpeed(-1000);

  Serial.println("Homing..");
//  while (digitalRead(HOME_SWITCH) == HIGH) {
//    stepper_1.runSpeed();
//    stepper_2.runSpeed();
//  }
//  delay(20);
  stepper_1.moveTo(-100000);
  while (digitalRead(HOME_SWITCH) == HIGH) stepper_1.run();

  unsigned long t0 = millis();
  while (digitalRead(HOME_SWITCH)==LOW && millis() - t0 < 50) {
    delay(1);
  }
  
//  if (digitalRead(HOME_SWITCH) == LOW) {
//    Serial.println("스위치 눌림");
//  }
  stepper_1.stop();
  stepper_2.stop();
  delay(20);

  stepper_1.moveTo(BACKOFF_STEPS);
  stepper_2.moveTo(BACKOFF_STEPS);
  while (stepper_1.distanceToGo() != 0 || stepper_2.distanceToGo() != 0) {
    stepper_1.run();
    stepper_2.run();
  }
  delay(100);
  
  stepper_1.setCurrentPosition(0);
  stepper_2.setCurrentPosition(0);
  Serial.println("Homing success");
  
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

    long target1 = lround(x_mm / STEP_MM);
    long target2 = lround(y_mm / STEP_MM);

    target1 = constrain(target1, 0, MAX_STEPS);
    target2 = constrain(target2, 0, MAX_STEPS);

    while (stepper_1.isRunning() || stepper_2.isRunning()) {
      stepper_1.run();
      stepper_2.run();
    }

}

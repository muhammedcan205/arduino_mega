#include <SimpleFOC.h>

// BLDC motor & driver instances
BLDCMotor motor1 = BLDCMotor(11, 12.5, 100); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 5, 6, 46);  // Motor 1: PWM pinleri ve enable pini

BLDCMotor motor2 = BLDCMotor(11, 12.5, 100); 
BLDCDriver3PWM driver2 = BLDCDriver3PWM(10, 3, 4, 47); // Motor 2: PWM pinleri ve enable pini

BLDCMotor motor3 = BLDCMotor(11, 12.5, 100); 
BLDCDriver3PWM driver3 = BLDCDriver3PWM(11, 12, 13, 49); // Motor 3: PWM pinleri ve enable pini

BLDCMotor motor4 = BLDCMotor(11, 12.5, 100); 
BLDCDriver3PWM driver4 = BLDCDriver3PWM(2, 7, 8, 48); // Motor 4: PWM pinleri ve enable pini

// instantiate the commander
Commander command = Commander(Serial);
void doTargetMotor1(char* cmd) { command.scalar(&motor1.target, cmd); }
void doTargetMotor2(char* cmd) { command.scalar(&motor2.target, cmd); }
void doTargetMotor3(char* cmd) { command.scalar(&motor3.target, cmd); }
void doTargetMotor4(char* cmd) { command.scalar(&motor4.target, cmd); }
void doLimitCurrentMotor1(char* cmd) { command.scalar(&motor1.current_limit, cmd); }
void doLimitCurrentMotor2(char* cmd) { command.scalar(&motor2.current_limit, cmd); }
void doLimitCurrentMotor3(char* cmd) { command.scalar(&motor3.current_limit, cmd); }
void doLimitCurrentMotor4(char* cmd) { command.scalar(&motor4.current_limit, cmd); }

void setup() {
  // Serial communication
  Serial.begin(115200);
  Serial.println("Motor setup...");

  // driver1 config
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 6;
  if(!driver1.init()) {
    Serial.println("Driver 1 init failed!");
    return;
  }
  motor1.linkDriver(&driver1);
  motor1.current_limit = 0.5;
  motor1.controller = MotionControlType::velocity_openloop;
  motor1.init();
  motor1.initFOC();

  // driver2 config
  driver2.voltage_power_supply = 12;
  driver2.voltage_limit = 6;
  if(!driver2.init()) {
    Serial.println("Driver 2 init failed!");
    return;
  }
  motor2.linkDriver(&driver2);
  motor2.current_limit = 0.5;
  motor2.controller = MotionControlType::velocity_openloop;
  motor2.init();
  motor2.initFOC();

  // driver3 config
  driver3.voltage_power_supply = 12;
  driver3.voltage_limit = 6;
  if(!driver3.init()) {
    Serial.println("Driver 3 init failed!");
    return;
  }
  motor3.linkDriver(&driver3);
  motor3.current_limit = 0.5;
  motor3.controller = MotionControlType::velocity_openloop;
  motor3.init();
  motor3.initFOC();

  // driver4 config
  driver4.voltage_power_supply = 12;
  driver4.voltage_limit = 6;
  if(!driver4.init()) {
    Serial.println("Driver 4 init failed!");
    return;
  }
  motor4.linkDriver(&driver4);
  motor4.current_limit = 0.5;
  motor4.controller = MotionControlType::velocity_openloop;
  motor4.init();
  motor4.initFOC();

  // Commander commands
  command.add('T', doTargetMotor1, "Motor 1 target velocity");
  command.add('Y', doTargetMotor2, "Motor 2 target velocity");
  command.add('U', doTargetMotor3, "Motor 3 target velocity");
  command.add('O', doTargetMotor4, "Motor 4 target velocity");
  command.add('C', doLimitCurrentMotor1, "Motor 1 current limit");
  command.add('D', doLimitCurrentMotor2, "Motor 2 current limit");
  command.add('E', doLimitCurrentMotor3, "Motor 3 current limit");
  command.add('F', doLimitCurrentMotor4, "Motor 4 current limit");

  Serial.println("Motors ready!");
  Serial.println("Set target velocity for Motor 1 with T, Motor 2 with Y, Motor 3 with U, Motor 4 with O");
}

void loop() {
  // Motor 1 control loop
  motor1.loopFOC();
  motor1.move();

  // Motor 2 control loop
  motor2.loopFOC();
  motor2.move();

  // Motor 3 control loop
  motor3.loopFOC();
  motor3.move();

  // Motor 4 control loop
  motor4.loopFOC();
  motor4.move();

  // Debugging outputs
  Serial.print("Motor 1 target: ");
  Serial.print(motor1.target);
  Serial.print(" | Motor 1 actual velocity: ");
  Serial.println(motor1.shaft_velocity);

  Serial.print("Motor 2 target: ");
  Serial.print(motor2.target);
  Serial.print(" | Motor 2 actual velocity: ");
  Serial.println(motor2.shaft_velocity);

  Serial.print("Motor 3 target: ");
  Serial.print(motor3.target);
  Serial.print(" | Motor 3 actual velocity: ");
  Serial.println(motor3.shaft_velocity);

  Serial.print("Motor 4 target: ");
  Serial.print(motor4.target);
  Serial.print(" | Motor 4 actual velocity: ");
  Serial.println(motor4.shaft_velocity);

  // User communication
  command.run();
  delay(500);  // A small delay for readability
}

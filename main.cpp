#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SimpleFOC.h>

#define IN1_1 32 
#define IN2_1 33
#define IN3_1 25
#define IN4_1 26

#define IN1_2 2
#define IN2_2 17
#define IN3_2 16
#define IN4_2 4

BluetoothSerial SerialBT;

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, 5);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, 15);

StepperMotor motor1 = StepperMotor(50, 3, 240);
StepperMotor motor2 = StepperMotor(50, 3, 240);

StepperDriver4PWM driver1 = StepperDriver4PWM(IN1_1, IN2_1, IN3_1, IN4_1);
StepperDriver4PWM driver2 = StepperDriver4PWM(IN1_2, IN2_2, IN3_2, IN4_2);

//Commander command = Commander(Serial);
//void doTarget(char* cmd){command.motion(&motor1, cmd);}
int ks = 5; // Initialize ks
int Km = 5; // Initialize Km

void setup() {
  Serial.begin(115200); 
   Serial.println("Enter value for ks (slave motor):");
  while (!Serial.available()) {
    delay(100);
  }
  ks = Serial.parseInt();

  // Prompt the user to set Km value
  Serial.println("Enter value for Km (master motor):");
  while (!Serial.available()) {
    delay(100);
  }

    Km = Serial.parseInt();

  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  motor1.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_ANGLE;
motor2.monitor_variables = _MON_TARGET | _MON_VOLT_Q| _MON_ANGLE;

motor1.monitor_downsample = 100; // default 10
  motor2.monitor_downsample = 100; // default 10
  
  sensor1.init();
  sensor2.init();

  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  
  motor1.foc_modulation = FOCModulationType::SinePWM;
  motor2.foc_modulation = FOCModulationType::SinePWM;

  driver1.voltage_power_supply = 12;
  driver1.init();
  driver2.voltage_power_supply = 12;
  driver2.init();
  
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  
  motor1.current_limit = 4.0;
  motor2.current_limit = 4.0;
  
 motor1.torque_controller = TorqueControlType::voltage;
 motor2.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
   motor1.init();
  motor2.init();
  
  motor1.initFOC();
  motor2.initFOC();
  

  // add command to commander
    //command.add('M', doTarget, "target");
  delay(1000);
}  
// Define constants for the moving average filter
const int windowSize = 5; // Adjust the window size as needed
float angleBuffer[windowSize];
int bufferIndex = 0;

// Function to update the moving average buffer with a new angle value
void updateAngleBuffer(float newAngle) {
  angleBuffer[bufferIndex] = newAngle;
  bufferIndex = (bufferIndex + 1) % windowSize;
}

// Function to calculate the moving average of the angle
float calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += angleBuffer[i];
  }
  return sum / windowSize;
}


void loop() {
  // Check if there's data available to read from the serial monitor
 
  //slave motor
  motor2.loopFOC();
  motor2.move(ks * (calculateMovingAverage() - motor2.shaft_angle)); // Use moving average for motor1.shaft_angle
  motor2.monitor();
  //master motor
  motor1.loopFOC();
  motor1.move(Km * (motor2.shaft_angle - calculateMovingAverage())); // Use moving average for motor2.shaft_angle
  motor1.monitor();
  updateAngleBuffer(motor1.shaft_angle - motor2.shaft_angle); // Update the moving average buffer
  
  //Serial.println(motor1.shaft_angle - motor2.shaft_angle);
}




#include <Wire.h>
#include <Servo.h>
#define sign(x) (((x) > 0) - ((x) < 0))

// RPM calculation variables
volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long sampleInterval = 600; // 500 ms
float rpm = 0.0;

// MPU and ESC setup
Servo esc;
int minThrottle = 1020;
int maxThrottle = 2000;

// PID constants
float Kp =1.110;
float Ki = 0.586;
float Kd = 0.869;

float error, Pout, Iout, Dout, previousError, output, AccXX, AccYY, AccZZ;
float timeStep = 0.085;
float setAngle = 11;

const int MPU_address = 0x68; // MPU I2C address
float AccX = 0, AccY = 0, AccZ = 0;
float GyroX = 0, GyroY = 0, GyroZ = 0;
float accAngleY;
float accAngleX;

// Moving average buffer for accAngleY
float accAngleYBuffer[20] = {0.0}; // Buffer to store 20 samples
int bufferIndex = 0;
float accAngleYFiltered = 0.0;

void countPulse() {
  pulseCount++;
}

void setup() {
  // Serial communication
  Serial.begin(19200);

  // Initialize MPU
  Wire.begin();
  Wire.beginTransmission(MPU_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_address);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);

  delay(50);

  // Initialize ESC
  esc.attach(9);
  esc.writeMicroseconds(minThrottle);
  delay(2000);
  Serial.println("System initialized. Motor ready.");
}

void loop() {
  unsigned long currentTime = millis();

  // Calculate RPM every sample interval
  if (currentTime - lastTime >= sampleInterval) {
    noInterrupts();
    unsigned int count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Calculate RPM
    rpm = (count * 60.0) / (sampleInterval / 1000.0);
    lastTime = currentTime;
  }

  // Read accelerometer data
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address, 6, true);

  AccX = (Wire.read() << 8 | Wire.read());
  AccXX = AccX / 4096.0 + 0.46;
  AccY = (Wire.read() << 8 | Wire.read());
  AccYY = AccY / 4096.0;
  AccZ = (Wire.read() << 8 | Wire.read());
  AccZZ = AccZ / 4096.0 + 0.06;

  // Calculate raw angles
  accAngleY = (atan2(AccXX, sqrt(pow(AccYY, 2) + pow(AccZZ, 2))) * 180 / PI);
  accAngleX = (atan2(AccYY, sqrt(pow(AccXX, 2) + pow(AccZZ, 2))) * 180 / PI);

  // Update the moving average buffer
  accAngleYBuffer[bufferIndex] = accAngleY;
  bufferIndex = (bufferIndex + 1) % 20;

  // Calculate the moving average
  accAngleYFiltered = 0.0;
  for (int i = 0; i < 20; i++) {
    accAngleYFiltered += accAngleYBuffer[i];
  }
  accAngleYFiltered /= 20;

  // PID control
  error = setAngle - accAngleYFiltered;
  Pout = Kp * error; // Proportional term
  Iout += Ki * error * timeStep; // Integral term
  Dout = Kd * (error - previousError) / timeStep; // Derivative term
  previousError = error;

  // Compute throttle target
  output = minThrottle + Pout + Iout + Dout;

  // Constrain throttle target
  if (output >= maxThrottle) {
    output = maxThrottle;
  } else if (output <= minThrottle) {
    output = minThrottle;
  }

  // Send signal to ESC
  esc.writeMicroseconds(output);

  // Debugging information
  Serial.print("Raw accAngleY: ");
  Serial.print(accAngleY);
  Serial.print(", Filtered accAngleY: ");
  Serial.print(accAngleYFiltered);
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", RPM: ");
  Serial.print(rpm);
  Serial.print(", Output: ");
  Serial.println(output);
/*Serial.print(", GyroX: ");
Serial.print(GyroX);
Serial.print(", GyroY: ");
Serial.print(GyroY);
Serial.print(", GyroZ: ");
Serial.print(GyroZ);
Serial.print(", AccXX: ");
Serial.print(AccXX);
Serial.print(", AccYY: ");
Serial.print(AccYY);
Serial.print(", AccZZ: ");
Serial.print(AccZZ);*/
  // Timing adjustment
  unsigned long loopDuration = millis() - currentTime;
  if (loopDuration < (timeStep * 1000)) {
    delay((timeStep * 1000) - loopDuration);
  }
}

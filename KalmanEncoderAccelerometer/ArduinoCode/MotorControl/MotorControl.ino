// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4


#include <Encoder.h>
#include <RoboLib.h>
const int dataSizeConst = 6;
RoboLib myBot(dataSizeConst,dataSizeConst*5);
float* processedData;
boolean newData = false;
Encoder rightEnc(20, 21);
Encoder leftEnc(19, 18);

 long initialPosRight = 0;
 long initialPosLeft = 0;
void setup() {
    myBot.begin(9600);
    pinMode(A8, INPUT);
    initialPosRight = rightEnc.read(); 
    initialPosLeft = leftEnc.read();
    motor(1, BRAKE, 0);
    motor(2, BRAKE,0);
    motor(4, BRAKE, 0);
    motor(3, BRAKE, 0);
    while(!Serial.available());
    Serial.setTimeout(90);
}

float getVoltage()  {
  return mapf(analogRead(A8),0,1023,0,10);
}
              
void loop() {
  myBot.saveData(); //wait for data
  processedData = myBot.getData(); //get the data
  long initial = millis(); 
  double p14 = mapf(constrain(processedData[0],0,1),0,1,0,255);
  double p23 = mapf(constrain(processedData[1],0,1),0,1,0,255);
  
  if(processedData[2] > 0)  {
      Serial.print(leftEnc.read()-initialPosLeft);
      Serial.print(";");
      Serial.print(rightEnc.read()-initialPosRight);
      Serial.println(";");
      motor(1, BACKWARD, p14);
      motor(2, BACKWARD, p23);
      motor(4, BACKWARD, p14);
      motor(3, BACKWARD, p23);
  }  else  {
      initialPosRight = rightEnc.read(); 
      initialPosLeft = leftEnc.read();

      Serial.println(getVoltage()); //Sends Voltaage
      motor(1, BRAKE, 0);
      motor(2, BRAKE,0);
      motor(4, BRAKE, 0);
      motor(3, BRAKE, 0);
  }
  
  delay(10);

   //initial = millis();
   
  
   
   /*
   while((millis()-initial < 500) && processedData[2] != 0)  {
     Serial.print(leftEnc.read()-initialPosLeft);
     Serial.print(";");
     Serial.println(rightEnc.read()-initialPosRight);
     motor(1, BRAKE, 0);
     motor(2, BRAKE,0);
     motor(4, BRAKE, 0);
     motor(3, BRAKE, 0);
     delay(10);
   }
  */
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}

void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}

void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}


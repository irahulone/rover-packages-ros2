#include <TFMPlus.h> // Include TFMini Plus Library v1.5.0
#include <G2MotorDriver.h>

TFMPlus dispenser_dist_sensor; // Create a TFMini Plus object
TFMPlus auger_dist_sensor;



//  G2MotorDriver24v21(unsigned char DIR, unsigned char PWM, unsigned char SLP, unsigned char FLT, unsigned char CS)
G2MotorDriver24v21 lin_motor = G2MotorDriver24v21(2, 3, 255, 255, 255);
G2MotorDriver24v21 drill_motor = G2MotorDriver24v21(4, 5, 255, 255, 255);

int dispenserBaseDistance = 0;
int augerBaseDistance = 0;
int off = 5;

int num_of_saplings = 0;
bool prev_laser_status = true;


int distanceSensor(TFMPlus tfmP)
{
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  if (tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device.
  {
    return tfDist;
  }
  else // If the command fails...
  {
    return -1;
  }
}

void stopIfFault(G2MotorDriver24v21 *motor)
{
  if (motor->getFault())
  {
    motor->Sleep(); // put the driver to sleep on fault
    delay(1);
    Serial.println("Motor fault");
    motor->Wake();
  }
}

bool blocking(int dist)
{
  if (dist < dispenserBaseDistance - off)
  {
    return true;
  }
  return false;
}

void forwardMotor(G2MotorDriver24v21 *motor, int speed)
{
  motor->setSpeed(speed);
  stopIfFault(motor);
}

void reverseMotor(G2MotorDriver24v21 *motor, int speed)
{
  motor->setSpeed(-speed);
  stopIfFault(motor);
}

void turnOffMotor(G2MotorDriver24v21 *motor)
{
  motor->setSpeed(0);
  stopIfFault(motor);
}

void sendPayload(int d1, int d2, bool block)
{
  Serial.println("START");
  Serial.print("ActuatorDistance:");
  Serial.println(distanceSensor(dispenser_dist_sensor));
  Serial.print("DispenserDistance:");
  Serial.println(distanceSensor(auger_dist_sensor));
  Serial.print("Blocking:");
  Serial.println(block);
  Serial.println("END");
}

bool accept_input(int timeout)
{
  long long start = millis();
  long long current = start;
  bool isInput = false;
  String input;
  while (current - start <= timeout)
  {
    current = millis();
    if (Serial.available())
    {
      input.reserve(40);
      input = Serial.readString();
      input.trim();
      Serial.print(input);
      Serial.println(" Response:");
      Serial.flush();
      isInput = true;
      break;
    }
  }
  if (isInput)
  {
    String command = input.substring(0, 2);
    if (command.compareTo("LF") == 0)
      forwardMotor(&lin_motor, 300);
    else if (command.compareTo("LR") == 0)
      reverseMotor(&lin_motor, 300);
    else if (command.compareTo("LO") == 0)
      turnOffMotor(&lin_motor);
    else if (command.compareTo("DF") == 0)
      forwardMotor(&drill_motor, 200);
    else if (command.compareTo("DR") == 0)
      reverseMotor(&drill_motor, 200);
    else if (command.compareTo("DO") == 0)
      turnOffMotor(&drill_motor);
  }
  return isInput;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("TEST");

  Serial2.begin(115200); // Initialize TFMPLus device serial port.
  delay(20);             // Give port time to initalize
  dispenser_dist_sensor.begin(&Serial2); // Initialize device library object and...
  // pass device serial port to the object.

  Serial3.begin(115200);
  delay(20);
  auger_dist_sensor.begin(&Serial3);


  Serial.print("Firmware version: ");
  if (dispenser_dist_sensor.sendCommand(GET_FIRMWARE_VERSION, 0))
  {
    Serial.print(dispenser_dist_sensor.version[0]); // print three single numbers
    Serial.print(".");
    Serial.print(dispenser_dist_sensor.version[1]); // each separated by a dot
    Serial.print(".");
    Serial.println(dispenser_dist_sensor.version[2]);
  }

  if (auger_dist_sensor.sendCommand(GET_FIRMWARE_VERSION, 0))
  {
    Serial.print(auger_dist_sensor.version[0]); // print three single numbers
    Serial.print(".");
    Serial.print(auger_dist_sensor.version[1]); // each separated by a dot
    Serial.print(".");
    Serial.println(auger_dist_sensor.version[2]);
  }
  // we want to calibrate the sensor to amke sure we know what the max distance is

  Serial.println("HANGUP #1");
  dispenserBaseDistance = distanceSensor(dispenser_dist_sensor);
  Serial.println("HANGUP #2");
  lin_motor.init();
  Serial.println("HANGUP #3");
  lin_motor.Wake(); // Wake the driver for current readings
  Serial.println("HANGUP #4");
  lin_motor.calibrateCurrentOffset();
  delay(10);
  lin_motor.Sleep(); // Put the Motor driver into sleep mode until you need to use it.
  delay(10);

  lin_motor.Wake();

  //    drill_motor.init();
  //    drill_motor.Wake(); // Wake the driver for current readings
  //    drill_motor.calibrateCurrentOffset();
  //    delay(10);
  //    drill_motor.Sleep(); // Put the Motor driver into sleep mode until you need to use it.
  //    delay(10);
  //
  //    drill_motor.Wake();
}

void loop()
{
//  delay(3000);

accept_input(5);
//
int dispenser_dist = distanceSensor(dispenser_dist_sensor);
int auger_dist = distanceSensor(auger_dist_sensor);
bool b = blocking(dispenser_dist);
////
sendPayload(dispenser_dist, auger_dist, b);

//analogWrite(3, 100);
//digitalWrite(2, HIGH);
//
//delay(3000);
//
//analogWrite(3, 100);
//digitalWrite(2, LOW);

  
}

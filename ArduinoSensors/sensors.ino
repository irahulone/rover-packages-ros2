
#include <TFMPlus.h> // Include TFMini Plus Library v1.5.0

TFMPlus dispenser_dist_sensor; // Create a TFMini Plus object
TFMPlus auger_dist_sensor;

int laser_pin = 6;
int reciever_pin = 7;
int distance_sensor_pin = 3;

int num_of_saplings = 0;
bool prev_laser_status = true;

bool getLaserSensor()
{
    return digitalRead(reciever_pin);
}

bool getDistanceSensor()
{
    return digitalRead(distance_sensor_pin);
}

int* distanceSensor(TFMPlus tfmP)
{
    if (tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device.
    {
        return [tfDist, tfFlux];
    }
    else // If the command fails...
    {
        return -1;
    }
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

    pinmode(laser_pin, OUTPUT);
    pinmode(reciever_pin, INPUT);
    pinmode(distance_sensor_pin, INPUT);

    print("Firmware version: ");
    if (dispenser_dist_sensor.sendCommand(GET_FIRMWARE_VERSION, 0))
    {
        print(dispenser_dist_sensor.version[0]); // print three single numbers
        print(".");
        print(dispenser_dist_sensor.version[1]); // each separated by a dot
        print(".");
        println(dispenser_dist_sensor.version[2]);
    }

    if (auger_dist_sensor.sendCommand(GET_FIRMWARE_VERSION, 0))
    {
        print(auger_dist_sensor.version[0]); // print three single numbers
        print(".");
        print(auger_dist_sensor.version[1]); // each separated by a dot
        print(".");
        println(auger_dist_sensor.version[2]);
    }
}

void sendPayload(int passes)
{
    Serial.println("START");
    Serial.print("ActuatorDistance:");
    Serial.println(distanceSensor());
    Serial.print("DispenserDistance:");
    Serial.println(getLaserSensor());
    Serial.println("END");
}

void loop()
{
    delay(100);

    sendPayload(passes);
}
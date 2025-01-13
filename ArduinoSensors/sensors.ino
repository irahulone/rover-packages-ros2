
int laser_pin = 6;
int reciever_pin = 7;
int base_switch_pin = 2;
int end_switch_pin = 3;

int num_of_saplings = 0;
bool prev_laser_status = true;

bool getLaserSensor()
{
    return digitalRead(reciever_pin);
}

bool getBaseSwitchSensor()
{
    return digitalRead(base_switch_pin);
}

bool getEndSwitchSensor()
{
    return digitalRead(end_switch_pin);
}

int numOfPasses()
{
    bool curr_laser_status = getLaserSensor();
    if (curr_laser_status && !prev_laser_status)
    {
        num_of_saplings++;
    }

    prev_laser_status = curr_laser_status;

    return num_of_saplings;
}

void setup()
{
    Serial.begin(9600);
    Serial.println("TEST");
    pinmode(laser_pin, OUTPUT);
    pinmode(reciever_pin, INPUT);
    pinmode(base_switch_pin, INPUT);
    pinmode(end_switch_pin, INPUT);
}

void sendPayload(int passes)
{
    Serial.println("START");
    Serial.print("BaseSwitch:");
    Serial.println(getBaseSwitchSensor());
    Serial.print("EndSwitch:");
    Serial.println(getEndSwitchSensor());
    Serial.print("SaplingPassed:");
    Serial.println(passes);
    Serial.println("END");
}

void loop()
{
    digitalWrite(laser_pin, HIGH);

    int passes = numOfPasses();

    sendPayload(passes);
}
//Control IP: 192.168.1.108	

#include "MQTT.h"

// Pin declarations
pin_t PosControlPin = A4;
pin_t PosFeedbackPin = A3;
pin_t ButtonPin = D3;
pin_t DoorSensorPin = D2;

// Button debounce variables
Timer heartBeat(5, timerInterruptHandler);
const int BOUNCE = 4;
const int LONG_PRESS_COUNT = 400;
const int RESET_PRESS_COUNT = 1000;

enum buttonPressTypes
{
    INACTIVE,
    PRESS,
    LONG_PRESS,
    RESET_PRESS
};

volatile buttonPressTypes buttonEvent;

// MQTT Configuration
void mqttMessageHandler(char *topic, byte *payload, unsigned int length);
const uint8_t server[] = {192, 168, 1, 108};
MQTT client(server, 1883, mqttMessageHandler);

// Servo movement variables
Servo lockServo;
const int servoMinPos = 0;
const int servoMaxPos = 180;
const int servoMinFeedback = 3671;
const int servoMaxFeedback = 511;
const unsigned long timeout = 5000;
const int tolerance = 10;
const int stepInterval = 10;
int unlockedTarget = servoMinPos;
int lockedTarget = servoMaxPos;
int unlockedFeedback = servoMinFeedback;
int lockedFeedback = servoMaxFeedback;
int threshold = (unlockedFeedback + lockedFeedback) / 2;

// EEPROM congfiguration values
int unlockedTargetAddress = 10;
int unlockedFeedbackAddress = unlockedTargetAddress + sizeof(unlockedTarget);
int lockedTargetAddress = unlockedFeedbackAddress + sizeof(unlockedFeedback);
int lockedFeedbackAddress = lockedTargetAddress + sizeof(lockedTarget);
int thresholdAddress = lockedFeedbackAddress + sizeof(lockedFeedbackAddress);

// Flags
bool isLocked = false;
volatile bool wantLocked = false;
volatile bool isClosed = false;
bool lockObstructed = false;
bool cloudUnlockRequest = false;
bool cloudLockRequest = false;
bool controlUnlockRequest = false;
bool controlLockRequest = false;

// State definitions
enum state
{
    WAIT,
    UNLOCKING,
    LOCKING,
    CONFIGURATION,
    INIT,
    ERROR,
};

state lastState = INIT;
state currentState = INIT;
String errorMessage = "";

// ************************** Interrupt handlers *******************************

// On receipt of MQTT message set the appropriate variable.
void mqttMessageHandler(char *topic, byte *payload, unsigned int length)
{
    String stringTopic = topic;
    if (stringTopic == String("Lock"))
    {
        controlLockRequest = true;
    }
    else if (stringTopic == String("Unlock"))
    {
        controlUnlockRequest = true;
    }
}

// Timer interrupt handler to read door sensor and button sensor.
void timerInterruptHandler()
{
    readDoorSensor();
    readButton();
}

// Read and debounce the door sensor then set a flag.
void readDoorSensor()
{
    static int counter = 0;
    bool rawRead = pinReadFast(DoorSensorPin);
    if (rawRead != isClosed)
    {
        counter++;
    }
    else
    {
        counter = 0;
    }

    if (counter > BOUNCE)
    {
        isClosed = rawRead;
        counter = 0;
    }
}

// Read and debounce the button, setting the press type.
void readButton()
{
    static int counter = 0;
    bool rawRead = pinReadFast(ButtonPin);
    if (rawRead)
    {
        counter++;
    }

    if (counter > RESET_PRESS_COUNT && rawRead == false)
    {
        buttonEvent = RESET_PRESS;
        counter = 0;
    }
    else if (counter > LONG_PRESS_COUNT && rawRead == false)
    {
        buttonEvent = LONG_PRESS;
        counter = 0;
    }
    else if (counter > BOUNCE && rawRead == false)
    {
        buttonEvent = PRESS;
        counter = 0;
    }
}

// Handle cloud unlock/lock requests from Particle cloud
int cloudUnlock(String command)
{
    cloudUnlockRequest = true;
    return 0;
}

int cloudLock(String command)
{
    cloudLockRequest = true;
    return 0;
}

// ****************************** Functions ************************************

// Function to check the current lock state
bool checkLocked()
{
    int lockPosition = analogRead(PosFeedbackPin);
    return lockPosition < threshold;
}

// Function to position servo at desired location. Speed is regulated to avoid
// drawing too much current
int setLockPosition(int target, int targetFeedback, unsigned long timeout)
{
    // Attach the servo so we can move it
    lockServo.attach(PosControlPin);
    delay(50);

    unsigned long time = millis();

    int currentPos = map(analogRead(PosFeedbackPin), servoMinFeedback,
        servoMaxFeedback, servoMinPos, servoMaxPos);
    bool incomplete = true;
    while (incomplete)
    {
        if (currentPos < target)
            lockServo.write(++currentPos);
        else
            lockServo.write(--currentPos);
        delay(stepInterval);

        if (millis() - time > timeout)
        {
            lockServo.detach();
            return -1;
        }
        int currentFeedback = analogRead(PosFeedbackPin);
        incomplete = abs(currentFeedback - targetFeedback) > tolerance;
    }

    // Detach the servo so it can be manually moved
    lockServo.detach();
    return analogRead(PosFeedbackPin);
}

// A function to connect to MQTT broker
void connectMQTT()
{
    // Subscribe to MQTT topics
    client.connect("LockModule");
    if (client.isConnected())
    {
        client.subscribe("Unlock");
        client.subscribe("Lock");
        client.publish("Status", "\nConnected");
    }
}

// Function to calculate battery charge percentage based on rolling average of
// raw voltage readings.
int getBatterySOC()
{
    
    // Create a circular buffer to hold the measurements
    const int bufferSize = 100;
    static float batVoltages[bufferSize] = {0};
    static int index = 0;

    // Update the buffer and total with the new value. Initial total assumes all
    // readings equal to first and fills the buffer. Subsequent reads just
    // work with current buffer position and update total to avoid iterations.
    // Constant multiplier for raw read taken from here:
    //    https://docs.particle.io/reference/device-os/firmware/#battery-voltage
    float raw = analogRead(BATT) * 0.0011224;
    static float total = raw * bufferSize;
    static bool first = true;
    if (first)
    {
        for (int i = 0; i < bufferSize; i++)
        {
            batVoltages[i] = raw;
        }
        first = false;
    }
    else
    {
        total -= batVoltages[index];
        batVoltages[index] = analogRead(BATT) * 0.0011224;;
        total += batVoltages[index];
    }
    
    // Move the index in the buffer to the next position, or the start
    index++;
    if (index > bufferSize - 1) index = 0;

    // Calculate average of array
    float avgVoltage = total / bufferSize;

    // Convert the reading to a percent value.
    // Empirical testing determined min reading when 3.7V input is 3.55 and 
    // max reading when 4.2V input is 4.05. As these are the acceptable bounds
    // for single cell LiPo battery we will use these
    // (https://blog.ampow.com/lipo-voltage-chart/)
    // Mapping is from 0 to 100% to present normalized values to user.
    int batPercent = (int)map(avgVoltage, 3.55, 4.05, 0.0, 100.0);
    if (batPercent > 100) batPercent = 100;
    if (batPercent < 0) batPercent = 0;
    return batPercent;
}

// ************************** States for lock **********************************

// Configuration State is used to setup the lock. Set the locked and unlocked
// position
void configuration()
{
    // Publish the current configuration
    if (currentState != lastState)
    {
        Particle.publish("Status", "CONFIGURATION state");
        Particle.publish("Status", "Current Config:\nunlockedFB: "
            + String(unlockedFeedback)
            + "\nunlockedTgt: " + String(unlockedTarget)
            + "\nlockedFB: " + String(lockedFeedback)
            + "\nlockedTgt: " + String(lockedTarget)
            + "\nthreshold: " + String(threshold)
        );
        client.publish("Status", "CONFIGURATION state");
        client.publish("Status", "Current Config:\nunlockedFB: "
            + String(unlockedFeedback)
            + "\nunlockedTgt: " + String(unlockedTarget)
            + "\nlockedFB: " + String(lockedFeedback)
            + "\nlockedTgt: " + String(lockedTarget)
            + "\nthreshold: " + String(threshold)
        );
    }
    lastState = currentState;

    // Handle configuration input
    static int counter = 0;
    if (buttonEvent == PRESS)
    {
        buttonEvent = INACTIVE;

        if (counter == 0)
        {
            // set unlocked position
            unlockedFeedback = analogRead(PosFeedbackPin);
            unlockedTarget = map(unlockedFeedback, servoMinFeedback, 
                servoMaxFeedback, servoMinPos, servoMaxPos);
            counter++;
        }
        else if (counter == 1)
        {
            // set unlocked position
            lockedFeedback = analogRead(PosFeedbackPin);
            lockedTarget = map(lockedFeedback, servoMinFeedback, 
                servoMaxFeedback, servoMinPos, servoMaxPos);
            threshold = (unlockedFeedback + lockedFeedback) / 2;
            counter++;
        }
        else
        {
            // Store the new config in EEPROM and publish an update
            EEPROM.put(unlockedFeedbackAddress, unlockedFeedback);
            EEPROM.put(unlockedTargetAddress, unlockedTarget);
            EEPROM.put(lockedFeedbackAddress, lockedFeedback);
            EEPROM.put(lockedTargetAddress, lockedTarget);
            EEPROM.put(thresholdAddress, threshold);

            Particle.publish("Status", "New Config:\nunlockedFB: "
                + String(unlockedFeedback)
                + "\nunlockedTgt: " + String(unlockedTarget)
                + "\nlockedFB: " + String(lockedFeedback)
                + "\nlockedTgt: " + String(lockedTarget)
                + "\nthreshold: " + String(threshold)
            );
            client.publish("Status", "New Config:\nunlockedFB: "
                + String(unlockedFeedback)
                + "\nunlockedTgt: " + String(unlockedTarget)
                + "\nlockedFB: " + String(lockedFeedback)
                + "\nlockedTgt: " + String(lockedTarget)
                + "\nthreshold: " + String(threshold)
            );
            counter = 0;
            currentState = WAIT;
            return;
        }
    }

    // Exit configuration if a long press. Don't save new settings until all
    // have been entered.
    if (buttonEvent == LONG_PRESS)
    {
        buttonEvent = INACTIVE;
        currentState = WAIT;
    }
}

// Wait state is the default state of the lock
void wait()
{
    // Turn off RGB LED
    RGB.color(0, 0, 0);

    // Update lock state to account for manual lock/unlock
    // static bool lastLocked = checkLocked();
    if (checkLocked())
    {
        isLocked = true;
        wantLocked = true;
    }
    else
    {
        isLocked = false;
        wantLocked = false;
    }

    // Log on state change
    if (currentState != lastState)
    {
        client.publish("Status", "WAIT state");
    }
    lastState = currentState;

    // If the lock state changes then update status
    static bool lastIsLocked = false;
    if (isLocked != lastIsLocked)
    {
        Particle.publish("Status", (isLocked ? "Locked" : "Unlocked"));
        client.publish("Status", (isLocked ? "Locked" : "Unlocked"));
    }
    lastIsLocked = isLocked;

    // If the door state changes then update status
    static bool lastIsClosed = false;
    if (isClosed != lastIsClosed)
    {
        Particle.publish("Status", (isClosed ? "Closed" : "Open"));
        client.publish("Status", (isClosed ? "Closed" : "Open"));
    }
    lastIsClosed = isClosed;

    // Check button events and update variables.
    if (buttonEvent == PRESS)
    {
        wantLocked = !wantLocked;
        buttonEvent = INACTIVE;
    }

    if (buttonEvent == LONG_PRESS)
    {
        currentState = CONFIGURATION;
        buttonEvent = INACTIVE;
    }

    // Handle cloud functions
    if (cloudUnlockRequest == true)
    {
        cloudUnlockRequest = false;
        wantLocked = false;
    }

    if (cloudLockRequest == true)
    {
        cloudLockRequest = false;
        wantLocked = true;
    }

    // Update MQTT messages
    if (client.isConnected())
    {
        client.loop();
    }
    else
    {
        connectMQTT();
    }

    // Handle MQTT functions
    if (controlUnlockRequest == true)
    {
        client.publish("Debug", "Control unlock request");
        controlUnlockRequest = false;
        wantLocked = false;
    }

    if (controlLockRequest == true)
    {
        client.publish("Debug", "Control lock request");
        controlLockRequest = false;
        wantLocked = true;
    }

    // Change to the appropriate state
    // Low battery error
    if (getBatterySOC() < 5)
    {
        errorMessage = "Low Battery";
        currentState = ERROR;
    }
    
    // Door open but locked error
    if (!isClosed && isLocked)
    {
        errorMessage = "Lock actuated while door open";
        currentState = ERROR;
    }

    // Lock the door
    if (wantLocked && !isLocked && isClosed)
    {
        currentState = LOCKING;
    }

    // Unlock the door
    if (!wantLocked && isLocked)
    {
        currentState = UNLOCKING;
    }
}

// Unlocking state
void unlocking()
{
    // Log state entry
    if (currentState != lastState)
    {
        client.publish("Status", "UNLOCKING state");
    }
    lastState = currentState;

    // Move the lock to the unlocked position
    if (setLockPosition(unlockedTarget, unlockedFeedback, timeout) < 0)
    {
        errorMessage = "Unlock timed out";
        currentState = ERROR;
    }
    else
    {
        isLocked = false;
        currentState = WAIT;
    }
    buttonEvent = INACTIVE;
}

// Locking State
void locking()
{
    // Log state entry
    if (currentState != lastState)
    {
        client.publish("Status", "LOCKING state");
    }
    lastState = currentState;

    // Move the lock to the locked position
    if (setLockPosition(lockedTarget, lockedFeedback, timeout) < 0)
    {
        errorMessage = "Lock timed out";
        currentState = ERROR;
    }
    else
    {
        isLocked = true;
        currentState = WAIT;
    }

    buttonEvent = INACTIVE;
}

// Error State
void error()
{
    // Log messages
    if (currentState != lastState)
    {
        client.publish("Status", "ERROR: " + errorMessage);
        Particle.publish("Status", "ERROR: " + errorMessage);
    }
    lastState = currentState;

    // Set state defaults
    RGB.color(255, 0, 0);
    lockServo.detach();

    if (buttonEvent == PRESS)
    {
        buttonEvent = INACTIVE;
        currentState = WAIT;
    }
}

// Use setup as initialisation state
void setup()
{
    // Set pins.
    pinMode(PosControlPin, OUTPUT);
    pinMode(ButtonPin, INPUT_PULLDOWN);
    pinMode(DoorSensorPin, INPUT_PULLUP);

    // Turn of RGB
    RGB.control(true);

    // Start the timer interrupt
    heartBeat.start();

    // Load configuration from EEPROM
    EEPROM.get(unlockedFeedbackAddress, unlockedFeedback);
    EEPROM.get(unlockedTargetAddress, unlockedTarget);
    EEPROM.get(lockedFeedbackAddress, lockedFeedback);
    EEPROM.get(lockedTargetAddress, lockedTarget);
    EEPROM.get(thresholdAddress, threshold);

    // Declare cloud functions
    Particle.function("unlock", cloudUnlock);
    Particle.function("lock", cloudLock);

    // Subscribe to MQTT topics
    connectMQTT();

    // Set the start state
    currentState = WAIT;
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
    // Handle resets for all states
    if (buttonEvent == RESET_PRESS)
    {
        System.reset();
    }

    // Update battery level for all states
    static int lastBatterySOC = -100;
    int currentBatterySOC = getBatterySOC();
    if (abs(currentBatterySOC - lastBatterySOC) > 1)
    {
        client.publish("Status", "Battery: " + String(currentBatterySOC));
    }
    lastBatterySOC = currentBatterySOC;
    
    switch (currentState)
    {
    case WAIT:
        wait();
        break;
    case UNLOCKING:
        unlocking();
        break;
    case LOCKING:
        locking();
        break;
    case CONFIGURATION:
        configuration();
        break;
    case INIT:
        setup();
        break;
    default:
        error();
        break;
    }
}
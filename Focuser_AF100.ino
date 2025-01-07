#include <avr/wdt.h>
#include <AccelStepper.h>
#include <DallasTemperature.h>
#include "Definitions.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <TimerOne.h>

AccelStepper stepper(
  AccelStepper::FULL4WIRE,
  STEPPER_IN_1,
  STEPPER_IN_3,
  STEPPER_IN_2,
  STEPPER_IN_4,
  false);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);

// movement globals
unsigned long currentPosition = 0;
bool isEnabled = false;
bool isMoving = false;
unsigned long lastSavedPosition = 0;
const long millisDisableDelay = 15000;
long millisLastMove = 0;
int speedFactor = 16;
int speedFactorRaw = 4;
unsigned long targetPosition = 0;
float temperatureCoefficient = 0.00;

// serial stuff
String _command = "";
String _inputString = "";
bool _stringComplete = false;
String _param = "";

// setup stuff
bool _pinsInitialized = false;

// function declaration
static void intHandler();

void softwareReset() {
  wdt_enable(WDTO_15MS);
}

void explodeCommand(String input, String delimiter) {
  int index = 0;
  while (input.length() > 0) {
    int pos = input.indexOf(delimiter) > 0 ? input.indexOf(delimiter) : input.length();

    if (index == 0) {
      _command = input.substring(0, pos);
    } else if (index == 1) {
      _param = input.substring(0, pos);
    }

    input.remove(0, pos + delimiter.length());
    index++;
  }
}

void setup() {
  MCUSR = 0;
  wdt_disable();

  Serial.begin(9600);
  _inputString.reserve(200);

  if (!_pinsInitialized) {
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_OUT, INPUT_PULLUP);
    pinMode(SWITCH_SPEED, INPUT_PULLUP);

    digitalWrite(LED_BUILTIN, LOW);
    _pinsInitialized = true;
  }

  EEPROM.get(0, currentPosition);
  currentPosition = max(0, currentPosition);

  stepper.setCurrentPosition(currentPosition);
  lastSavedPosition = currentPosition;
  targetPosition = currentPosition;

  stepper.setMaxSpeed(speedFactor * SPEED_MULTIPLICATOR);
  stepper.setAcceleration(ACCELERATION);

  temperatureSensor.begin();

  Timer1.initialize(PERIOD_US);
  Timer1.attachInterrupt(intHandler);
}

void loop() {
  if (_stringComplete) {
    explodeCommand(_inputString, ":");
    handleCommands(_command, _param);

    _command = "";
    _inputString = "";
    _param = "";
    _stringComplete = false;
  }

  if (stepper.distanceToGo() != 0) {
    millisLastMove = millis();
    currentPosition = stepper.currentPosition();
  } else {
    if (millis() - millisLastMove > millisDisableDelay) {
      if (lastSavedPosition != currentPosition) {
        EEPROM.put(0, currentPosition);
        lastSavedPosition = currentPosition;
      }

      if (isEnabled) {
        stepper.disableOutputs();
        isEnabled = false;
      }
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    _inputString += inChar;
    if (inChar == '\n') {
      _inputString.trim();
      _stringComplete = true;
    }
  }
}

void handleCommands(String command, String param) {
  if (command.equalsIgnoreCase("STATUS")) {
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
  }
  
  if (command.equalsIgnoreCase("RESET")) {
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
    softwareReset();
  }

  if (command.equalsIgnoreCase("GP")) {
    currentPosition = stepper.currentPosition();
    Serial.println(String(currentPosition) + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("GT")) {
    Serial.println(String(getTemperature()) + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("GM")) {
    Serial.println(String(isMoving ? "1" : "0") + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("SP")) {
    currentPosition = param.toInt();
    stepper.setCurrentPosition(currentPosition);
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("ST")) {
    targetPosition = param.toInt();
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("MOVE")) {
    stepper.enableOutputs();
    isEnabled = true;
    stepper.moveTo(targetPosition);
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
  }

  if (command.equalsIgnoreCase("STOP")) {
    stepper.stop();
    Serial.println(String(RESPONSE_OK) + String(TERMINATION_CHAR));
  }
}

static void intHandler() {
  stepper.run();
  isMoving = stepper.distanceToGo() != 0;
  digitalWrite(LED_BUILTIN, isMoving);
}

float getTemperature() {
  temperatureSensor.requestTemperatures();
  float temperature = temperatureSensor.getTempCByIndex(0);
  if (temperature > 100 || temperature < -50) {
    temperature = 0;
  }

  return temperature;
}

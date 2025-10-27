

//libraries
#include "Adafruit_BluefruitLE_SPI.h"
#include <Adafruit_MotorShield.h>


//constants relating to our adafruit board (these were found online)

#define VERBOSE_MODE false

#define MOTOR_A_TERMINAL 1
#define MOTOR_B_TERMINAL 2
#define MOTOR_C_TERMINAL 3
#define MOTOR_D_TERMINAL 4
#define BLUEFRUIT_SPI_CS 8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 4
#define pin1 9
#define pin2 6
#define pin3 5

// Instantiating the motorshield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *MOTOR_A = AFMS.getMotor(MOTOR_A_TERMINAL);
Adafruit_DCMotor *MOTOR_B = AFMS.getMotor(MOTOR_B_TERMINAL);
Adafruit_DCMotor *MOTOR_C = AFMS.getMotor(MOTOR_C_TERMINAL);
Adafruit_DCMotor *MOTOR_D = AFMS.getMotor(MOTOR_D_TERMINAL);


//Define the bluetooth module object


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);



// A function that prints the error


void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1)
    ;
}


void setup() {


  // Initialise the bluetooth module
  if (!ble.begin(VERBOSE_MODE)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }


  // Perform a factory reset to make sure everything is in a known state


  if (!ble.factoryReset()) {
    error(F("Couldn't factory reset"));
  }


  //Disable command echo from Bluefruit so commands aren't sent back for confirmation
  ble.echo(false);


  //ble.info() can be used to get info about the bluefruit module


  // prevents module from sending extensive debug info
  ble.verbose(false);

  //Wait for connection
  while (!ble.isConnected()) {
    delay(500);
  }




  // Set module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // put your setup code here, to run once:
  // Turn on motorshield and PWM driver
  AFMS.begin();
  //Reset A
  MOTOR_A->setSpeed(0);
  MOTOR_A->run(RELEASE);
  //Reset B
  MOTOR_B->setSpeed(0);
  MOTOR_B->run(RELEASE);
  //Reset C
  MOTOR_C->setSpeed(0);
  MOTOR_C->run(RELEASE);
  //Reset D
  MOTOR_D->setSpeed(0);
  MOTOR_D->run(RELEASE);
}

bool motorsRun = false;

void loop() {
  char button;
  char mode;

  //control motors based on inputs
  int sensorVal1 = digitalRead(pin1);
  int sensorVal2 = digitalRead(pin2);
  int sensorVal3 = digitalRead(pin3);
  if (ble.available()) {
    button = getInput();
    //Serial.print("button");
    Serial.print(button);
  }

  if (ble.available()) {
    mode = getMode();
    //Serial.print("mode");
    Serial.print(mode);
  }
  
  if (mode == '4') {
    if (ble.available()) {
    button = getInput();
    Serial.print(button);
  }
    if (button == '8') {  //right
      MOTOR_A->setSpeed(250);
      MOTOR_A->run(BACKWARD);
      MOTOR_D->setSpeed(250);
      MOTOR_D->run(FORWARD);
      MOTOR_B->setSpeed(0);
      MOTOR_B->run(FORWARD);
      MOTOR_C->setSpeed(0);
      MOTOR_C->run(BACKWARD);
    } else if (button == '7') {  //left
      MOTOR_A->setSpeed(0);
      MOTOR_A->run(FORWARD);
      MOTOR_D->setSpeed(0);
      MOTOR_D->run(BACKWARD);
      MOTOR_B->setSpeed(250);
      MOTOR_B->run(BACKWARD);
      MOTOR_C->setSpeed(250);
      MOTOR_C->run(FORWARD);
    } else if (button == '5') {  //forward
      MOTOR_A->setSpeed(250);
      MOTOR_A->run(BACKWARD);
      MOTOR_B->setSpeed(250);
      MOTOR_B->run(BACKWARD);
      MOTOR_C->setSpeed(250);
      MOTOR_C->run(FORWARD);
      MOTOR_D->setSpeed(250);
      MOTOR_D->run(FORWARD);
    } else if (button == '6') {  //back
      MOTOR_A->setSpeed(250);
      MOTOR_A->run(FORWARD);
      MOTOR_B->setSpeed(250);
      MOTOR_B->run(FORWARD);
      MOTOR_C->setSpeed(250);
      MOTOR_C->run(BACKWARD);
      MOTOR_D->setSpeed(250);
      MOTOR_D->run(BACKWARD);
    } else {
      MOTOR_A->setSpeed(0);
      MOTOR_A->run(RELEASE);
      //Reset B
      MOTOR_B->setSpeed(0);
      MOTOR_B->run(RELEASE);
      //Reset C
      MOTOR_C->setSpeed(0);
      MOTOR_C->run(RELEASE);
      //Reset D
      MOTOR_D->setSpeed(0);
      MOTOR_D->run(RELEASE);
    }
  }else if ((mode == '2') || (button == '2')){

      if ((sensorVal1 == 1) && (sensorVal2 == 0)) {
        MOTOR_A->setSpeed(5);
        MOTOR_A->run(FORWARD);
        MOTOR_D->setSpeed(5);
        MOTOR_D->run(BACKWARD);
        MOTOR_B->setSpeed(250);
        MOTOR_B->run(BACKWARD);
        MOTOR_C->setSpeed(250);
        MOTOR_C->run(FORWARD);
      } else if ((sensorVal1 == 0) && (sensorVal2 == 1)) {
        MOTOR_A->setSpeed(250);
        MOTOR_A->run(BACKWARD);
        MOTOR_D->setSpeed(250);
        MOTOR_D->run(FORWARD);
        MOTOR_B->setSpeed(5);
        MOTOR_B->run(FORWARD);
        MOTOR_C->setSpeed(5);
        MOTOR_C->run(BACKWARD);
      } else{
        MOTOR_A->setSpeed(250);
        MOTOR_A->run(BACKWARD);
        MOTOR_B->setSpeed(250);
        MOTOR_B->run(BACKWARD);
        MOTOR_C->setSpeed(250);
        MOTOR_C->run(FORWARD);
        MOTOR_D->setSpeed(250);
        MOTOR_D->run(FORWARD);
      }
    
  }else if(mode == '0'){
        MOTOR_A->setSpeed(0);
        MOTOR_A->run(RELEASE);
          //Reset B
        MOTOR_B->setSpeed(0);
        MOTOR_B->run(RELEASE);
          //Reset C
        MOTOR_C->setSpeed(0);
        MOTOR_C->run(RELEASE);
          //Reset D
        MOTOR_D->setSpeed(0);
        MOTOR_D->run(RELEASE);
    }
}



//function that returns mode

char getMode() {
  char mode = ble.read();
  char mode2 = ble.read();
  char mode3 = ble.read();
  char mode4 = ble.read();
  char mode5 = ble.read();
  if (((mode3 = '4') && (mode4 == '1')) || ((mode3 == '2') && (mode4 == '1'))) {
    return mode3;
  }
  else if(mode4 = '0'){
    return mode4;
  }
}

//function that reads the next 5 chars, returns the last button pressed
char getInput() {
  char reading = ble.read();
  char reading2 = ble.read();
  char reading3 = ble.read();
  char reading4 = ble.read();
  char reading5 = ble.read();
  if (reading3 == '2'){
    return reading3;
  }
  if (reading4 == '1') {
    return reading3;
  }
  else if (reading4 == '0') {
    return reading4;
  }
  //use variable = ble.read() to read a single char
}

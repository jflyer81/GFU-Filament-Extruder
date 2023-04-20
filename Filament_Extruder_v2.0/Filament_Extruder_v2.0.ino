// Filament Extruder Project
// George Fox University

// Development started by Team Alpha - Servant Engineering 2021-22
// Megan Clausen, Brian Rogers, Grant Walker, Josiah Armstrong, Nile Haynes-Irby and James Marcelia

// Continued by James Marcelia 2022-23

// Code last revised 10 January 2023 by James Marcelia
// Contact jmarcelia19@georgefox.edu


//Button does something if HIGH, buttons are normal inputs

/*
   To Do:
   incorporate buttons library (either ezButton or https://github.com/madleech/Button)
   Fix UI
*/

// bool debuggingTemp = 1;
// bool debuggingDia = 0;

// ************************************** pin declaration *******************************************************
// Diameter Control
const int diaSensorPin = A1;  // analog pin for reading diameter from the bearing sensor
const int winderMotor = 31;

// Themp Control
const int thermistorNozzle = A15;  // aka pin 41; thermistor on the nozzle
const int thermistorMid = A14;     // aka pin 40; thermistor in between the two heaters
const int thermistorTip = A17;     // aka pin 39; thermistor between nozzle and heaters
const int thermistorHopper = A16;  // aka pin 38; thermistor at the base of the hopper

const int heatSleevesPin = 32;  // Heater sleeves output pin
const int augerFanPin = -1;     // PWM control for the auger fan
const int filFanPin = -1;

// User I/O
const int leftButton = 1;
const int rightButton = 2;
const int xButton = 0;
const int selectButton = 3;  // UI button pins
#define BUTTON_ACTIVE_STATE LOW

const int potPin1 = A9;  // pin 23 for potentiometer

const int greenLED = 22;
const int yellowLED = 21;
const int redLED = 20;  // UI LED output pins


// Input Limits
const int MaxTemp = 2600;
const int MinTemp = 0;  // max barrel temp: 260.0 deg C (decimal omitted on left)
const int MaxDia = 225;
const int MinDia = 100;  // max/min diameters: 2.25-1.00 millimeters (decimal omitted in code)

// Vars for UI control
unsigned long lastSelectButtonPress = 0;
unsigned long lastRightButtonPress = 0;
unsigned long lastLeftButtonPress = 0;
unsigned long lastXButtonPress = 0;


// Vars for Diameter Control moving averages ************************************
#define NUMSAMPLES_dia 500
#define DIA_FILTER_T 10    //seconds
int samples_dia[NUMSAMPLES_dia];
int diaSampleTimer = 0;
float averageDia;
double userDia = 1.75;
double realDia;

//serial and lcd print help *****************************************************
unsigned long lastSerialPrint = 0;
unsigned long lastLCDPrint = 0;



// ***** Temperature Control Reading Variables / Inputs ***************************
#define baseResistance 100000  // Ω - the resistance of the thermistor @ 25 deg C (in data sheet)
#define Btherm 3950            // K - the B value of the thermistor (in data sheet)
#define VCC 3.3                // Supply voltage
#define res 10000              // R=10KΩ - The resistor that is used with the thermistor
#define NUMSAMPLES 20          // Number of samples taken for temperature readings

// vars for converting thermistor inputs to temperatures
float thermistorResistance;
float thermistorVoltage;
float logg;
float temp;
float baseTemperature;

float voltageRideThrough;
float currentTemp;
float average;
float currentTime;
float userTemp;
float lastStep;

unsigned long startPreHeatTime;

// vars to hold temperature outputs at different locations
double TempMid;
double TempTip;
double TempNozzle;
double TempHopper;
double TempAvgAB;
double TempAvgABNozzle;
double TempSpread;

// vars for temperature reading running averages for noise reduction
int samples_temp[NUMSAMPLES];
int waitTime;

unsigned long lastPrintDia = 0;

// the error margine for our "bang bang" heater control
const int upperTempBoundary = 1;  // if temp is this much over the target, turn heaters OFF
const int lowerTempBoundary = 1;  // if temp is this much below target temp, turn heaters ON
// ***** Temperature Control Reading Variables / Inputs END ************************


// =====================================================================================
// ==================================  Libraries  ======================================
// =====================================================================================

//--------------------------- initialize LCD library -----------------------------------
//#include <LiquidCrystal_I2C.h>       // includes the library
//LiquidCrystal_I2C lcd(0x27, 20, 4);  // creates our 20x4 lcd object with I2C address 0x27


#include <Wire.h>
#include <hd44780.h>                        // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header
hd44780_I2Cexp lcd;                         // declare lcd object: auto locate & auto config expander chip

// ========================= MOTOR CONTROL CONSTANTS =======================
//--------------------------- initialize PID library ------------------------------------
#include "ArduPID.h"
ArduPID diaPID;

double setpoint;      // var to which we will assign our target diameter
double diaInput;      // parameter which we are trying to get to setpoint
double pullerOutput;  // var which we will map to motor speed
double p = 0.0001;       // proportional (const)
double i = 0.0003;     // integral term (const)
double d = 50000;       // derivative term (const)

ArduPID tempPID;
double tempSetpoint;
double tempInput;
double tempOutput;
double p1 = 20;
double i1 = 0.0;
double d1 = 0.0;

unsigned long lastTempChange;
double dutyCycle;
int tempPeriod = 2000;  // mHz

//-------------------------- initialize stepper library ------------------------------
#include <ContinuousStepper_TeensyTimerTool.h>
TeensyTimerTool::PeriodicTimer timer;
ContinuousStepper_TeensyTimerTool stepper(timer);

int motorSpeed = 0;         // var to hold the commanded motor speed
int prevMotorSpeed = 0;     // var to hold value the last time we updated the motor speed input
int speedChangeBuffer = 5;  // if motorspeed is less than 5 from prevMotorSpeed, we don't change anything

const int maxMotorSpeed = 2000;  // max and min puller motor speeds
const int minMotorSpeed = 150;

const int stepDirPin = 35;     // direction pin for stepper driver
const int stepStepPin = 34;    // step pin for stepper driver
const int stepEnablePin = 33;  // enable pin (LOW == ENABLED)

int motorSpeedScaled;
int winderSpeed = 255;


// ----------------------------------------------------- State Machine ----------------------------------------------------
// statemachine states
enum States {
  USER,
  PRINT_PRE_HEAT,
  PRE_HEAT,
  PRINT_EXTRUDING,
  EXTRUDING,
  PRINT_DONE,
  DONE
};


// Menu states
enum Menu {
  PRINT_DIA,
  DIA,
  PRINT_TEMP,
  TEMP,
  PRINT_CONFIRM,
  CONFIRM
};

// Extruding States
enum Extruding {
  NOTHING,
  PRINT_ADJUST_TEMP,
  ADJUST_TEMP,
  PRINT_ADJUST_DIA,
  ADJUST_DIA
};

States state = USER;
Menu menu = DIA;
Extruding extruding = NOTHING;

void setup() {
  pinMode(winderMotor, OUTPUT);
  analogWrite(winderMotor, winderSpeed);

  Serial.begin(9600);  // Note: Teensy doesn't use the baud rate

  // init. diameter PID
  diaPID.begin(&diaInput, &pullerOutput, &setpoint, p, i, d);
  diaPID.start();

  // init. temp PID
  tempPID.begin(&tempInput, &tempOutput, &tempSetpoint, p1, i1, d1);
  tempPID.start();

  stepper.begin(stepStepPin, stepDirPin);

  // Temperature baseTemperature from datasheet, conversion from Celsius to kelvin
  baseTemperature = 25 + 273.15;

  // ----------------------------- Pin Declaration ----------------------------
  // heat sleeves and heat sink fan
  pinMode(heatSleevesPin, OUTPUT);
  pinMode(filFanPin, OUTPUT);
  pinMode(augerFanPin, OUTPUT);

  // diameter sensor and thermistors
  pinMode(diaSensorPin, INPUT);
  pinMode(thermistorMid, INPUT);
  pinMode(thermistorTip, INPUT);
  pinMode(thermistorNozzle, INPUT);
  pinMode(thermistorHopper, INPUT);

  // UI buttons                                                                 // Change this to use input pullup and eliminate external pulldowns/ups
  pinMode(leftButton, INPUT);
  pinMode(rightButton, INPUT);
  pinMode(xButton, INPUT);
  pinMode(selectButton, INPUT);

  // LED outputs
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // stepper motor manual outputs
  pinMode(stepEnablePin, OUTPUT);


  // set start points for user inputs
  userDia = 1.75;
  userTemp = 208;

  //initializes the lcd and turns on the backlight
  lcd.init();
  lcd.backlight();  // function lcd.noBacklight() turns off the lcd backlight

  //disable stepper until we need it
  digitalWrite(stepEnablePin, HIGH);


  // Set up initial states
  state = USER;         // machine state USER
  menu = PRINT_TEMP;    // menu state PRINT_TEMP
  extruding = NOTHING;  // extrude state NOTHING
}

void loop() {
  /*
      The machine is initialized into the state machine USER.
      Within USER state is the "menu" state machine. "menu" boots into PRINT_TEMP,
      which prompts the user for the desired temperature for extrusion
  */

  // ========================================== Main State Machine "state" =============================================
  switch (state) {

      //======================================== USER state (state sm) ========================================//
      //======================================================================================================//
    case USER:
      {
        /*
            The USER state is the machine state in which the user is prompted to enter parameters
            for the upcoming extrude job. Inside USER is a "menu" state machine which sequentially
            progresses, prompting the user for different extrusion parameters.
        */

        switch (menu) {
            //================================ menu State Machine "menu" =======================================

          case PRINT_TEMP:
            {
              // ----------------------- PRINT_TEMP state (menu sm) ---------------------------//
              /*
                  This case prompts the user to enter a desired barrel temperature and then
                  switches over to the next case, TEMP, in which the temperature is actually read.
              */
              getAllTemps();

              // disables fans, heaters, and extrusion processes
              digitalWrite(filFanPin, LOW);       // disable filament fans
              digitalWrite(heatSleevesPin, LOW);  // disable heaters
              extruding = NOTHING;                // set extruding state machine to NOTHING state

              // prints prompt
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Enter Desired");
              lcd.setCursor(0, 1);
              lcd.print("Barrel Temp in C");

              menu = TEMP;  // move directly to TEMP state of the "menu" state machine
            }
            break;


          case TEMP:
            {
              // ------------------------------ TEMP state (menu sm) -------------------------------//
              /*
                 This menu state is where the user adjusts the userTemp via the incremement function;
                 userTemp is displayed to the user as it is adjusted, and only when the user presses
                 "select" does the machine move to the next state, PRINT_DIA
              */
              // reads user input using the Increment() function
              userTemp = Increment(userTemp, MaxTemp, MinTemp);

              // print currently selected temperature to lcd screen
              lcd.setCursor(12, 3);
              lcd.print(userTemp);

              getAllTemps();
              safteyHeatCheck(TempMid, 50);

              // exit conditions
              // If user hits select, move to the next state
              if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {  // simplify by using button library
                // Serial.print("select button pushed");

                menu = PRINT_DIA;  // move to next menu state
              }
            }
            break;

          case PRINT_DIA:
            {
              // ------------------------------ PRINT_DIA state (menu sm) -------------------------------//
              /*
                 This menu state is just like state PRINT_TEMP. It merely prints the request for the user to enter
                 a desired filament diameter and then moves directly into the next state, DIA.
              */

              // Serial.println("in PRINT_DIA");

              // prints prompt
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Enter Desired");
              lcd.setCursor(0, 1);
              lcd.print("Filament Diameter");
              lcd.setCursor(0, 2);
              lcd.print("in units of mm:");

              // sends user to next state
              menu = DIA;
            }
            break;

          case DIA:
            {
              // ------------------------------ DIA state (menu sm) -------------------------------//
              /*
                 This state is like TEMP. using the increment() function to allow the user to adjust desired diameter and then
                 moving to the next state when "select" is pressed.
              */

              // Heater check
              getAllTemps();  //reads all temperatures using the getTemp() function
              safteyHeatCheck(TempMid, 50);

              // transfer diameter input from from decimal form to whole number (this helps the Increment function)
              userDia = userDia * 100;

              //read user input for diameter
              userDia = Increment(userDia, MaxDia, MinDia);

              //user Dia goes back to decimal form for lcd printing
              userDia = userDia / 100;

              /*
                 because incrememnt() uses integers, it is necessary to multiply the float (double?) userDia by 100
                 to allow the user to enter the 2 decimals for diameter in mm.
              */

              // prints the user Diameter to the lcd screen
              lcd.setCursor(12, 3);
              lcd.print(userDia);

              // exit condition if select is pressed.                                           // replace this with button library stuff
              if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
                Serial.println("select Button Pressed");
                menu = PRINT_CONFIRM;
              }
            }
            break;

          case PRINT_CONFIRM:
            {
              // --------------------------- PRINT_CONFIRM state (menu sm) ----------------------------//
              /*
                 This function prints a message to the screen and then moves
                 directly to the CONFIRM state to read user input
              */

              // Serial.println("in PRINT_CONFIRM");

              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("To Begin Extruding");
              lcd.setCursor(0, 1);
              lcd.print("Press Select");
              lcd.setCursor(0, 2);
              lcd.print("To Change Inputs");
              lcd.setCursor(0, 3);
              lcd.print("Press X");

              menu = CONFIRM;
            }
            break;

          case CONFIRM:
            {
              // --------------------------- CONFIRM state (menu sm) ----------------------------//
              /*
                 This state just listens user input;
                 If select is pressed, we move out of the USER case of the main "state" state machine.
                 If X is pressed, we remian in USER of "state" and go back to "PRINT_TEMP" of "menu" in order to
                 allow the user to re-enter extrusion parameters
              */

              //monitors saftey and red LED based on temp Mid
              getAllTemps();  //reads all temperatures using the getTemp() function
              safteyHeatCheck(TempMid, 50);

              //exit ccondition 1, sends user to pre heating stage if select pressed
              if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
                //                        Serial.println("select Button Pressed");
                state = PRINT_PRE_HEAT;
              }
              // exit condition 2, allows user to change inputs again if x pressed
              if (Debounce(digitalRead(xButton), &lastXButtonPress)) {
                //                        Serial.println("x button Pressed");
                menu = PRINT_TEMP;
              }
            }
            break;
        }
      }
      break;
    //======================================== USER state end (state sm) ========================================//
    //==========================================================================================================//


    //======================================== PRINT_PRE_HEAT state begin (state sm) ============================//
    //==========================================================================================================//
    case PRINT_PRE_HEAT:
      {
        /*
           This case is executed once at the beginning of an extrude cycle.
        */
        lcd.clear();
        // Serial.print("in PRINT_PRE_HEAT");

        // print out the current temp and the user temp
        lcd.setCursor(0, 0);
        lcd.print("heating...");
        lcd.setCursor(0, 1);
        lcd.print("");
        lcd.setCursor(0, 2);
        lcd.print("User Temp:");
        lcd.setCursor(0, 3);
        lcd.print("Current NTemp:");


        // turn yellow light on to indicate heating but not ready
        digitalWrite(yellowLED, HIGH);

        startPreHeatTime = millis();  // note what time we started heating
        state = PRE_HEAT;             // send us directly to the PRE_HEAT state
      }
      break;


    //======================================== PRE_HEAT state begin (state sm) ==================================//
    //==========================================================================================================//
    case PRE_HEAT:
      {
        //displays the current temp of the nozzle end and the set user temp
        lcd.setCursor(14, 2);
        lcd.print(userTemp);
        lcd.setCursor(14, 3);
        lcd.print(TempNozzle);

        getAllTemps();                 //reads all temperatures using the getTemp() function
        safteyHeatCheck(TempMid, 50);  // call safetyHeatCheck to monitor temp limits and warn user if appropriate
        printAllTemps();               // [for debugging] prints all Temp readouts to serial monitor
        ControlTemp(TempMid);          // manages Heat sleeves using A and B temp averages


        // exit condition 1
        /*
           This occurs when the NOZZLE temperature reaches the userTemp.
           At this point, we turn off the yellow LED and move directly to PRINT_EXTRUDING
        */
        if (TempNozzle >= userTemp - 50) {
          //turns yellow LED off
          digitalWrite(yellowLED, LOW);
          lcd.clear();
          state = PRINT_EXTRUDING;
        }

        // exit condition 2, if user wants to cancel preheat completely, can push xButton to return input stage
        if (Debounce(digitalRead(xButton), &lastXButtonPress)) {  // button library!!!
          Serial.println("x Button Pressed");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Pre Heat");
          lcd.setCursor(0, 1);
          lcd.print("Cancelled...");
          // turns yellow LED off
          digitalWrite(yellowLED, LOW);
          state = USER;
          menu = PRINT_TEMP;
        }
      }
      break;
    //--------------------------pre-heat END------------------------------------------//


    //--------------------------extruding state---------------------------------------//
    case PRINT_EXTRUDING:
      {
        // Serial.print("in PRINT_EXTRUDING");


        lcd.clear();

        // These print commands are the LCD UI elements which are NOT continuously updated during extrusion
        lcd.setCursor(0, 0);
        lcd.print(" User Temp: ");
        lcd.setCursor(0, 1);
        lcd.print("Current Temp: ");
        lcd.setCursor(0, 2);
        lcd.print(" User Dia: ");
        lcd.setCursor(0, 3);
        lcd.print("current Dia: ");

        lcd.setCursor(19, 0);
        lcd.print("C");
        lcd.setCursor(19, 1);
        lcd.print("C");
        lcd.setCursor(18, 2);
        lcd.print("mm");
        lcd.setCursor(18, 3);
        lcd.print("mm");

        digitalWrite(greenLED, HIGH);
        //digitalWrite(enablePin, LOW);

        state = EXTRUDING;  // moves directly into EXTRUDING without pausing
      }
      break;



    case EXTRUDING:
      {

        //condition to enter into
        if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
          extruding = PRINT_ADJUST_TEMP;
        }

        //=======================================================================================//
        // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = //
        // There is a new state machine embedded in the "EXTRUDING" case of the main "state" state machine.
        // This embedded state machine is called "extruding"
        // It includes the states, "NOTHING", (PRINT_)ADJUST_TEMP, (PRINT_)ADJUST_DIA,
        // This switch state allows user inputs to be adjusted while extruding
        switch (extruding) {
          case NOTHING:

            break;

          case PRINT_ADJUST_TEMP:
            {
              lcd.setCursor(0, 0);
              lcd.print(">");
              extruding = ADJUST_TEMP;
            }
            break;

          case ADJUST_TEMP:
            {
              // reads user input
              userTemp = Increment(userTemp, MaxTemp, MinTemp);

              if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
                extruding = PRINT_ADJUST_DIA;
              }
            }
            break;

          case PRINT_ADJUST_DIA:
            {
              lcd.setCursor(0, 0);
              lcd.print(" ");
              lcd.setCursor(0, 2);
              lcd.print(">");
              extruding = ADJUST_DIA;
            }
            break;

          case ADJUST_DIA:
            {

              //transfer diameter input from from decimal form to whole number (this helps the Increment function)
              userDia = userDia * 100;
              //read user input for diameter
              userDia = Increment(userDia, MaxDia, MinDia);
              //user Dia goes back to decimal form for lcd printing
              userDia = userDia / 100;

              if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
                lcd.setCursor(0, 2);
                lcd.print(" ");
                extruding = NOTHING;
              }
            }
            break;
        }
        // = = = = = = = = = = End of embedded state machine "extruding" = = = = = = = = = = = = //
        // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = //
        //=======================================================================================//

        digitalWrite(filFanPin, HIGH);  // enable filament cooling


        //this bit serial print of code is here for testing purposes
        // printAllTemps();
        getAllTemps();  //reads all temperatures using the getTemp() function
        safteyHeatCheck(TempMid, 50);



        //manages Heat sleeve temp, (i.e. turns them off if system gets to hot!)
        ControlTemp(TempAvgAB);

        //========================================================= MOTOR CONTROL CODE =====================================================//

        ControlDiameter();
        ControlWinder();

        printDiameters();
        printAllTemps();


        //--------------------------------------------------- end motor control code -----------------------------------------------------//

        //things being printed while machine is extruding
        lcd.setCursor(13, 0);
        lcd.print(userTemp);
        lcd.setCursor(13, 1);
        lcd.print(TempNozzle);
        lcd.setCursor(13, 2);
        lcd.print(userDia);
        lcd.setCursor(13, 3);
        lcd.print(diaInput);


        // exit condition, when user hits X button, system will restart
        if (Debounce(digitalRead(xButton), &lastXButtonPress)) {
          digitalWrite(greenLED, LOW);
          //                 Serial.println("x Button Pressed");
          digitalWrite(stepEnablePin, HIGH);  // disable stepper
          state = PRINT_DONE;
        }
      }
      break;
    //-------------------------- extruding state END -------------------------------------//

    //-------------------------- done state ----------------------------------------------//
    case PRINT_DONE:
      {
        Serial.println("PRINT_DONE");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Extrude Again?");

        state = DONE;
      }
    case DONE:
      {

        // reads temperature and gets an average
        TempNozzle = getTemp(thermistorNozzle);
        TempMid = getTemp(thermistorMid);
        TempTip = getTemp(thermistorTip);
        TempHopper = getTemp(thermistorHopper);
        TempAvgAB = (TempMid + TempTip) / 2;
        TempAvgABNozzle = (TempMid + TempTip + TempNozzle) / 3;

        printAllTemps();
        digitalWrite(filFanPin, LOW);  // disable filament cooling


        //turns heating sleeves off
        digitalWrite(heatSleevesPin, LOW);

        //monitors saftey and red LED based on temp A
        safteyHeatCheck(TempMid, 50);


        // exit condition for DONE state
        if (Debounce(digitalRead(selectButton), &lastSelectButtonPress)) {
          Serial.println("encoder Button Pressed");
          state = USER;
          menu = PRINT_TEMP;
        }
      }
      //--------------------------done state END------------------------------------------//
  }
}



/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/****************************** Separate Functions *******************************/


void safteyHeatCheck(double temp, double threshold) {
  // turns red LED on if barrel is above a certain temperature
  if (temp > threshold) {
    digitalWrite(redLED, HIGH);
  } else {
    digitalWrite(redLED, LOW);
  }
}

bool Debounce(int buttonRead, unsigned long* lastButtonPress) {
  // debounces any button
  int ret = false;
  if (millis() - *lastButtonPress > 250) {
    if (buttonRead == BUTTON_ACTIVE_STATE) {
      ret = true;
    } else {
      ret = false;
    }
    *lastButtonPress = millis();
  }
  //lastButtonPress = millis();
  return ret;
}


void getAllTemps() {
  // reads temperature and gets an average
  TempNozzle = getTemp(thermistorNozzle);                  // temp at nozzle
  TempMid = getTemp(thermistorMid);                        // temp between heaters
  TempTip = getTemp(thermistorTip);                        // temp between heater and nozzle
  TempHopper = getTemp(thermistorHopper);                  // temp at hopper base
  TempAvgAB = (TempMid + TempTip) / 2;                     // avg of middle temp
  TempAvgABNozzle = (TempMid + TempTip + TempNozzle) / 3;  // avg of all thermistors in hot zone

  TempSpread = max(max(TempMid, TempTip), TempNozzle) - min(min(TempMid, TempTip), TempNozzle);  // the max difference between hot zone temps

  // Auger Fan Control
  if (TempHopper > 25) {
    digitalWrite(augerFanPin, HIGH);
  } else {
    digitalWrite(augerFanPin, LOW);
  }
}


double getTemp(int pin) {
  uint8_t i;  // declaring i as a variable

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples_temp[i] = analogRead(pin);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples_temp[i];
  }

  average /= NUMSAMPLES;  // dividing the added samples by the number of samples to obtain an average reading

  // thermistor maths
  voltageRideThrough = average;                                           // Average readings of the voltage ride through
  voltageRideThrough = (VCC / 1023.00) * voltageRideThrough;              //Conversion to voltage
  thermistorVoltage = VCC - voltageRideThrough;                           // test
  thermistorResistance = voltageRideThrough / (thermistorVoltage / res);  // current resistance of thermistor - Resistance of RT
  logg = log(thermistorResistance / baseResistance);                      // thermistor maths
  temp = (1 / ((logg / Btherm) + (1 / baseTemperature)));                 // Temperature from thermistor in K
  temp = temp - 273.15;                                                   // Convert to Celsius
  return temp;
}



double getDia() {
  //take N samples in a row, with a slight delay
  //place them into the samples[] array
  if (diaSampleTimer < millis()){

    //shift ring buffer:
    for (int i = NUMSAMPLES_dia - 1; i > 0; i--) {
      //samples_dia[i] = analogRead(diaSensorPin);
      //Serial.println(analogRead(diaSensorPin));
      samples_dia[i] = samples_dia[i - 1];
    }
    samples_dia[0] = analogRead(diaSensorPin);
    diaSampleTimer = millis() + ((DIA_FILTER_T * 1000) / NUMSAMPLES_dia );
  }
  // average all the datapoints in samples[] out
  averageDia = 0;
  for (int i = 0; i < NUMSAMPLES_dia; i++) {  //sum all the samples
    averageDia += samples_dia[i];
  }

  averageDia /= NUMSAMPLES_dia;  //divide by numSamples


  //realDia = 1.74 + 0.0108 * (averageDia) - (4.76E-05) * (averageDia) * (averageDia);
  realDia = 1.89 + 0.0108 * (averageDia) - (4.76E-05) * (averageDia) * (averageDia);
  if (averageDia < 100) {
    realDia = 2.40;
  }
  //Serial.println(realDia);

  //return averageDia;     // return value for diameter in analog form

  //Serial.println(realDia);

  return realDia;  // return value for diameter in mm
}


double Increment(int butter, int MAX, int MIN) {
  /*
     A functiont to allow the user to use buttons to increment a value
     up and down and then return that value when a button is pressed.
     Butter is the name of the variable which
  */

  if (digitalRead(leftButton) == BUTTON_ACTIVE_STATE) {
    if (millis() - lastLeftButtonPress > 100) {
      if (digitalRead(xButton) == BUTTON_ACTIVE_STATE) {
        butter = butter - 10;
      } else {
        butter--;
      }
      // keeps input no less than minimum value
      if (butter < MIN) {
        butter = MIN;
      }
      lastLeftButtonPress = millis();
    }
    // Remember last button press event
    //lastButtonPress = millis();
  }

  if (digitalRead(rightButton) == BUTTON_ACTIVE_STATE) {
    if (millis() - lastRightButtonPress > 100) {
      if (digitalRead(xButton) == BUTTON_ACTIVE_STATE) {
        butter = butter + 10;
      } else {
        butter++;
      }
      // keeps input only as high as max value
      if (butter > MAX) {
        butter = MAX;
      }
      lastRightButtonPress = millis();
    }
    // Remember last button press event
    //lastButtonPress = millis();
  }
  return butter;
}



//------------------------- Function ControlDiameter --------------------------
void ControlDiameter() {
  //gets reading for the actual diameter of the filament
  setpoint = userDia;

  diaInput = getDia();
  //computes PID and maps to motor output
  diaPID.compute();
  motorSpeed = map(pullerOutput, 0, 255, maxMotorSpeed, minMotorSpeed);
  motorSpeedScaled = map(motorSpeed, minMotorSpeed, maxMotorSpeed, 0, 100);


  // The PID outputs a value between 0 and 255 (with 124 being the value when we are spot-on?)
  // So what this does it it takes current speed and increments speed up or down depending on the PID output
  // when we reach steady state, the PID will be outputting a constant 124 and speed will also remain constant
  // adjust P, I, and D values (found where PID is initialized, approx line 105) to adjust PID response rate

  //This code prevents motorSpeed from becoming negative or becoming larger than maximum allowable speed (set where PID is set up)
  if (motorSpeed < 0) {
    motorSpeed = 0;
  }
  if (motorSpeed > maxMotorSpeed) {
    motorSpeed = maxMotorSpeed;
    Serial.println("Maximum Motor Command Exceeded");
  }
  if (motorSpeed < minMotorSpeed) {
    motorSpeed = minMotorSpeed;
  }

  if (prevMotorSpeed <= motorSpeed - speedChangeBuffer || prevMotorSpeed >= motorSpeed + speedChangeBuffer) {
    stepper.spin(motorSpeed);
    //Serial.println("=== Speed Change ===");
    prevMotorSpeed = motorSpeed;
  }


  //this code simply disables the stepper driver if the motor speed is below a certain threshold. This prevents the driver buzzing and being annoying but isn't too important.
  if (motorSpeed < 40) {
    digitalWrite(stepEnablePin, HIGH);
    Serial.println("Motor Disabled");
  } else {
    digitalWrite(stepEnablePin, LOW);
  }
}

void ControlWinder() {

  if (stepEnablePin == LOW) {
    double scalingFactor = map(analogRead(potPin1), 0, 1024, 0, 1);

    winderSpeed = (55 + 200 * motorSpeedScaled / 100) * scalingFactor;

  } else {
    winderSpeed = 0;
  }
  //analogWrite(winderMotor, analogRead(potPin1));
  digitalWrite(winderMotor, LOW);
  // Serial.print("winderSpeed:");
  // Serial.println(analogRead(potPin1));
}

// ------------------------ Function ControlTemp() ---------------------------
// Controls the heater sleeves based on input temperature
void ControlTemp(double temp) {

  tempSetpoint = userTemp;

  tempInput = temp;
  //computes PID
  tempPID.compute();

  //this does temp control!!!!!!!!

  dutyCycle = map(tempOutput, 0, 255, 0, 1);


  // Serial.println(dutyCycle);

  // The PID outputs a value between 0 and 255 with 124 being the value when we are spot-on
  // So what this does it it takes current temp and if temp is less than setpoint, dutyCycle will be close to 1
  // When temp is greater than setpoint, dutyCycle will be closer to 0
  // adjust P, I, and D values (found where PID is initialized, approx line 105) to adjust PID response rate

  currentTime = millis();
  if (currentTime - lastTempChange < dutyCycle * tempPeriod) {  // only check/adjust heater state every 0.333 seconds
    digitalWrite(heatSleevesPin, HIGH);                         // the heat sleeves will be turned off
    // Serial.println("Heaters on");
  } else if (currentTime - lastTempChange < tempPeriod) {
    digitalWrite(heatSleevesPin, LOW);
    // Serial.println("Heaters off");
  } else if (currentTime - lastTempChange > tempPeriod) {
    lastTempChange = millis();
    // Serial.println("Next Cycle!");
    // Serial.print("dutyCycle:");
    // Serial.print(dutyCycle);
    // Serial.print(",");
  }
}

// -------------------------- Function printWelcomeScreen() --------------------------
// UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED UNUSED
void printWelcomeScreen() {
  // Prints Greeting For Filament Extruder, delays, and clears screen - lcd (columns, rows)
  lcd.setCursor(0, 0);
  lcd.print("Extruder 1.0");
  lcd.setCursor(0, 1);
  lcd.print("Presented by:");
  lcd.setCursor(0, 2);
  lcd.print("Team Alpha");
  lcd.setCursor(0, 3);
  lcd.print("system loading");
  delay(1000);
  lcd.setCursor(15, 3);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(16, 3);
  lcd.print(".");
  delay(1000);
  lcd.setCursor(17, 3);
  lcd.print(".");
  delay(1000);
  delay(2000);
  lcd.clear();
}

// --------------------------- Serial Print Code ---------------------------------
void printAllTemps() {
  //this bit serial print of code is here for testing purposes
  if (millis() - lastSerialPrint > 500) {
    Serial.print("TempNozzle:");
    Serial.print(TempNozzle);
    Serial.print(",");
    Serial.print("TempMid:");
    Serial.print(TempMid);
    Serial.print(",");
    Serial.print("TempTip:");
    Serial.print(TempTip);
    Serial.print(",");

    // Serial.print("TempAvgAB:");
    // Serial.print(TempAvgAB);
    // Serial.print(",");

    Serial.print("TempHopper:");
    Serial.print(TempHopper);
    Serial.println("");

    lastSerialPrint = millis();
  }
}

void printDiameters() {

  // This is a section of code that will print setpoint and diameter for tuning the diameter PID control
  if (millis() - lastPrintDia > 500) {

    Serial.print("Dia_Setpoint:");
    Serial.print(setpoint * 100);
    Serial.print(",");
    Serial.print("Diameter:");
    Serial.print(diaInput * 100);
    Serial.print(",");
    Serial.print("PID_Output:");
    Serial.print(pullerOutput);
    Serial.print(",");
    Serial.print("Motor_Command%:");
    Serial.print(motorSpeedScaled);
    // Serial.print(",");
    // Serial.print("Winder_Speed:");
    // Serial.print(winderSpeed);
    Serial.println("");
    lastPrintDia = millis();
  }
}

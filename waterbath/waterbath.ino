//-------------------------------------------------------------------
//
// Water Bath Controller
//
// Based oSous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregarda
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit RGB/LCD Shield
#include <Wire.h>
#include <LiquidCrystal.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 13

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 2


// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;

#define btnNONE   0
#define BUTTON_UP     1
#define BUTTON_DOWN   2
#define BUTTON_LEFT   3
#define BUTTON_SHIFT 4
#define BUTTON_RIGHT  5


// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);
 

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Initialize LCD DiSplay 

   lcd.begin(16, 2);
   lcd.createChar(1, degree); // create degree symbol from the binary
   
   lcd.clear();
   //lcd.setBacklight(VIOLET);
   lcd.setCursor(0, 0);
   lcd.print(F("   Water Bath"));
   lcd.setCursor(0, 1);
   lcd.print(F("   by Jill Xu"));


   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      LcdClearLine(1);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Splash screen



   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);
   
   
  // Run timer1 interrupt every 15 ms (roughly 67Hz) 
  noInterrupts();           // disable all interrupts  
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

// check the following link how to calculate OCR1A
// http://www.instructables.com/id/Arduino-Timer-Interrupts/step1/Prescalers-and-the-Compare-Match-Register/

  OCR1A = 932;            // compare match register 16MHz/256/67Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  
  interrupts();             // enable all interrupts
}


ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  //Serial.print("millis in timer1 int: ");
  //Serial.println(millis());
  
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

void printStatus()
{
  char buf[32]; // needs to be at least large enough to fit the formatted text
  
  dtostrf(Setpoint, 2, 2, buf);
  String parameters = String("Setpoint=")+String(buf) +" ";
  
  dtostrf(Kp, 2, 2, buf);
  parameters += String("Kp=")+String(buf) +" ";
  
  dtostrf(Kd, 2, 2, buf);
  parameters += String("Kd=")+String(buf) +" ";

  dtostrf(Ki, 2, 2, buf);
  parameters += String("Ki=")+String(buf);
     
  Serial.println(parameters);  
  
  sensors.requestTemperatures();
  
  Serial.print("The temperature from sensor is: ");
  Serial.println(sensors.getTempCByIndex(0));
  
  Serial.print("Input and Output: ");
  Serial.print(Input);
  Serial.print(", ");
  Serial.println(Output);
  
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   //printStatus();
   
  
   
   // wait for button release before changing state
   while(ReadButtons() != 0) {}

   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
   
  
}

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
 if (adc_key_in < 50)   return BUTTON_RIGHT;  
 if (adc_key_in < 250)  return BUTTON_UP; 
 if (adc_key_in < 450)  return BUTTON_DOWN; 
 if (adc_key_in < 650)  return BUTTON_LEFT; 
 if (adc_key_in < 850)  return BUTTON_SHIFT;  
 
 return btnNONE;  // when all others fail, return this...
}


// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   //lcd.setBacklight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.print(F("    Water Bath"));
   lcd.setCursor(0, 1);
   lcd.print(F("    off state"));
   uint8_t buttons = 0;

   while(buttons != BUTTON_RIGHT)
   {
      buttons = ReadButtons();
   }
   
   Serial.println("Right button is pressed");
   printStatus();
   
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Temperature:"));
   Serial.println("Entering Tune Set point");
   uint8_t buttons = 0;

   while(true)
   {
      buttons = ReadButtons();
      
      float increment = 0.1; 
      /*
      Serial.print("buttons= ");
      Serial.println(buttons);
      */
      if (buttons == BUTTON_SHIFT)
      {
        Serial.println("button shift is pressed");
        increment *= 10;
        Serial.print("increment= ");
        Serial.println(increment);
      }
      if (buttons == BUTTON_LEFT)
      {
         opState = RUN;
         return;
      }
      if (buttons == BUTTON_RIGHT)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons == BUTTON_UP)
      {
         Setpoint += increment;
         delay(200);
      }
      if (buttons == BUTTON_DOWN)
      {
         Serial.println("reduce setpoint");
         Setpoint -= increment;
         Serial.print("increment= ");
         Serial.println(increment);
         delay(200);
      }
    
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Kp"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1.0;
      if (buttons == BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons == BUTTON_LEFT)
      {
         opState = SETP;
         return;
      }
      if (buttons == BUTTON_RIGHT)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons == BUTTON_UP)
      {
         Kp += increment;
         delay(200);
      }
      if (buttons == BUTTON_DOWN)
      {
         Kp -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Ki"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.01;
      if (buttons == BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons == BUTTON_LEFT)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons == BUTTON_RIGHT)
      {
         opState = TUNE_D;
         return;
      }
      if (buttons == BUTTON_UP)
      {
         Ki += increment;
         delay(200);
      }
      if (buttons == BUTTON_DOWN)
      {
         Ki -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
   //lcd.setBacklight(TEAL);
   lcd.print(F("Set Kd"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 0.01;
      if (buttons == BUTTON_SHIFT)
      {
        increment *= 10;
      }
      if (buttons == BUTTON_LEFT)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons == BUTTON_RIGHT)
      {
         opState = RUN;
         return;
      }
      if (buttons == BUTTON_UP)
      {
         Kd += increment;
         delay(200);
      }
      if (buttons == BUTTON_DOWN)
      {
         Kd -= increment;
         delay(200);
      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   Serial.println("Entering Run state");
   // set up the LCD's number of rows and columns: 
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(1);
   lcd.print(F("C : "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      setBacklight();  // set backlight based on state

      buttons = ReadButtons();
      if ((buttons == BUTTON_SHIFT) 
         //&& (buttons == BUTTON_RIGHT) 
         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
         Serial.println("Entering auto tune");
         StartAutoTune();
      }
      else if (buttons == BUTTON_RIGHT)
      {
        opState = SETP;
        return;
      }
      else if (buttons == BUTTON_LEFT)
      {
        opState = OFF;
        return;
      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(10,1);
      //lcd.print(F("      "));
      lcd.setCursor(9,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        lastLogTime = millis();
        
        printStatus();
        /**
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
        **/
      }

      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        Serial.println("finish auto tune.");
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     //Serial.println("Turn on the heater");
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     //Serial.println("Shut off the heater");
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
      //lcd.setBacklight(VIOLET); // Tuning Mode
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
      //lcd.setBacklight(RED);  // High Alarm - off by more than 1 degree
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      //lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
   }
   else
   {
      //lcd.setBacklight(WHITE);  // We're on target!
   }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  //uint8_t buttons = lcd.readButtons();
  uint8_t buttons = read_LCD_buttons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint) || Setpoint > 40 || Setpoint < 0)
   {
     Setpoint = 20;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

void LcdClearLine(int r)
{
  lcd.setCursor(0,r);
  lcd.print("                ");
  lcd.setCursor(0,r);
}

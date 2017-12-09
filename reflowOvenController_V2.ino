/* Original ********************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com

/* Ver. 2.0 Modifications ******************************************************
* Title: Reflow Oven Controller
* Version: 2.0
* Date: 06-12-2017
* Company: Mr & Mrs Robot
* Modifier: Lachlan Mares
* Website: 

* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
  Lead-Free Reflow Curve
  ======================
  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)
  Leaded Reflow Curve (Kester EP256)
  ==================================
  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
* - MAX6675 Library (for board v1.50 & below):
*   >> https://github.com/adafruit/MAX6675-library
*
* Revision  Description
* ========  ===========
* 2.0       - Add additional reflow profile
*           - Greater flexability of temperature profiles
*           - Use hardware PWM for SSR relay control
*           
* 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*					  - Uses analog based switch (allowing D2 & D3 to be used for user 
*						  application).	
*						Adds waiting state when temperature too hot to start reflow process.
*						Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/

// Comment either one the following #define to select your board revision
// Newer board version starts from v1.60 using MAX31855KASA+ chip 

#define  USE_MAX31855

// ***** INCLUDES *****
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <MAX31855.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_HEAT,
  REFLOW_STATE_LEVEL,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COOL_DOWN,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH
{
	SWITCH_NONE,
	SWITCH_1,	
	SWITCH_2
}	switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;

// ***** CONSTANTS *****
#define PROFILE_TYPE_ADDRESS       0                // EEPROM storage address for profile selection

#define TEMPERATURE_ROOM           50               
#define SEGMENTS                   20               // Number of segments between profile points

// Lead-Free Reflow Curve
#define TEMPERATURE_SOAK_MIN_LF    150              // Lead free Profile temperatures (degrees)
#define TEMPERATURE_SOAK_MAX_LF    200
#define TEMPERATURE_REFLOW_MAX_LF  250
#define TEMPERATURE_REFLOW_MIN_LF  200
#define TEMPERATURE_COOL_MIN_LF    100
#define PREHEAT_DURATION_LF        80000            // Lead free Profile durations (milliseconds)
#define SOAK_DURATION_LF           105000
#define REFLOW_HEAT_DURATION_LF    55000
#define REFLOW_LEVEL_DURATION_LF   5000
#define REFLOW_COOL_DURATION_LF    50000

// Leaded Reflow Curve
#define TEMPERATURE_SOAK_MIN_PB    150              // Leaded Profile temperatures (degrees)
#define TEMPERATURE_SOAK_MAX_PB    180
#define TEMPERATURE_REFLOW_MAX_PB  220
#define TEMPERATURE_REFLOW_MIN_PB  180
#define TEMPERATURE_COOL_MIN_PB    100
#define PREHEAT_DURATION_PB        80000            // Leaded Profile durations (milliseconds)
#define SOAK_DURATION_PB           80000
#define REFLOW_HEAT_DURATION_PB    45000
#define REFLOW_LEVEL_DURATION_PB   5000
#define REFLOW_COOL_DURATION_PB    40000

#define SENSOR_SAMPLING_TIME       200              // Period between thermocouple reading (milliseconds)
#define ONE_SECOND                 1000
#define DEBOUNCE_PERIOD_MIN        50

// ***** PID PARAMETERS *****
#define PID_SAMPLE_TIME            100
#define PWM_MAX                    255              // Pin 5 on Arduino has pwm functionality values (0 to 255)

// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT             100
#define PID_KI_PREHEAT             0.025
#define PID_KD_PREHEAT             20

// ***** SOAKING STAGE *****
#define PID_KP_SOAK                300
#define PID_KI_SOAK                0.05
#define PID_KD_SOAK                250

// ***** REFLOW STAGE *****
#define PID_KP_REFLOW              300
#define PID_KI_REFLOW              0.05
#define PID_KD_REFLOW              350

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow H",
  "Reflow L",
  "Reflow C",
  "Cooling",
  "Complete",
  "Wait,hot",
  "Error"
};

const char* lcdMessagesReflowProfile[] = {
  "LF",
  "PB"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
#ifdef	USE_MAX31855
	int ssrPin = 5;
	int thermocoupleSOPin = A3;
	int thermocoupleCSPin = A2;
	int thermocoupleCLKPin = A1;
	int lcdRsPin = 7;
	int lcdEPin = 8;
	int lcdD4Pin = 9;
	int lcdD5Pin = 10;
	int lcdD6Pin = 11;
	int lcdD7Pin = 12;
	int ledRedPin = 4;
	int buzzerPin = 6;
	int switchPin = A0;
#else
	int ssrPin = 5;
	int thermocoupleSOPin = A5;
	int thermocoupleCSPin = A4;
	int thermocoupleCLKPin = A3;
	int lcdRsPin = 7;
	int lcdEPin = 8;
	int lcdD4Pin = 9;
	int lcdD5Pin = 10;
	int lcdD6Pin = 11;
	int lcdD7Pin = 12;
	int ledRedPin = A1;
	int ledGreenPin = A0;
	int buzzerPin = 6;
	int switch1Pin = 2;
	int switch2Pin = 3;
#endif

// ***** VARIABLES *****
bool buzzerBit;
double setpoint;                                                                            // PID setpoint 
double input = 0.0f;                                                                        // PID input
double newInput[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};                                        // Array for moving average filter
double output;                                                                              // PID output
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
double tempSoakMin;
double tempSoakMax;
double tempReflowMax;
double tempReflowMin;
double tempCoolMin;
double preHeatTempStep;
double soakTempStep;          
double reflowHeatTempStep;  
double reflowCoolTempStep; 
int readFaultCounter=0;
int timerSeconds = 0;                                                                       // Seconds timer
int preHeatTempTime;
int soakTempTime; 
int reflowHeatTempTime;
int reflowLevelTime;
int reflowCoolTempTime;   
unsigned long lastReadSensor = 0;                                                           // Sensor read timer
unsigned long lastSecond = 0;                                                               // One second timer 
unsigned long lastTimer = 0;                                                                // Segement timer
unsigned long lastUpdate = 0;                                                               // PID update timer
long lastDebounceTime;                                                                      // Switch debounce timer

reflowState_t reflowState;                                                                  // Reflow oven controller state machine state variable
reflowStatus_t reflowStatus;                                                                // Reflow oven controller status
debounceState_t debounceState;                                                              // Switch debounce state machine state variable
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
reflowProfile_t reflowProfile;                                                              // Reflow Profile

PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);                          // Specify PID control interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);               // Specify LCD interface
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin,thermocoupleCLKPin);             // Specify MAX31855 thermocouple interface

void setup()
{
  pinMode(ssrPin, OUTPUT);                                                                  // Set ssrPin as output
  pinMode(buzzerPin, OUTPUT);                                                               // Set buzzer pin as output
  pinMode(ledRedPin, OUTPUT);                                                               // Set LED pin as output
  
  analogWrite(ssrPin, 0);                                                                   // SSR pin initialization to ensure reflow oven is off
  digitalWrite(buzzerPin, LOW);                                                             // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(ledRedPin, LOW);                                                             // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(buzzerPin, HIGH);                                                            // Start-up splash
 
  Serial.begin(57600);                                                                      // Serial communication at 57600 bps
  
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();
  lcd.print("Reflow");
  lcd.setCursor(0, 1);
  lcd.print("Oven v2");
  digitalWrite(buzzerPin, LOW);
  delay(2500);
  lcd.clear();                                                                  

  digitalWrite(ledRedPin, HIGH);                                                            // Turn off LED (active low)  
  
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);                                  // Search EEPROM for last profile used
  if ((value == 0) || (value == 1))                                                         // If exists
  {
    reflowProfile = value;                                                                  // Valid reflow profile value
  } else
    {
      EEPROM.write(PROFILE_TYPE_ADDRESS, 0);                                                // Store lead-free profile into EEPROM
      reflowProfile = REFLOW_PROFILE_LEADFREE;                                              // Set current profile
    }
  loadProfile(reflowProfile);                                                               // Load profile into working variables
  reflowStatus = REFLOW_STATUS_OFF;                                                         // Reflow off
}

void loop()
{
  if(abs(millis() - lastReadSensor) > SENSOR_SAMPLING_TIME)                                 // Time to read thermocouple?
  {
    newInput[0] = thermocouple.readThermocouple(CELSIUS);                                   // Get new temperature reading
    if (thermocoupleFault(newInput[0]))                                                     // If error reported
    {
      readFaultCounter++;                                                                   // Increment fault counter
    } else
      {
	readFaultCounter = 0;                                                               // Valid reading reset counter
        input = (newInput[0]+newInput[1]+newInput[2]+newInput[3]+newInput[4])/5.0f;         // Compute average of last 5 readings
        newInput[4] = newInput[3];                                                          // Update moving average filter
        newInput[3] = newInput[2];
        newInput[2] = newInput[1];
        newInput[1] = newInput[0];
      }        
                       
    if(readFaultCounter > 10)                                                               // If too many bad readings
    {
      reflowState = REFLOW_STATE_ERROR;                                                     // Change to ERROR state
    }   
    lastReadSensor = millis();
  }

  if (abs(millis() - lastSecond) > ONE_SECOND)                                              // Updates every 1000mS
  {
    if (reflowStatus == REFLOW_STATUS_ON)                                                   // If reflow process is on going
    {   
      digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));                                   // Toggle red LED as system heart beat
      timerSeconds++;                                                                       // Increase seconds timer for reflow curve analysis 
      Serial.print(lcdMessagesReflowStatus[reflowState]);                                   // Print current mode
      Serial.print(",");                                                                    // Use comma to create a .csv file from outptut
      Serial.print(timerSeconds);                                                           // Send temperature and time stamp to serial
      Serial.print(",");
      Serial.print(setpoint);                                                               // Current Setpoint
      Serial.print(",");
      Serial.print(input);                                                                  // Current Temperature
      Serial.print(",");
      Serial.println(output);                                                               // Current PID output (0 to MAX_PWM)
    } else
      {
        digitalWrite(ledRedPin, HIGH);                                                      // Turn off red LED
      }

    lcd.clear();                                                                            // Clear LCD  
    lcd.print(lcdMessagesReflowStatus[reflowState]);                                        // Print current system state
    if (reflowState == REFLOW_STATE_IDLE)
    {
      lcd.print(" ");
      lcd.print(lcdMessagesReflowProfile[reflowProfile]);
    }
    lcd.setCursor(0, 1);                                                                    // Move the cursor to the 2 line

    if (reflowState == REFLOW_STATE_ERROR)                                                  // If currently in error state
    {
      reflowStatus = REFLOW_STATUS_OFF;
      lcd.print("TC Error!");                                                               // No thermocouple wire connected
    } else
      {
        lcd.print(input);                                                                   // Print current temperature
	#if ARDUINO >= 100
	  lcd.write((uint8_t)0);                                                            // Print degree Celsius symbol
	#else
	  lcd.print(0, BYTE);                                                               // Print degree Celsius symbol
	#endif
        lcd.print("C ");
      }
    lastSecond = millis();
  }

  switch (reflowState)                                                                      // Reflow oven controller state machine
  {
    case REFLOW_STATE_IDLE:                                                                 // If state = IDLE
      if (input >= TEMPERATURE_ROOM)                                                        // If oven temperature is still above room temperature
      {
	reflowState = REFLOW_STATE_TOO_HOT;                                                 // Change to TOO_HOT state 
        reflowStatus = REFLOW_STATUS_OFF;                                                   // Reflow status OFF
      } else if (switchStatus == SWITCH_1)                                                  // If switch 1 is pressed to start reflow process
        {
          Serial.println("State,Time,Setpoint,Input,Output");                               // Send header for CSV file
          timerSeconds = 0;                                                                 // Intialize seconds timer for serial debug information
          setpoint = input;                                                                 // Start heating from current temperature
          reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);         // Set PID paramaters to Preheat profile                                        
          reflowOvenPID.SetOutputLimits(0, PWM_MAX);                                        // Tell the PID to range between 0 and PWM_MAX
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);                                     // Set PID sample time
          reflowOvenPID.SetMode(AUTOMATIC);                                                 // Turn the PID on
          reflowState = REFLOW_STATE_PREHEAT;                                               // Proceed to preheat stage
          lastTimer = millis();
        } 
        
        if (switchStatus == SWITCH_2)                                                       // If switch 2 is pressed change profiles 
        {
          reflowProfile = (reflowProfile == REFLOW_PROFILE_LEADFREE) ? REFLOW_PROFILE_LEADED : REFLOW_PROFILE_LEADFREE;
          EEPROM.write(PROFILE_TYPE_ADDRESS, reflowProfile);                                // Store new profile in EEPROM
          loadProfile(reflowProfile);                                                       // Load profile into working variables
        }
      break;

    case REFLOW_STATE_PREHEAT:                                                              // If state = PREHEAT
      if (abs(millis() - lastTimer) > preHeatTempTime)                                      // If time segment period expired  
      { 
        if (input >= tempSoakMin)                                                           // If minimum soak temperature is achieved
        {
          reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);                  // Set PID paramaters to Soak profile
          reflowState = REFLOW_STATE_SOAK;                                                  // Proceed to soaking state
        } else 
          {
            reflowStatus = REFLOW_STATUS_ON;                                                // Set Reflow status to ON
            setpoint = (setpoint > tempSoakMin) ? setpoint : setpoint + preHeatTempStep;    // Add next temperature step to setpoint, cap at first value over tempSoakMin
          } 
        lastTimer = millis();
      }
      break;

    case REFLOW_STATE_SOAK:                                                                 // If state = SOAK
      if (abs(millis() - lastTimer) > soakTempTime)                                         // If time segment period expired  
      { 
        if (input >= tempSoakMax)                                                           // If maximum soak temperature is achieved        
        {
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);            // Set PID paramaters to Reflow profile
          reflowState = REFLOW_STATE_HEAT;                                                  // Proceed to reflowing state
        } else
          {
            setpoint = (setpoint > tempSoakMax) ? setpoint : setpoint + soakTempStep;       // Add next temperature step to setpoint, cap at first value over tempSoakMax
          }
        lastTimer = millis();
      }
      break; 
      
    case REFLOW_STATE_HEAT:                                                                 // If state = REFLOW HEAT
      if (abs(millis() - lastTimer) > reflowHeatTempTime)                                   // If time segment period expired
      { 
        if (input >= tempReflowMax)                                                         // If maximum reflow temperature is achieved
        {
          setpoint = tempReflowMax;                                                         // Set setpoint to Reflow Max
          reflowState = REFLOW_STATE_LEVEL;                                                 // Proceed to reflowing level state
        } else
          {
            setpoint = (setpoint >= tempReflowMax) ? setpoint : setpoint + reflowHeatTempStep;  // Add next temperature step to setpoint, cap at first value over tempReflowMax
          }
        lastTimer = millis();
      }
      break; 
      
    case REFLOW_STATE_LEVEL:                                                                // If state = REFLOW LEVEL
      if (abs(millis() - lastTimer) > reflowLevelTime)                                      // Stay at this temperature for reflowLevelTime
      { 
        setpoint = tempReflowMax - reflowCoolTempStep;                                      // Lower setpoint by reflowCoolTempStep
        reflowState = REFLOW_STATE_COOL;                                                    // Proceed to reflowing cool state
        lastTimer = millis();
      }
      break; 
      
    case REFLOW_STATE_COOL:                                                                 // If state = REFLOW COOL
      if (abs(millis() - lastTimer) > reflowCoolTempTime)                                   // If time segment period expired
      { 
        if (input <= tempReflowMin)                                                         // If minimummum reflow temperature is achieved
        {         
          setpoint = 0;                                                                     // Set setpoint to 0
          reflowState = REFLOW_STATE_COOL_DOWN;                                             // Proceed to cool down state                                       
        } else
          {
            setpoint = (setpoint < tempReflowMin) ? setpoint : setpoint - reflowCoolTempStep; // Add next temperature step to setpoint, cap at first value below tempReflowMin
          }
        lastTimer = millis();
      }
      break; 

    case REFLOW_STATE_COOL_DOWN:                                                            // If state = REFLOW COOL DOWN
      if (input < tempCoolMin && reflowStatus == REFLOW_STATUS_ON)                          // If REFLOW ON and Temp < tempCoolMin
      {
        reflowStatus = REFLOW_STATUS_OFF;                                                   // Turn REFLOW OFF
        buzzerBit = true;                                                                   // Set Buzzer on flag
        digitalWrite(buzzerPin, HIGH);                                                      // Turn on buzzer
        lastTimer = millis();
      }
      
      if (buzzerBit && abs(millis() - lastTimer) > ONE_SECOND)                              // If buzzer has been on for 1000mS
        {
          digitalWrite(buzzerPin, LOW);                                                     // Turn off buzzer
          buzzerBit = false;                                                                // Clear buzzer flag
      }          
         
      if (input <= TEMPERATURE_ROOM)                                                        // If temperature lower than TEMPERATURE_ROOM
      {                                                             
        reflowState = REFLOW_STATE_COMPLETE;                                                // Proceed to complete state             
        lastTimer = millis();
      }  
      break;    

    case REFLOW_STATE_COMPLETE:                                                             // If state = COMPLETE
      if (abs(millis() - lastTimer) > 3000)                                                 // Stay in this state for 3 seconds
      {                                                             
	reflowState = REFLOW_STATE_IDLE;                                                    // Proceed to IDLE state
      }
      break;
	
    case REFLOW_STATE_TOO_HOT:                                                              // If state = TOO HOT
      if (input < TEMPERATURE_ROOM)                                                         // If oven temperature drops below room temperature
      {
         reflowState = REFLOW_STATE_IDLE;                                                    // Proceed to IDEL State
      }
      break;
		
    case REFLOW_STATE_ERROR:
      if (abs(millis() - lastTimer) > ONE_SECOND) 
      {
        if(thermocoupleFault(thermocouple.readThermocouple(CELSIUS)))                         // If thermocouple problem is still present
        {
          reflowState = REFLOW_STATE_ERROR;                                                   // Wait until fault is cleared
          reflowStatus = REFLOW_STATUS_OFF;                                                   // Turn reflow off
        } else
          {
            reflowState = REFLOW_STATE_IDLE;                                                  // Clear to perform reflow process
          }
        lastTimer = millis();
      }
      break;    
  }    

  if (switchStatus == SWITCH_1)                                                             // If switch 1 is pressed
  {
    if (reflowStatus == REFLOW_STATUS_ON)                                                   // If currently reflow process is on going 
    {
      reflowStatus = REFLOW_STATUS_OFF;                                                     // Turn off reflow process
      reflowState = REFLOW_STATE_IDLE;                                                      // Reinitialize state machine
    }
  } 

  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:                                                               // No valid switch press
      switchStatus = SWITCH_NONE;
      switchValue = readSwitch(analogRead(switchPin));
      if (switchValue != SWITCH_NONE)                                                       // If either switch is pressed
      {        
        switchMask = switchValue;                                                           // Keep track of the pressed switch
        lastDebounceTime = millis();                                                        // Intialize debounce counter
        debounceState = DEBOUNCE_STATE_CHECK;                                               // Proceed to check validity of button press
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch(analogRead(switchPin));
      if (switchValue == switchMask)
      {
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)                            // If minimum debounce period is completed
        {
          switchStatus = switchMask;                                                        // Valid switch press
          debounceState = DEBOUNCE_STATE_RELEASE;                                           // Proceed to wait for button release
        }
      } else
        {
          debounceState = DEBOUNCE_STATE_IDLE;                                              // Reinitialize button debounce state machine
        }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch(analogRead(switchPin));
      if (switchValue == SWITCH_NONE)
      {        
        debounceState = DEBOUNCE_STATE_IDLE;                                                // Reinitialize button debounce state machine
      }
      break;
  }

  if (reflowStatus == REFLOW_STATUS_ON)                                                     // PID computation and SSR control
  {
    if(abs(millis() - lastUpdate) > PID_SAMPLE_TIME)                                        // Update PID at PID_SAMPLE_TIME
    {
      reflowOvenPID.Compute();                                                              // Update PID
      analogWrite(ssrPin, output);                                                          // Adjust PWM duty cycle according to PID output (0 to PWM_MAX)
      lastUpdate = millis(); 
    }
  } else 
    {
      analogWrite(ssrPin, 0);                                                               // Reflow oven process is off, ensure oven is off
    }
}

void loadProfile(reflowProfile_t profile)
{
  if (profile == REFLOW_PROFILE_LEADFREE)
  {
    tempSoakMin = TEMPERATURE_SOAK_MIN_LF;
    tempSoakMax = TEMPERATURE_SOAK_MAX_LF;
    tempReflowMax = TEMPERATURE_REFLOW_MAX_LF;
    tempReflowMin = TEMPERATURE_REFLOW_MIN_LF;
    tempCoolMin = TEMPERATURE_COOL_MIN_LF;
    preHeatTempTime = int(PREHEAT_DURATION_LF / SEGMENTS);
    preHeatTempStep = (TEMPERATURE_SOAK_MIN_LF - TEMPERATURE_ROOM) / SEGMENTS;
    soakTempTime = int(SOAK_DURATION_LF / SEGMENTS);
    soakTempStep = (TEMPERATURE_SOAK_MAX_LF - TEMPERATURE_SOAK_MIN_LF) / SEGMENTS;
    reflowHeatTempTime = int(REFLOW_HEAT_DURATION_LF / SEGMENTS);
    reflowHeatTempStep = (TEMPERATURE_REFLOW_MAX_LF - TEMPERATURE_SOAK_MAX_LF) / SEGMENTS;
    reflowLevelTime = REFLOW_LEVEL_DURATION_LF;
    reflowCoolTempTime = int(REFLOW_HEAT_DURATION_LF / SEGMENTS);
    reflowCoolTempStep = (TEMPERATURE_REFLOW_MAX_LF - TEMPERATURE_REFLOW_MIN_LF) / SEGMENTS;
  } else 
    {
      tempSoakMin = TEMPERATURE_SOAK_MIN_PB;
      tempSoakMax = TEMPERATURE_SOAK_MAX_PB;
      tempReflowMax = TEMPERATURE_REFLOW_MAX_PB;
      tempReflowMin = TEMPERATURE_REFLOW_MIN_PB;
      tempCoolMin = TEMPERATURE_COOL_MIN_PB;
      preHeatTempTime = int(PREHEAT_DURATION_PB / SEGMENTS);
      preHeatTempStep = (TEMPERATURE_SOAK_MIN_PB - TEMPERATURE_ROOM) / SEGMENTS;
      soakTempTime = int(SOAK_DURATION_PB / SEGMENTS);
      soakTempStep = (TEMPERATURE_SOAK_MAX_PB - TEMPERATURE_SOAK_MIN_PB) / SEGMENTS;
      reflowHeatTempTime = int(REFLOW_HEAT_DURATION_PB / SEGMENTS);
      reflowHeatTempStep = (TEMPERATURE_REFLOW_MAX_PB - TEMPERATURE_SOAK_MAX_PB) / SEGMENTS;
      reflowLevelTime = REFLOW_LEVEL_DURATION_PB;
      reflowCoolTempTime = int(REFLOW_HEAT_DURATION_PB / SEGMENTS);
      reflowCoolTempStep = (TEMPERATURE_REFLOW_MAX_PB - TEMPERATURE_REFLOW_MIN_PB) / SEGMENTS;
    } 
}

bool thermocoupleFault(double reading)
{
  return (reading == FAULT_OPEN || reading == FAULT_SHORT_GND || reading == FAULT_SHORT_VCC) ? true : false;
}

switch_t readSwitch(int switchVal)
{
  if (switchVal >= 1000) return SWITCH_NONE;
  if (switchVal <= 10) return SWITCH_1;
  if (switchVal <= 522) return SWITCH_2;
  return SWITCH_NONE;
}


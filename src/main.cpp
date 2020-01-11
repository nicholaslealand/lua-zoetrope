#include "Arduino.h"

/** This demo should demonstrate the majority of required parts for your project
 * It includes:
 *  * Bluetooth serial
 *  * Bluetooth serial input
 *  * Flashing LED
 *  * Can set frequncy and ducty cycle from bluetooth serial
 * TODO (For you):
 *  1. Persisting settings to permenant memory when updated
 *  2. Inclusion of RPM sensing code
 * 
 * Some interesting links:
 * MMA: https://www.baldengineer.com/measure-pwm-current.html
 * 
 * Frequency from interrupt
 * https://esp32.com/viewtopic.php?t=6533
 * 
 * TODO: Allow choice between fixed/set PWM freq and measured
 * 
 **/
#include "BluetoothSerial.h"

// Length of frequency changes 
#define COOL_PERIOD_SECONDS           400

// Argument type defines
#define ARGUMENT_TYPE_NONE            0
#define ARGUMENT_TYPE_LONG            1
#define ARGUMENT_TYPE_DOUBLE          2
#define ARGUMENT_TYPE_STRING          3

// Motor to Zeo rotation conversion factor
#define MOTOR_ZEO_GEARING_FACTOR      4.19
// to move to slower sensor I have multiplied this factor by 14.099
// average motor sensor is 17.019 ms avare head sensor is 239.95ms
// #define MOTOR_ZEO_GEARING_FACTOR      0.29 this is pretty good for the orange belt
// #define MOTOR_ZEO_GEARING_FACTOR      0.275 this is using the elastic band

// The 'pin' the LED is on - In the case of NodeMCU pin2 is the onboard led
#define LED_ONBOARD_PIN               2

// PWM resolution in bits
#define LED_PWM_RESOLUTION            8
// PWM inital duty
#define LED_PWM_INITIAL_DUTY           5

// We set up for three LEDs to be controlled...
// DEVKIT V1 uses pin 23
// #define LED1_PIN               23
#define LED1_PIN                       12
// The PWM channel for the LED 0 to 15
#define LED1_PWM_CHANNEL               0

#define LED2_PIN                       13
// The PWM channel for the LED 0 to 15
#define LED2_PWM_CHANNEL               1

#define LED3_PIN                       14
// The PWM channel for the LED 0 to 15
#define LED3_PWM_CHANNEL               2


// Frequency measure
#define FREQ_MEASURE_PIN              19
#define FREQ_MEASURE_TIMER            1
#define FREQ_MEASURE_TIMER_PRESCALAR  80
#define FREQ_MEASURE_TIMER_COUNT_UP   true
#define FREQ_MEASURE_TIMER_PERIOD     FREQ_MEASURE_TIMER_PRESCALAR/F_CPU
#define FREQ_MEASURE_SAMPLE_NUM       128
// this returns a pointer to the hw_timer_t global variable
// 0 = first timer
// 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
// true - counts up

//Timers and counters and things
/** Timer and process control **/
uint32_t timestamp = 0;
int timestampQuarter = 0;
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  timestampQuarter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

/** Timer for measuring freq **/
volatile uint64_t StartValue;                     // First interrupt value
bool fAdded = false;
// Our own Ring Buffer
uint8_t ringIndex = 0;
uint64_t myRing[FREQ_MEASURE_SAMPLE_NUM] = {0};
// average freq intermediate values as globals. Bite me!
uint64_t sumPeriod;
float avgPeriod;
// prev freq for freq compare
long prevFreq = 0;

hw_timer_t * fTimer = NULL;                       // pointer to a variable of type hw_timer_t 
portMUX_TYPE fTimerMux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?

// Store the delta changes
float freqDeltaLut[COOL_PERIOD_SECONDS];

// Digital Event Interrupt
// Enters on falling edge in this example
//=======================================
void IRAM_ATTR handleFrequencyMeasureInterrupt()
{
  portENTER_CRITICAL_ISR(&fTimerMux);
      // uint8_t blah = 8;
      // value of timer at interrupt
      uint64_t TempVal= timerRead(fTimer);
      if (ringIndex == FREQ_MEASURE_SAMPLE_NUM -1 ) {
        ringIndex = 0;
      } else {
        ringIndex++;
      }
      // // Add period to RunningAverage, period is in number of FREQ_MEASURE_TIMER_PERIOD
      // // Note: Is timer overflow safe
      // This one MOFO's
      myRing[ringIndex]= TempVal - StartValue;
      // frequencyRA.addValue(TempVal - StartValue);
      // // puts latest reading as start for next calculation
      StartValue = TempVal;
      fAdded = true;
  portEXIT_CRITICAL_ISR(&fTimerMux);
}

// Create a BluetoothSerial thingee
BluetoothSerial SerialBT;

// Some variables to use in our program - global scope
String serialBuffer = "";
String messages;

// A 'struct' is an object containing other variables
// This defines the struct data type
struct ProgramVars {
  long    pwmFreq;
  long    setFreq;
  bool    useSetFreq;
  long    pwmDutyThou;
  double  freqDelta;
  bool    runVariableDelta;
  double  freqConversionFactor;
  bool    ledEnable;
  bool    logging;
  bool    stateChange;
  String  randomString;
};
// This creates a new variable which is of the above struct type
ProgramVars programVars = {
  0,      // pwmFreq
  0,      // setFreq
  false,  // useSetFreq
  LED_PWM_INITIAL_DUTY,      // pwmDutyThou
  1.0,    // freqDelta
  true,    // runVariableDelta
  MOTOR_ZEO_GEARING_FACTOR, // freqConversionFactor
  true,   // ledEnable
  false,  //  logging
  false,  // stateChange
  ""      //randomString
};


/** This function takes a string and separates out the command and argument
 * The command is the first character, the argument is the remainder
 * 
 * In C/C++ rather than return the parts, we act on pointers to the variables
 * Pointers to variables (the memory location of a variable) are denoted by a '*'
 * 
 * This function returns EXIT_FAILURE (could be 0) if it fails,
 * or EXIT_SUCCESS (could be 1) if everything is OK
 **/
 int getCommandAndArgument(String inputString, char *command, String *argument) {
   // Check that the String is long enough to process and return fail otherwise
   if (inputString.length() <= 1) {
     return EXIT_FAILURE;
   }

   *command = inputString.charAt(0);
   *argument = inputString.substring(1);
   return EXIT_SUCCESS;
 }

 /** stringToLong function that returns state
  * Why is state important?
  * The standard string to integer functions the integer representation
  * of a string if one exists and 0 if it does not (i.e atoi('a') = 0)
  * This is **shit** because "0" is a valid and common number
  * If the string cannot be converted then we should know about it.
  * 
  * To do this we use a function that returns a state and modifes a
  * pointer to an int if the conversion is successful
  * 
  **/
  int stringToLong(String inputString, long *targetInt) {
    // convert the input string to an integer
    int32_t intTemp = inputString.toInt();
    // If the resulting integer is not 0 then no problem
    if (intTemp != 0) {
      *targetInt = intTemp;
      return EXIT_SUCCESS;
    // If the input string is literally "0" no problem
    } else if (inputString == "0") {
      *targetInt = 0;
      return EXIT_SUCCESS;
    // Otherwise there was a problem
    } else {
      return EXIT_FAILURE;
    }    
  }

  // A 'struct' to hold parsed commands and arguments
  struct CommandAndArguments {
    char    command;
    int     argType;
    long    argLong;
    String  argString;
    boolean parseState;
  };

  /** Parse the command/args string
   * Find the command if present, and parse the arguments
   * determining where there are none, are a number, or a string
   **/
  CommandAndArguments parseCommandArgs(String commandArgs) {
    char comChar = 'h';
    int argType = ARGUMENT_TYPE_NONE;
    long argLong = 0;
    String argString = "";


    // Trim the result, include removing the trailing '/n'
    commandArgs.trim();

    // Check that the String is long enough to process and return fail otherwise
    if (commandArgs.length() == 0) {
      return CommandAndArguments{
        comChar, argType, argLong, argString, EXIT_FAILURE
      };
    } 
    // Get the command
    comChar = commandArgs.charAt(0);

    // If there are enough characters in 'commandArgs' get and parse them
    if (commandArgs.length() > 1) {
      // Separate the argument from the command
      argString = commandArgs.substring(1);
      // If we can convert the argString to a number we do
      if (stringToLong(argString, &argLong) == EXIT_SUCCESS) {
        argType = ARGUMENT_TYPE_LONG;
      } else {
        argType = ARGUMENT_TYPE_STRING;
      }
    }

    // Return all the things
    return CommandAndArguments{
      comChar, argType, argLong, argString, EXIT_SUCCESS
    };
  }

  /** The standard OP for getting/setting/displaying command and args
   * There are two semi identical forms of this function, one where the 
   * argument is a number (long), and one where it is a string
   **/
  boolean argDisplayOrSetLong(String argName, CommandAndArguments comAndArg, long *var, String *message) {
    if (comAndArg.argType == ARGUMENT_TYPE_NONE) {
      *message = argName + " is : " + String(*var);
      return false;
    }
    if (comAndArg.argType == ARGUMENT_TYPE_LONG) {
      *var = comAndArg.argLong;
      *message = "Set '" + argName + "' to : " + String(*var);
      return true;
    }
    return false;
  }

  /** The standard OP for getting/setting/displaying command and args
   * There are two semi identical forms of this function, one where the 
   * argument is a number (long), and one where it is a string
   **/
  boolean argDisplayOrSetDoubleFromLong(String argName, CommandAndArguments comAndArg, double *var, uint16_t denominator, String *message) {
    if (comAndArg.argType == ARGUMENT_TYPE_NONE) {
      *message = argName + " is : " + String(*var);
      return false;
    }
    if (comAndArg.argType == ARGUMENT_TYPE_LONG) {
      *var = 1.0 * comAndArg.argLong / denominator;
      *message = "Set '" + argName + "' to : " + String(*var);
      return true;
    }
    return false;
  }

  // String version
  boolean argDisplayOrSetString(String argName, CommandAndArguments comAndArg, String *var, String *message) {
    if (comAndArg.argType == ARGUMENT_TYPE_NONE) {
      *message = argName + " is : '" + *var + "'";
      return false;
    }
    if (comAndArg.argType == ARGUMENT_TYPE_STRING) {
      *var = comAndArg.argString;
      *message = "Set '" + argName + "' to : '" + *var + "'";
      return true;
    }
    return false;
  }
  // Boolean version
  boolean argDisplayOrSetBoolean(String argName, CommandAndArguments comAndArg, boolean *var, String *message) {
    if (comAndArg.argType == ARGUMENT_TYPE_NONE) {
      *message = argName + " is : '" + String(*var) + "'";
      return false;
    }
    // Check if true both string and Long
    comAndArg.argString.toLowerCase();
    if (
        // String and equals 'true'
        (comAndArg.argType == ARGUMENT_TYPE_STRING && comAndArg.argString == "true") ||
        // Long and equals 1
        (comAndArg.argType == ARGUMENT_TYPE_LONG && comAndArg.argLong == 1)
      ) {
        *var = true;
        *message = "Set '" + argName + "' to : 'true'";
        return true;
    }
    // Check if false both string and Long
    if (
        // String and equals 'true'
        (comAndArg.argType == ARGUMENT_TYPE_STRING && comAndArg.argString == "false") ||
        // Long and equals 1
        (comAndArg.argType == ARGUMENT_TYPE_LONG && comAndArg.argLong == 0)
      ) {
        *var = false;
        *message = "Set '" + argName + "' to : 'false'";
        return true;
    }
    return false;
  }

 /** *do things* based on inputString:
  *   * Update progVars
  *   * Show progVars
  * This function should not change output state, but if vars change
  * the 'stateChange' flag shoudl be set
  *  
  **/
  int processCommands(String inputString, ProgramVars *progVars, String *message) {
    // Parse the 'inputString'
    CommandAndArguments comArgState = parseCommandArgs(inputString);

    // Exit with message if no command
    if (comArgState.parseState == EXIT_FAILURE) {
      *message = "Input string is not a valid command/argument";
      return EXIT_FAILURE;
    }

    // Let us process the commands
    switch (comArgState.command)
    {
    case 'h':
      progVars->stateChange = false;
      *message = String("Help: \n") + 
        String("Commands will return current value if no argument given, and set to value if given\n") +
        String("'f': PWM frequency in HZ\n") +
        String("'p': Whether to measure frequency or use frequency set by 'p'\n") +
        String("'d': PWM duty cycle 0-reolution max (ie 255 for 8 bit)\n") +
        String("'m': Frequency modifier to apply to measured frequency as percentage\n") +
        String("'v': Run variable delta programme Enable (1), or disable (0)\n") +
        String("'r': Rotational gearing ratio * 1000 \n") +
        String("'l': Enable (1), or disable (0) led\n") +
        String("'L': Enable (1), or disable (0) logging");
      break;
    case 'f':
      progVars->stateChange = argDisplayOrSetLong("useSetFreq", comArgState, &progVars->setFreq, message);
      break;
    case 'p':
      progVars->stateChange = argDisplayOrSetBoolean("useSetFreq", comArgState, &progVars->useSetFreq, message);
      break;
    case 'd':
      progVars->stateChange = argDisplayOrSetLong("pwmDuty", comArgState, &progVars->pwmDutyThou, message);
      break;
    case 'm':
      progVars->stateChange = argDisplayOrSetDoubleFromLong("freqDelta", comArgState, &progVars->freqDelta, 100, message);
      break;
    case 'v':
      progVars->stateChange = argDisplayOrSetBoolean("runVariableDelta", comArgState, &progVars->runVariableDelta, message);
      break;
    case 'r':
      progVars->stateChange = argDisplayOrSetDoubleFromLong("freqConversionFactor", comArgState, &progVars->freqConversionFactor, 1000, message);
      break;
    case 's':
      progVars->stateChange = argDisplayOrSetString("randomString", comArgState, &progVars->randomString, message);
      break;
    case 'l':
      progVars->stateChange = argDisplayOrSetBoolean("ledEnable", comArgState, &progVars->ledEnable, message);
      break;
    case 'L':
      progVars->stateChange = argDisplayOrSetBoolean("logging", comArgState, &progVars->logging, message);
      break;
    default:
      progVars->stateChange = false;
      *message = "No recognised command";
      break;
    }
    return EXIT_SUCCESS;
  }

String formatProgVars(long time, ProgramVars progVars) {
  return String(time) + " ledEnable: " + String(progVars.ledEnable) +
    " setFreq: " + String(progVars.setFreq) +
    " pwmFreq: " + String(progVars.pwmFreq) +
    " pwmDuty: " + String(progVars.pwmDutyThou) +
    " freqDelta: " + String(progVars.freqDelta) +
    " pwmDuty: " + String(progVars.pwmDutyThou) +
    " Random string: '" + progVars.randomString +"'";
}

double calculateFinalFrequency(float avgPeriod, double conversionFactor) {
  double frequencyAtMotor = 1 / (avgPeriod * FREQ_MEASURE_TIMER_PERIOD);
  // Apply the conversion factor
  return frequencyAtMotor * conversionFactor;
}

struct KeyPoint {
  uint16_t t;
  float freqDelta;
};

void buildLookupTable(KeyPoint *keypoints, uint16_t num_keypoints, float* lookup_table, uint16_t lookup_table_size) {
  float default_freqDelta = 1.0;
  // Without any keypoints, just set all entries zero
  if (num_keypoints == 0) {
    for (int i=0; i < lookup_table_size; i++) {
      lookup_table[i] = default_freqDelta;
    }
  }
  KeyPoint last_point;
  int start_idx;
  if (keypoints[0].t != 0) {
    last_point = KeyPoint { 0, default_freqDelta };
    start_idx = 0;
  } else {
    last_point = keypoints[0];
    start_idx = 1;
  }
  
  int lut_idx = 0;
  for (int i=start_idx; i < num_keypoints; i++) {
    while (lut_idx <= keypoints[i].t && lut_idx < lookup_table_size) {
      // Just using linear interpolation for now
      // TODO maybe smooth keypoints with splines if you want to get fancy...
      lookup_table[lut_idx] = last_point.freqDelta + ((keypoints[i].freqDelta - last_point.freqDelta) / (keypoints[i].t - last_point.t));
      lut_idx++;
    } 
  }
  while (lut_idx < lookup_table_size) {
    lookup_table[lut_idx + 1] = lookup_table[lut_idx];
    lut_idx++;
  }
}

/** This function modifies the delta/modifier based on the timestamp in seconds
 * on a cycle (hence the modulo)
 **/
void makeShitCoolAgain(uint32_t timestamp, ProgramVars *programVars) {
  uint32_t relativeTime = timestamp % COOL_PERIOD_SECONDS;
  programVars->freqDelta = freqDeltaLut[relativeTime];
}


void setup() {
  // Give a semaphore that we can check in the loop
  // Setup Timer 1 on interrupt
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every quarter second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000/4, true);
  // Start an alarm
  timerAlarmEnable(timer);

  // Setup PWM channels
  double startFreq = 500.0;
  // configure LED PWM functionalitites
  ledcSetup(LED1_PWM_CHANNEL, startFreq, LED_PWM_RESOLUTION);
  ledcSetup(LED2_PWM_CHANNEL, startFreq, LED_PWM_RESOLUTION);
  ledcSetup(LED3_PWM_CHANNEL, startFreq, LED_PWM_RESOLUTION);
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(LED1_PIN, LED1_PWM_CHANNEL);
  ledcAttachPin(LED2_PIN, LED2_PWM_CHANNEL);
  ledcAttachPin(LED3_PIN, LED3_PWM_CHANNEL);
  // Set initial duty now, to avoid things being too bright at startup...
  ledcWrite(LED1_PWM_CHANNEL, programVars.pwmDutyThou);
  ledcWrite(LED2_PWM_CHANNEL, programVars.pwmDutyThou);
  ledcWrite(LED3_PWM_CHANNEL, programVars.pwmDutyThou);

  // Make onboard LED mimic LED1
  ledcAttachPin(LED_ONBOARD_PIN, LED1_PWM_CHANNEL);

  // Initialise the serial hardware at baude 115200
  Serial.begin(115200);
 
  // Initialise the Bluetooth hardware with a name 'ESP32'
  if(!SerialBT.begin("ESP32")){
    Serial.println("An error occurred initializing Bluetooth");
  }

  // Build lookup tables for freq modulation
  // This replaces the giant switch/case statement and adds linear interpolation between values...
  KeyPoint keypoints[] = {
    KeyPoint { 0, 1.0 },
    KeyPoint { 101, 1.0 },
    KeyPoint { 102, 1.1 },
    KeyPoint { 111, 2.0 },
    KeyPoint { 129, 2.0 },
    KeyPoint { 139, 3.0 },
    KeyPoint { 150, 3.0 },
    KeyPoint { 161, 4.0 },
    KeyPoint { 180, 4.0 },
    KeyPoint { 200, 1.1 },
    KeyPoint { 210, 1.0 },
    KeyPoint { 200, 1.1 },
    KeyPoint { 214, 1.1 },
    KeyPoint { 215, 1.95 },
    KeyPoint { 220, 1.9 },
    KeyPoint { 230, 1.8 },
    KeyPoint { 250, 1.4 },
    KeyPoint { 260, 1.3 },
    KeyPoint { 280, 1.1 },
    KeyPoint { 290, 1.05 },
    KeyPoint { 300, 1.01 },
    KeyPoint { 330, 1 },
    KeyPoint { 340, 5 },
    KeyPoint { 350, 1 },
    KeyPoint { 360, 5 },
    KeyPoint { 370, 1 },
  };
  buildLookupTable(keypoints, sizeof(keypoints)/sizeof(KeyPoint), freqDeltaLut, sizeof(freqDeltaLut)/sizeof(float));

  // Setup frequency measure timer
  // sets pin high
  pinMode(FREQ_MEASURE_PIN, INPUT);
  // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(FREQ_MEASURE_PIN), handleFrequencyMeasureInterrupt, FALLING);
  // Setup the timer
  fTimer = timerBegin(FREQ_MEASURE_TIMER, FREQ_MEASURE_TIMER_PRESCALAR, FREQ_MEASURE_TIMER_COUNT_UP);
  // Start the timer
  timerStart(fTimer);
}

inline void updateLED(uint32_t pwm_channel, ProgramVars& programVars) {
  if (programVars.runVariableDelta == true) {
    makeShitCoolAgain(timestamp, &programVars);
  }

  // Change the PWM freq if it has changed
  if ( programVars.pwmFreq != prevFreq) {
    ledcWriteTone(pwm_channel, programVars.pwmFreq);
    prevFreq = programVars.pwmFreq;
    if (programVars.ledEnable == true) {
      ledcWrite(pwm_channel, programVars.pwmDutyThou);
    } else {
      ledcWrite(pwm_channel, 0);
    }
  }
}
 
void loop() {
 
  // If Timer has fired do some non-realtime stuff
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // If the buffers end in newline, try to parse the command an arguments
    if(serialBuffer.endsWith("\n")) {
      // Print out the buffer - for fun
      Serial.println(serialBuffer);
      // Process the commands
      processCommands(serialBuffer, &programVars, &messages);
      // Print the message
      Serial.println(messages);
      SerialBT.println(messages);
      // Reset the buffer to empty
      serialBuffer = "";
    }

    if (programVars.stateChange == true || fAdded == true) {
      // reset c flhangeag
      fAdded = false;
      programVars.stateChange = false;


      if (programVars.useSetFreq) {
        programVars.pwmFreq = programVars.setFreq;
      } else {
        // calculate the frequency from the average period
        sumPeriod = 0;
        for (int i = 0; i < FREQ_MEASURE_SAMPLE_NUM; ++i)
        {
            sumPeriod += myRing[i];
        }
        avgPeriod = ((float)sumPeriod)/FREQ_MEASURE_SAMPLE_NUM; //or cast sum to double before division
        programVars.pwmFreq = calculateFinalFrequency(avgPeriod, programVars.freqConversionFactor) * programVars.freqDelta;
      }

      messages = "Setting PWM duty to: " + String(programVars.pwmDutyThou) + \
        " Frequency to: " + String(programVars.pwmFreq) + \
        " User set freq to: " + String(programVars.setFreq);

      Serial.println(messages);
      SerialBT.println(messages);
      // We need to change duty to 0 if LED is disabled
      if (programVars.ledEnable == true) {
        ledcWrite(LED1_PWM_CHANNEL, programVars.pwmDutyThou);
        ledcWrite(LED2_PWM_CHANNEL, programVars.pwmDutyThou);
        ledcWrite(LED3_PWM_CHANNEL, programVars.pwmDutyThou);
      } else {
        messages = "Disabling LED";
        ledcWrite(LED1_PWM_CHANNEL, 0);
        ledcWrite(LED2_PWM_CHANNEL, 0);
        ledcWrite(LED3_PWM_CHANNEL, 0);
      }
    }

    // Timer fires every quarter second, so every four tickes
    // we increment the timestamp and log
    if (timestampQuarter%4 == 0) {
      timestamp++;
      timestampQuarter = 0;

      updateLED(LED1_PWM_CHANNEL, programVars);
      updateLED(LED2_PWM_CHANNEL, programVars);
      updateLED(LED3_PWM_CHANNEL, programVars);

      // print logging info if enabled
      if (programVars.logging == true) {
        String logMessage = formatProgVars(timestamp, programVars);    
        Serial.println(logMessage);
        SerialBT.println(logMessage);
      }
    }
  }

  // Do realtime things
  // Calculate the frequency
  // programVars.pwmFreq = programVars.useSetFreq;
  // if (programVars.useSetFreq) {
  //   programVars.pwmFreq = programVars.useSetFreq;
  // } else if (fAdded == true) {
  //   programVars.pwmFreq = calculateFinalFrequency(frequencyRA, programVars.freqConversionFactor) + programVars.freqDelta;
  // }


  // While there are characters in the Serial buffer
  // read them in one at a time into sBuffer
  // We need to go via char probably due to implicit type conversions
  while(Serial.available() > 0){
    char inChar = Serial.read();
    serialBuffer += inChar;
  }
 
  // As above but with the bluetooth device
  while(SerialBT.available() > 0){
    char inChar = SerialBT.read();
    serialBuffer += inChar;
  }


  // Wait 50 ms
  // ooooo, gross! Don't do that. naa fk ya
  // delay(50);
}

// #include "TimerOne.h"

// const int numreadings = 5;
// volatile unsigned long readings[numreadings];
// int index = 0;
// unsigned long average = 0;
// unsigned long period1 = 0;
// unsigned long total=0;
//
// volatile unsigned block_timer = 0;
// volatile bool block=false;
//
// volatile bool calcnow = false;
//
// //Strobe settings
// const int strobePin = 9;
// float dutyCycle = 5.0;  //percent
// float dutyCycle_use = (dutyCycle / 100) * 1023;
// float strobe_period_float = 0;
// unsigned long period_microsec = 0;
// const unsigned long non_strobe_period = 1000; //stobe period when not yet in strobe mode - just a light. 1000us == 1000hz
//
// float stobes_per_rot = 11.;
//
// void rps_counter(); // forward declare
//
//
// void rps_counter(){ /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
//   //this is really a bit much code for an ISR, but we only have ~3 rps so it should be ok
//
//   if(block==false){
//     readings[index] = micros();
//     index++;
//
//     //Double triggering block
//     block_timer = micros();
//     block=true;
//
//     if(index >= numreadings){
//      index=0;
//      calcnow=true;
//     }
//   } //if "blocked", do nothing
// }
//
// void setup() {
//    Serial.begin(115200);
//    //initial light mode
//    Timer1.initialize(non_strobe_period);  // 10 000 000 us = 10 Hz
//    Timer1.pwm(strobePin, dutyCycle_use);
//
//    attachInterrupt(0, rps_counter, FALLING); //interrupt 0 is pin 2
// }
//
//
// void loop(){
//
//   //Check hall effect block timer
//   if(block){
//     // detachInterrupt(0);    //Disable interrupt for ignoring multitriggers
//     if(micros()-block_timer >= 100000){
//       block=false;
//       // attachInterrupt(0, rps_counter, FALLING); //enable interrupt
//     }
//   }
//
//   if(calcnow){
//     // detachInterrupt(0);    //Disable interrupt when printing
//     total = 0;
//     for (int x=numreadings-1; x>=1; x--){ //count DOWN though the array, every 70 min this might make one glitch as the timer rolls over
//       period1 = readings[x]-readings[x-1];
//       total = total + period1;
//     }
//     average = total / (numreadings-1); //average period
//
//     Serial.println(average);
//
//     if(average<10000000){ //ie, more than 0.1hz of rotation.
//       strobe_period_float = average / stobes_per_rot ;
//       period_microsec = (unsigned int) strobe_period_float;
//       Timer1.setPeriod(period_microsec);
//       Timer1.pwm(strobePin, dutyCycle_use);
//     }
//     else{
//       Serial.println("ack too slow, going to constant light");
//       Timer1.setPeriod(non_strobe_period);
//       Timer1.pwm(strobePin, dutyCycle_use);
//     }
//     calcnow = false;
//     // attachInterrupt(0, rps_counter, FALLING); //enable interrupt
//   }
// }


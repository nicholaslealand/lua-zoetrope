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
#define NUM_LEDS 4
uint16_t led_pins[NUM_LEDS] = {13, 14, 15, 16};
uint8_t led_channels[NUM_LEDS] = {0, 1, 2, 3};
// prev freq for freq compare, 
long prevFreqs[NUM_LEDS] = {0, 0, 0, 0};

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
// Ring Buffer
uint8_t ringIndex = 0;
uint64_t myRing[FREQ_MEASURE_SAMPLE_NUM] = {0};

float final_freq;

// pointer to a variable of type hw_timer_t 
hw_timer_t * fTimer = NULL;
// syncs between main and interrupt
portMUX_TYPE fTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Store the delta changes
float freqDeltaLut[COOL_PERIOD_SECONDS];
// Each LED follows the same pattern, but are offset by this number of seconds
// e.g. LED3 is idx 2, 2 * 90 is 180. LED3 is 180 seconds ahead of LED1.
#define LED_TIME_OFFSET 90
// Multiply the timestamp by this number to the power of the LED idx
// e.g. LED3 is idx 2, 1.1^2 is 1.21
#define LED_TIME_MULTIPLIER 1.1

// Digital Event Interrupt
// Enters on falling edge in this example
//=======================================
void IRAM_ATTR handleFrequencyMeasureInterrupt()
{
  portENTER_CRITICAL_ISR(&fTimerMux);
      // value of timer at interrupt
      uint64_t TempVal= timerRead(fTimer);
      if (ringIndex == FREQ_MEASURE_SAMPLE_NUM -1 ) {
        ringIndex = 0;
      } else {
        ringIndex++;
      }
      myRing[ringIndex]= TempVal - StartValue;
      StartValue = TempVal;
      fAdded = true;
  portEXIT_CRITICAL_ISR(&fTimerMux);
}

// Create a BluetoothSerial thingee
BluetoothSerial SerialBT;

// Some variables to use in our program - global scope
String serialBuffer = "";
String messages;

// This will run each LED for 5 seconds, before letting things proceed normally
bool do_test = true;
uint32_t test_progress = 0;
#define LED_TEST_PERIOD 5
// Set to true to disable randomised changes to led parameters/patterns
bool do_randomisation = true;
uint32_t timestamp_of_last_change = 0;
#define RANDOM_COOLDOWN_PERIOD 60

// A 'struct' is an object containing other variables
// This defines the struct data type
struct ProgramVars {
  long    pwmFreq[NUM_LEDS];
  long    setFreq;
  bool    useSetFreq;
  long    pwmDutyThou;
  double  freqDelta[NUM_LEDS];
  int     patternOffset[NUM_LEDS];
  float   patternSpeed[NUM_LEDS];
  bool    runVariableDelta;
  double  freqConversionFactor;
  bool    ledEnable[NUM_LEDS];
  bool    logging;
  bool    stateChange;
  String  randomString;
};
// This creates a new variable which is of the above struct type
ProgramVars programVars = {
  {0, 0, 0, 0},      // pwmFreq
  0,      // setFreq
  false,  // useSetFreq
  LED_PWM_INITIAL_DUTY,      // pwmDutyThou
  {1.0, 1.0, 1.0, 1.0},    // freqDelta
  {0, 50, 100, 150}, // patternOffset
  {1.0f, 1.0f, 1.0f, 1.0f}, // patternSpeed
  true,    // runVariableDelta
  MOTOR_ZEO_GEARING_FACTOR, // freqConversionFactor
  {true, false, false, false},   // ledEnable
  false,  // logging
  false,  // stateChange
  ""      // randomString
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
      for (int i=0; i < NUM_LEDS; i++) {
        progVars->stateChange |= argDisplayOrSetDoubleFromLong("freqDelta", comArgState, &progVars->freqDelta[i], 100, message);
      }
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
      for (int i=0; i < NUM_LEDS; i++) {
        progVars->stateChange |= argDisplayOrSetBoolean("ledEnable", comArgState, &progVars->ledEnable[i], message);
      }
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


template<typename T> String arrayToString(T* array, uint32_t len) {
  String result = "[";
  for (uint32_t i=0; i < len - 1; i++) {
    result += String(array[i]) + ", ";
  }
  result += array[len - 1] + "]";
  return result;
}

String formatProgVars(long time, ProgramVars progVars) {
  return String(time) + " ledEnable: " + arrayToString(progVars.ledEnable, NUM_LEDS) +
    " setFreq: " + String(progVars.setFreq) +
    " pwmFreq: " + arrayToString(progVars.pwmFreq, NUM_LEDS) +
    " pwmDuty: " + String(progVars.pwmDutyThou) +
    " freqDelta: " + arrayToString(progVars.freqDelta, NUM_LEDS) +
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
  while (lut_idx < lookup_table_size - 1) {
    lookup_table[lut_idx + 1] = lookup_table[lut_idx];
    lut_idx++;
  }
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
  for (int i=0; i<NUM_LEDS; i++) {
    // configure LED PWM functionalitites
    ledcSetup(led_channels[i], startFreq, LED_PWM_RESOLUTION);
  
    // Attach the channel to the GPIO to be controlled
    ledcAttachPin(led_pins[i], led_channels[i]);
  
    // Set initial duty now, to avoid things being too bright at startup...
    ledcWrite(led_channels[i], programVars.pwmDutyThou);
  }
  // Make onboard LED mimic LED1
  ledcAttachPin(LED_ONBOARD_PIN, led_pins[0]);

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

inline void roll_the_dice(ProgramVars& programVars, int percentChance) {
  if (timestamp_of_last_change != 0 // if last_change is 0 we haven't rolled the dice yet
    && timestamp - timestamp_of_last_change < RANDOM_COOLDOWN_PERIOD) {
    // no change if we've already made a change in the last RANDOM_COOLDOWN_PERIOD seconds
    return;
  } else if (random(100) < percentChance) {
    // ^ after cooldown, chance of change is percentChance in % (0 - 100)

    for (int i=0; i<NUM_LEDS; i++) {
      programVars.ledEnable[i] = false;
    }
    // First select which combo of leds to have active
    // We have a number of desired combos:
    // - any single led, NUM_LED options
    // - any two leds, NUM_LED*(NUM_LED-1) options
    uint32_t single_or_dual_led = random(0, 1000);
    // threshold for enabling two leds instead of one
    uint32_t dual_threshold = 600;

    uint8_t first_led = NUM_LEDS;
    uint8_t second_led = NUM_LEDS;
    if (single_or_dual_led < dual_threshold) {
      uint16_t first_led_split = dual_threshold / NUM_LEDS;
      first_led = single_or_dual_led / first_led_split;
    } else {
      uint16_t first_led_split = (1000 - dual_threshold) / NUM_LEDS;
      uint16_t second_led_split = first_led_split / (NUM_LEDS - 1);

      first_led = (single_or_dual_led - dual_threshold) / first_led_split;
      second_led = ((single_or_dual_led - dual_threshold) % first_led_split) / second_led_split;
      if (second_led >= first_led) second_led += 1;
    }
    
    // Due to rounding of splits to integer, the selected led indexes could be >= NUM_LEDS
    first_led = first_led % NUM_LEDS;
    second_led = first_led % NUM_LEDS;

    if (first_led < NUM_LEDS) {
      programVars.ledEnable[first_led] = true;
      // 0.9 - 1.1
      programVars.patternSpeed[first_led] = 0.9f + (((float) random(0, 20)) * 0.01);
      programVars.patternOffset[first_led] = random(0, COOL_PERIOD_SECONDS);
    }
    if (second_led < NUM_LEDS) {
      programVars.ledEnable[second_led] = true;
      // These are based off of values taken for first_led.
      // speed +/- 0.05, offset +/- 5 seconds
      programVars.patternSpeed[second_led] = programVars.patternSpeed[first_led] - 0.05 + (((float) random(0, 10)) * 0.01);
      programVars.patternOffset[second_led] = (programVars.patternOffset[first_led] - 5 + random(0, 10)) % COOL_PERIOD_SECONDS;
      
    }

    timestamp_of_last_change = timestamp;
  }
}

inline void updateFreq(ProgramVars& programVars) {
  if (programVars.useSetFreq) {
    for (int i=0; i<NUM_LEDS; i++) {
        programVars.pwmFreq[i] = programVars.setFreq;
    }
  } else {
    for (int i=0; i<NUM_LEDS; i++) {
      programVars.pwmFreq[i] = final_freq * programVars.freqDelta[i];
    }
  }
}

inline void updateLED(uint32_t pwm_channel, ProgramVars& programVars, uint8_t led) {
  if (programVars.runVariableDelta == true) {
    // we need to base timestamp of off when the parameters were last set, otherwise leds with two different patternSpeed values
    // will diverge further and further the longer the code runs, even if they specify a small offset.
    uint32_t ts = timestamp - timestamp_of_last_change;
    // modifies the delta/modifier based on the timestamp in seconds on a cycle (hence the modulo)
    uint32_t relativeTime = ((uint32_t)(programVars.patternSpeed[led] * ts) + programVars.patternOffset[led]) % COOL_PERIOD_SECONDS;

    programVars.freqDelta[led] = freqDeltaLut[relativeTime];
  }
  updateFreq(programVars);

  if (programVars.ledEnable[led] == true) {
    ledcWrite(pwm_channel, programVars.pwmDutyThou);
  } else {
    ledcWrite(pwm_channel, 0);
  }
  
  // Change the PWM freq if it has changed
  if ( programVars.pwmFreq[led] != prevFreqs[led]) {
    ledcWriteTone(pwm_channel, programVars.pwmFreq[led]);
    prevFreqs[led] = programVars.pwmFreq[led];  
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

    bool do_led_update = false;
    if (programVars.stateChange == true || fAdded == true) {
      if (fAdded) {
        // calculate the frequency from the average period
        uint64_t sumPeriod = 0;
        for (int i = 0; i < FREQ_MEASURE_SAMPLE_NUM; ++i)
        {
          sumPeriod += myRing[i];
        }
        // TODO: This is will calculate an incorrect sum until we have 128 samples
        float avgPeriod = ((float)sumPeriod)/FREQ_MEASURE_SAMPLE_NUM;
        final_freq = calculateFinalFrequency(avgPeriod, programVars.freqConversionFactor);
      }
      // make sure to update leds so that they reflect the new frequency measure
      do_led_update = true;

      fAdded = false;
      programVars.stateChange = false;
    }

    // Timer fires every quarter second, so every four ticks
    // we increment the timestamp and log
    if (timestampQuarter % 4 == 0) {
      timestamp++;
      timestampQuarter = 0;

      // optionally run each led briefly at startup
      uint32_t total_test_period = LED_TEST_PERIOD * NUM_LEDS;
      if (do_test && test_progress < total_test_period) {
        uint8_t current_led = test_progress / NUM_LEDS;
        for (int i=0; i<NUM_LEDS; i++) {
          programVars.ledEnable[i] = false;
        }
        programVars.ledEnable[current_led] = true;
        programVars.patternSpeed[current_led] = 1.0f;
        // start the current test led from the start of the pattern
        programVars.patternOffset[current_led] = (2 * COOL_PERIOD_SECONDS - timestamp) % COOL_PERIOD_SECONDS;

        test_progress++;
        if (test_progress >= total_test_period) {
          do_test = false;
          test_progress = 0;
          // 100% chance of new state
          roll_the_dice(programVars, 100);
        }
      } else if (do_randomisation) {
        // select some random LED parameters, including which LEDs are
        // actually on
        roll_the_dice(programVars, 10);
      }

      do_led_update = true;
    }

    if (do_led_update) {
      for (int i=0; i<NUM_LEDS; i++) {
        updateLED(led_channels[i], programVars, 0);
      }

      // print logging info if enabled
      if (programVars.logging == true) {
        String logMessage = formatProgVars(timestamp, programVars);    
        Serial.println(logMessage);
        SerialBT.println(logMessage);
      }
    }
  }

  // While there are characters in the Serial buffer
  // read them in one at a time into serialBuffer
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

}

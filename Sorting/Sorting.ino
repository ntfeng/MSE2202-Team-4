/*
Authors: Nathan Eng, Ben Prentice, Shayla Diep, Malak Al-Hanafi 
*/

#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <MSE2202_Lib.h>

// Function declarations
void Indicator();                                                              // for mode/heartbeat on Smart LED
void ARDUINO_ISR_ATTR timerISR();

// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use


#define SERVO_ARM 41
#define S_STEP_PIN 40                     // GPIO pin for step signal to A4988
#define D_STEP_PIN 39                     // GPIO pin for direction signal to A4988
#define SERVO_BACK_DOOR 44
#define SERVO_SORT 45
#define cSDA 47                    // GPIO pin for I2C data
#define cSCL 48                    // GPIO pin for I2C clock
#define cTCSLED 14                    // GPIO pin for LED on TCS34725
#define cLEDSwitch 46
#define cServoChannel 5

const int cDisplayUpdate = 100;
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed

boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp5sec = false;
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;  
boolean timeUp500msec = false;
boolean timeUpsec = false;
boolean movementComplete = false;
boolean valuableDetected = false;
boolean stepDir = false;
boolean runState = true;
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned long timerCount5sec = 0;
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long timerCount500msec = 0;
unsigned long timerCountsec = 0;
unsigned long servoTickCount = 0;
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

boolean timeUp100msec = false;
unsigned long timerCount100msec = 0;
boolean timeUp400msec = false;
unsigned long timerCount400msec = 0;

Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

hw_timer_t * pTimer          = NULL;                   // pointer to timer used by timer interrupt

unsigned int robotModeIndex = 0;  
volatile int turnChange = 1;
volatile int numOfLoops = 0;
volatile int32_t stepCount = 0;
unsigned int maxLoops = 1;
unsigned long stepRate = 5000;                       // map to half period in microseconds
unsigned long servoIncr = 10;
unsigned long maxAngleArm = 150;
boolean greenEntered = false;
unsigned long greenTrue = 0;

Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};
unsigned int  modeIndicator[6] = {                                             // colours for different modes
   SmartLEDs.Color(255,0,0),                                                   //   red - stop
   SmartLEDs.Color(0,255,0),                                                   //   green - run
   SmartLEDs.Color(0,0,255),                                                   //   blue - empty case
   SmartLEDs.Color(255,255,0),                                                 //   yellow - empty case
   SmartLEDs.Color(0,255,255),                                                 //   cyan - empty case
   SmartLEDs.Color(255,0,255)                                                  //   magenta - empty case
}; 

volatile long targetCount = 0;  // Target encoder count for movements
volatile bool turnComplete = false;
const int cCountsRev = 1096; 


void setup() {
  // put your setup code here, to run once:

  // Set up motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder

  // Set up SmartLED
  SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                                          // clear pixel
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
  SmartLEDs.show();   

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton

  // Servos
  Bot.servoBegin("S1", SERVO_ARM); 
  Bot.servoBegin("S2", SERVO_SORT); 
  Bot.servoBegin("S3", SERVO_BACK_DOOR); 

  Bot.ToPosition("S1", degreesToDutyCycle(0));
  Bot.ToPosition("S2", degreesToDutyCycle(85));
  Bot.ToPosition("S3", degreesToDutyCycle(180));
  // pinMode(SERVO_DOOR_L, OUTPUT);                      // configure servo GPIO for output
  // ledcSetup(SERVO_DOOR_L, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution

  // pinMode(SERVO_DOOR_R, OUTPUT);                      // configure servo GPIO for output
  // ledcSetup(SERVO_DOOR_R, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution

  // pinMode(SERVO_SORT, OUTPUT);                      // configure servo GPIO for output
  // ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  // ledcAttachPin(SERVO_SORT, cServoChannel);         // assign servo pin to servo channel
  // ledcWrite(cServoChannel, degreesToDutyCycle(45));

  // pinMode(SERVO_BACK_DOOR, OUTPUT);                      // configure servo GPIO for output
  // ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  // ledcAttachPin(SERVO_BACK_DOOR, cServoChannel);         // assign servo pin to servo channel

  // pinMode(SERVO_ARM, OUTPUT);                      // configure servo GPIO for output
  // ledcSetup(SERVO_ARM, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution

  pinMode(S_STEP_PIN, OUTPUT);                           // assign output for step signal to A4988
  pinMode(D_STEP_PIN, OUTPUT);                            // assign output for direction signal to A4988
  
  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  //pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  modePBDebounce = 0;                                                         // reset debounce timer count// Set direction of stepper motor
  digitalWrite(D_STEP_PIN, stepDir);

  pTimer = timerBegin(0, 80, true);                    // start timer 0 (1 of 4) with divide by 80 prescaler for 1 MHz resolution
                                                       // (see ESP32 Technical Reference Manual for more info).
  timerAttachInterrupt(pTimer, &timerISR, true);       // configure timer ISR
  timerAlarmWrite(pTimer, 500, true);                  // set initial interrupt time (in microseconds), set to repeat
  timerAlarmEnable(pTimer);                            // enable timer interrupt
}

void loop() {

  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  // put your main code here, to run repeatedly:
  long pos[] = {0, 0};                                                        // current motor positions

  float total = red + green + blue;
  float red_ratio = red / total;
  float green_ratio = green / total;
  float blue_ratio = blue / total;

  currentMicros = micros();                                                   // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
    previousMicros = currentMicros;                                          // record current time in microseconds

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
    if (timerCount3sec > 3000) {                                             // if 3 seconds have elapsed
      timerCount3sec = 0;                                                   // reset 3 second timer count
      timeUp3sec = true;                                                    // indicate that 3 seconds have elapsed
    }
   
    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1;                                     // increment 2 second timer count
    if (timerCount2sec > 2000) {                                             // if 2 seconds have elapsed
      timerCount2sec = 0;                                                   // reset 2 second timer count
      timeUp2sec = true;                                                    // indicate that 2 seconds have elapsed
    }

    
    timerCount100msec = timerCount100msec + 1;                                     
    if (timerCount100msec > 100) {                                             
      timerCount100msec = 0;                                                   
      timeUp100msec = true;                                                    
    }

    timerCount500msec = timerCount500msec + 1;                                     
    if (timerCount500msec > 500) {                                             
      timerCount500msec = 0;                                                   
      timeUp500msec = true;                                                    
    }

    timerCountsec = timerCountsec + 1;                                     
    if (timerCountsec > 1000) {                                             
      timerCountsec = 0;                                                   
      timeUpsec = true;                                                    
    }
   
    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1;                               // Increment 200 millisecond timer count
    if(timerCount200msec > 200)                                              // If 200 milliseconds have elapsed
    {
      timerCount200msec = 0;                                                // Reset 200 millisecond timer count
      timeUp200msec = true;                                                 // Indicate that 200 milliseconds have elapsed
    }

    timerCount400msec = timerCount400msec + 1;                               
    if(timerCount400msec > 400)                                              
    {
      timerCount400msec = 0;                                                
      timeUp400msec = true;                                                
    }

    // 5 second timer, counts 5000 milliseconds
    timerCount5sec = timerCount5sec + 1;                               
    if(timerCount5sec > 5000)                                              
    {
      timerCount5sec = 0;                                                
      timeUp5sec = true;                                                
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {                                           // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
        if (modePBDebounce > 25) {                                         // if held for at least 25 mS
          modePBDebounce = 1000;                                          // change debounce timer count to 1 second
        }
      }
      if (modePBDebounce >= 1000) {                                         // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    }
    else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
      if(modePBDebounce <= 26) {                                            // if release occurs within debounce interval
        modePBDebounce = 0;                                                // reset debounce timer count
      }
      else {
        modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
        if(modePBDebounce >= 1025) {                                       // if pushbutton was released for 25 mS
          modePBDebounce = 0;                                             // reset debounce timer count
          robotModeIndex++;                                               // switch to next mode
          robotModeIndex = robotModeIndex & 7;                            // keep mode index between 0 and 7
          timerCount3sec = 0;                                             // reset 3 second timer count
          timeUp3sec = false;                                             // reset 3 second timer
        }
      }
    }
  
    // check if drive motors should be powered
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled
    switch(robotModeIndex) {
      case 0: // Robot stopped
        Bot.Stop("D1");    
        LeftEncoder.clearEncoder();                                        // clear encoder counts
        RightEncoder.clearEncoder();
        driveIndex = 0;                                                    // reset drive index
        timeUp2sec = false;                                                // reset 2 second timer
        break;

      case 1:
#ifdef DEBUG_ENCODER_COUNT
                if (timeUp200msec) {
                   timeUp200msec = false;                                       // reset 200 ms timer
                   LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                   RightEncoder.getEncoderRawCount();                           // read right encoder count
              //     Serial.print(F("Left Encoder count = "));
              //     Serial.print(LeftEncoder.lRawEncoderCount);
              //     Serial.print(F("  Right Encoder count = "));
              //     Serial.print(RightEncoder.lRawEncoderCount);
              //     Serial.print("\n");
                }
#endif
        
        
        timerAlarmWrite(pTimer, stepRate, true);             // update interrupt period to adjust step frequency
        

        switch(driveIndex){
          case 0:
            // leftDriveSpeed = map(4095, 0, 4095, cMinPWM, cMaxPWM);
            // rightDriveSpeed = map(4095, 0, 4095, cMinPWM, cMaxPWM) * 0.94;
            // Bot.Stop("D1");  
            // initiateMovement(100);
            // numOfLoops=0;
            Bot.ToPosition("S1", degreesToDutyCycle(0));
            Bot.ToPosition("S2", degreesToDutyCycle(85));
            Bot.ToPosition("S3", degreesToDutyCycle(180));
            timerCount100msec=0;
            servoTickCount=0;
            greenEntered=false;
            driveIndex++;
            break;
          
          case 1:
            if(servoTickCount < maxAngleArm){
              // if(timeUp200msec){
              //   Serial.println(servoTickCount);
              //   servoTickCount += servoIncr;
              //   Bot.ToPosition("S1", degreesToDutyCycle(servoTickCount));
              //   timeUp200msec=false;
              //   timerCount200msec=0;
              // }
              if(timeUp100msec){
                //Serial.println(servoTickCount);
                servoTickCount += servoIncr;
                Bot.ToPosition("S1", degreesToDutyCycle(servoTickCount));
                timeUp100msec=false;
                timerCount100msec=0;
              }
              // servoTickCount += servoIncr;
              // Bot.ToPosition("S1", degreesToDutyCycle(servoTickCount));
            } else {
              servoTickCount=0;
              timerCount500msec=0;
              timeUp500msec=false;
              driveIndex++;
            }
            break;

          case 2:
            Serial.println("2");
            if(timeUp500msec){
                driveIndex++;
            } 
          break;
          
          case 3:
            if(servoTickCount < maxAngleArm){
                // if(timeUp200msec){
                //   Serial.println(servoTickCount);
                //   servoTickCount += servoIncr;
                //   Bot.ToPosition("S1", degreesToDutyCycle(120-servoTickCount));
                //   timeUp200msec=false;
                //   timerCount200msec=0;
                // }
                if(timeUp100msec){
                  //Serial.println(servoTickCount);
                  servoTickCount += servoIncr;
                  Bot.ToPosition("S1", degreesToDutyCycle(maxAngleArm-servoTickCount));
                  timeUp100msec=false;
                  timerCount100msec=0;
                }
                // servoTickCount += servoIncr;
                // Bot.ToPosition("S1", degreesToDutyCycle(150-servoTickCount));
              } else {
                servoTickCount=0;
                driveIndex++;
              }
            break;
          
          case 4:
            Bot.Stop("D1");
            break;

        }

        //Sorting
        //if color sensor detects valuable {
          // turn sorting servo to valuable position
          // timerCount5sec=5000;
        // }

        if (isGreen(red_ratio, green_ratio, blue_ratio)) {
          greenTrue++;
          Serial.println(greenTrue);
          if(greenTrue > 3 && !greenEntered)
          {
            Serial.println("Green stone detected!");
            Bot.ToPosition("S2", degreesToDutyCycle(145));
            greenEntered=true;
            timerCount400msec=0;
            timeUp400msec=false;
          }
          
        } else {
          //Serial.println("No green stone detected.");
        }

        if(timeUp400msec && greenEntered){
          Bot.ToPosition("S2", degreesToDutyCycle(85));
          greenTrue=0;
          greenEntered=false;
        }

        
        break;
      
      case 2:
        robotModeIndex=0;
        break;

    }



    // Update brightness of heartbeat display on SmartLED
    displayTime++;                                                          // count milliseconds
    if (displayTime > cDisplayUpdate) {                                     // when display update period has passed
      displayTime = 0;                                                     // reset display counter
      LEDBrightnessIndex++;                                                // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {              // if all defined levels have been used
        LEDBrightnessIndex = 0;                                           // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // set brightness of heartbeat LED
      Indicator();                                                         // update LED
    }
  }
}

long distanceToEncoderCounts(float distance) {
    float wheelCircumference = 3.14159 * 6; // cm
    float revsNeeded = distance / wheelCircumference;
    return revsNeeded * cCountsRev;
}

// Function to initiate movement
void initiateMovement(float distance) {
    targetCount = distanceToEncoderCounts(distance);
    LeftEncoder.clearEncoder();
    RightEncoder.clearEncoder();
    movementComplete = false;
}

// Function to check movement completion
void checkMovementCompletion() {
    long leftCount = LeftEncoder.lRawEncoderCount;
    long rightCount = RightEncoder.lRawEncoderCount;
    long avgCount = (abs(leftCount) + abs(rightCount)) / 2;
    if (avgCount >= targetCount) {
        Bot.Stop("D1");
        movementComplete = true;
    }
}

void initiateTurn(int angle) {
    // Calculate the radius of the wheel
    float wheelRadius = 1;

    // Circumference
    float cir = wheelRadius * 2 * PI;

    // Convert angle to radians
    float angleRad = angle * PI / 180.0;

    // Calculate the distance traveled by one wheel for a 90-degree rotation
    float distance = cir * angleRad; 

    // Convert distance to encoder counts
    targetCount = distanceToEncoderCounts(distance);

    // Clear encoder counts
    LeftEncoder.clearEncoder();
    RightEncoder.clearEncoder();

    // Reset turn completion flag
    turnComplete = false;
}

void isTurnComplete() {
    // Calculate the absolute encoder counts for both wheels
    long leftCount = abs(LeftEncoder.lRawEncoderCount);
    long rightCount = abs(RightEncoder.lRawEncoderCount);
    long avgCount = (leftCount + rightCount) / 2;

    // Check if the average encoder counts reach the target counts
    if (avgCount >= targetCount) {
        Bot.Stop("D1");
        turnComplete = true;
    }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}

bool isGreen(float red_ratio, float green_ratio, float blue_ratio) {
  return green_ratio > red_ratio && green_ratio > blue_ratio && green_ratio > 0.33;
}

// timer interrupt service routine
void ARDUINO_ISR_ATTR timerISR() {
  if (runState) {                                      // Only send pulse if motor should be running
    digitalWrite(S_STEP_PIN, !digitalRead(S_STEP_PIN));    // toggle state of step pin
    if (stepDir) {
      stepCount++;                                     // add to count in forward direction
    }
    else {
      stepCount--;                                     // subtract from count in reverse direction
    }
  }
}


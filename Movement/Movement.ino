/*
Authors: Nathan Eng, Ben Prentice, Shayla Diep, Malak Al-Hanafi 
*/

#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Function declarations
void Indicator();                                                              // for mode/heartbeat on Smart LED

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

const int cDisplayUpdate = 100;
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed

boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;  
boolean movementComplete = false;
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

unsigned int robotModeIndex = 0;  
volatile int turnChange = 1;
volatile int numOfLoops = 0;
unsigned int maxLoops = 2;

Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

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
   modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop() {
  // put your main code here, to run repeatedly:
  long pos[] = {0, 0};                                                        // current motor positions

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
   
    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1;                               // Increment 200 millisecond timer count
    if(timerCount200msec > 200)                                              // If 200 milliseconds have elapsed
    {
      timerCount200msec = 0;                                                // Reset 200 millisecond timer count
      timeUp200msec = true;                                                 // Indicate that 200 milliseconds have elapsed
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
        switch(driveIndex){
          case 0:
            leftDriveSpeed = map(4095, 0, 4095, cMinPWM, cMaxPWM);
            rightDriveSpeed = map(4095, 0, 4095, cMinPWM, cMaxPWM);
            Bot.Stop("D1");  
            initiateMovement(50);
            numOfLoops=0;
            driveIndex++;
            break;
          
          case 1:
            if(!movementComplete){
              Bot.Forward("D1",leftDriveSpeed, rightDriveSpeed);
              checkMovementCompletion();
            } else{
              initiateTurn(90);
              driveIndex++;
            }
            break;
            
          case 2:
            Serial.println(turnChange);
            if(!turnComplete){
              if(turnChange>0){
                Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);
              } else {
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);
              }
              isTurnComplete();
            } else {
              initiateMovement(50);
              driveIndex++;
            }
          
          case 3:
            if(!movementComplete){
              Bot.Forward("D1",leftDriveSpeed, rightDriveSpeed);
              checkMovementCompletion();
            } else{
              initiateTurn(90);
              driveIndex++;
            }
            break;

          case 4:
            if(!turnComplete){
              if(turnChange>0){
                Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);
              } else {
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);
              }
              isTurnComplete();
            } else {
              initiateMovement(50);
              if(numOfLoops >= maxLoops){
                driveIndex++;
              } else {
                numOfLoops++;
                turnChange = turnChange*-1;
                driveIndex = 1;
              }
            }
        }

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
    float wheelRadius = 3;

    // Convert angle to radians
    float angleRad = angle * PI / 180.0;

    // Calculate the distance traveled by one wheel for a 90-degree rotation
    float distance = wheelRadius * angleRad; //1.5 to account for measurment error

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






/****************************************************************************
 Module
   TestHarnessService0.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for the framework and this service
*/
#include "ES_Configure.h"
#include "./Framework/ES_Framework.h"
#include "./Framework/ES_DeferRecall.h"
#include "UAVFSM.h"
#include "IMUService.h"
#include "RCService.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 10.24mS/tick timing
#define ONE_SEC 98
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)
#define CALI_VECTOR_LENGTH 50
#define CALI_PITCHROLL_LENGTH 500
#define CALI_RANGE 0.2
#define ZEROTIMERLENGTH 4000
#define INIT_THROTLE 1050
#define DEAD_BAND_MAX 1520
#define DEAD_BAND_MIN 1480
#define CONVERT_FACTOR 5.5
#define MAX_ROLL 400
#define MAX_PITCH 400
#define MAX_YAW 400
#define MAX_THROTTLE 1800
#define LOW_BATTERY 1000
#define MIN_BATTERY_RANGE 800
#define MAX_BATTERY_RANGE 1240
#define BATTERY_SCALE 3500
#define ESC_MAX 2000
#define ESC_MIN 1000

// pin config 
#define BATTERY_PIN A1
#define BATTERY_LED_PIN 12
#define CALI_COMPLETE_LED_PIN A0

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behaviour of this service
*/
static void pinConfig(void);
static void yawCali(void);
static void pitchrollCaliSum(void);
static bool yawCaliComplete(void);
static void yawCaliResult(void); 
static void calSetPoint(void);
static void getBatteryVoltage(void);
static void getReceiverInput(void);
static void getGyroValue(void);
static void calController(void);
static void ESCCommand(void);
static void isBatteryLow(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static float ypr[3];
static int RCInput[4];
static float yawCaliVector[CALI_VECTOR_LENGTH];
static double yawCalivalue = 0.00;
static double pitchCaliValue = 0.00;
static double rollCaliValue = 0.00;
static double yawCaliMax;
static double yawCaliMin;
static int yawCounter = 0;
static int pitchrollCounter = 0;
static UAVState_t CurrentState;
static unsigned long ZeroTimer;
static unsigned long TimerChannel1, TimerChannel2, TimerChannel3, TimerChannel4, ESCLoopTimer;
static int batteryVoltage;
static int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
static float lastGyroPitch, lastGyroRoll, lastGyroYaw;
static float currentGyroPitch, currentGyroRoll, currentGyroYaw;
static float pitchSetPoint, rollSetPoint, yawSetPoint;
static float currPitchError, currRollError, currYawError, lastPitchError, lastRollError, lastYawError ;
static float pitchSum, rollSum, yawSum;
static int throttle;
static int esc1, esc2, esc3, esc4;

// need to tune the PID gain here 
static float pRoll = 1.4;
static float iRoll = 0.05;
static float dRoll = 1; // use to be 15
static float pPitch = pRoll;
static float iPitch = iRoll;
static float dPitch = dRoll;
static float pYaw = 4.0;
static float iYaw = 0.02;
static float dYaw = 0.0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService0

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitUAVFSM ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/  
  // pin config
  pinConfig();
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  CurrentState = CaliUAVYaw;
  Serial.println("CurrentState = CaliUAVYaw");
  // CurrentState = CheckUAV;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostTestHarnessService0

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostUAVFSM ( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService0

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunUAVFSM( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors  
  switch (CurrentState){
    case CaliUAVYaw:
      if (ES_UPDATEYRP == ThisEvent.EventType){
        GetYPR(&ypr[0]);
        yawCali();
        if (yawCaliComplete()){
          yawCaliResult();
          // Initialize the interrupt for RC
          InitRCISR();
          CurrentState = CaliUAVPitchRoll;
          Serial.println("CurrentState = CaliUAVPitchRoll");
        }
      }
    break;

    case CaliUAVPitchRoll:
    if (ES_UPDATEYRP == ThisEvent.EventType){
        GetYPR(&ypr[0]);
        pitchrollCaliSum();
        // stop post ES_UPDATEYRP and will go to grab it when need 
        if (CALI_PITCHROLL_LENGTH == pitchrollCounter){
          turnOffIMUPost();
          pitchCaliValue = pitchCaliValue/(double)CALI_PITCHROLL_LENGTH;
          rollCaliValue = rollCaliValue/(double)CALI_PITCHROLL_LENGTH;
          CurrentState = CheckUAV;
          // calibration is completed
          digitalWrite(CALI_COMPLETE_LED_PIN, HIGH);
          // Serial.print("pitchCaliValue: ");
          // Serial.println(pitchCaliValue);
          // Serial.print("rollCaliValue: ");
          // Serial.println(rollCaliValue);  
          // Serial.print("pitchrollCounter: ");
          // Serial.println(pitchrollCounter);
          Serial.println("CurrentState = CheckUAV");
        }
      }
    break;

    case CheckUAV:
      if (ES_UPDATERC == ThisEvent.EventType){
        getReceiverInput(); 
        // input the first time might be all zero because there is not enought time to read from interrupt
        if (RCInput[0] != 0 && RCInput[1] != 0 && RCInput[2] != 0 && RCInput[3] != 0){  
          if (RCInput[2] < INIT_THROTLE){
            CurrentState = RunUAV;
            Serial.println("CurrentState = RunUAV");
          }
        }
      }
    break;

    case RunUAV:
    if (ES_UPDATERC == ThisEvent.EventType){
      getReceiverInput();
      getGyroValue();
      calSetPoint();
      calController();
      getBatteryVoltage();
      isBatteryLow();
      ESCCommand();
    }
    break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void pinConfig(void){
  pinMode(BATTERY_LED_PIN, OUTPUT);
  pinMode(CALI_COMPLETE_LED_PIN, OUTPUT);
  digitalWrite(CALI_COMPLETE_LED_PIN, LOW);
  digitalWrite(BATTERY_LED_PIN, LOW);
}

static void yawCali(void){
  if (yawCounter < CALI_VECTOR_LENGTH){
    yawCaliVector[yawCounter] = ypr[0]*(180/M_PI);
    yawCounter++;
  } else {
    for (int i = 0; i < CALI_VECTOR_LENGTH-1; i++){
      yawCaliVector[i] = yawCaliVector[i+1];
    }
    yawCaliVector[CALI_VECTOR_LENGTH-1] = ypr[0]*(180/M_PI);
  }
}

static void pitchrollCaliSum(void){
  pitchCaliValue += ypr[1]*(180/M_PI);
  rollCaliValue += ypr[2]*(180/M_PI);
  pitchrollCounter++;
}


static bool yawCaliComplete(void){
  if (yawCounter > CALI_VECTOR_LENGTH-1){
    yawCaliMax = -160.00;
    yawCaliMin = 160.00;
    for (int i = 0; i < CALI_VECTOR_LENGTH; i++){
      if (yawCaliVector[i] > yawCaliMax){
        yawCaliMax = yawCaliVector[i];
      }
      if (yawCaliVector[i] < yawCaliMin){
        yawCaliMin = yawCaliVector[i];
      }
    }
    Serial.println("CaliYawRange: ");
    Serial.print((yawCaliMax - yawCaliMin));
    Serial.println("");
    if ((yawCaliMax - yawCaliMin) < CALI_RANGE){
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

static void yawCaliResult(void){
  for (int i = 0; i < CALI_VECTOR_LENGTH; i++){
    yawCalivalue += yawCaliVector[i];
  }
  yawCalivalue /= (double) CALI_VECTOR_LENGTH;
}

static void getBatteryVoltage(void){
  // 10.5V is 1050
  batteryVoltage = (analogRead(BATTERY_PIN) + 65) * 1.2317;
  // Serial.print("batteryVoltage \t");
  // Serial.println(batteryVoltage);
  // batteryVoltage = 1200;
}

static void isBatteryLow(void){
  if (batteryVoltage < LOW_BATTERY){
    // battery is high
    digitalWrite(BATTERY_LED_PIN, HIGH);
  } else {
    // battery is low
    digitalWrite(BATTERY_LED_PIN, LOW);
  }
}

static void getReceiverInput(void){
  GetRC(&RCInput[0]);  
  receiverInputChannel1 = RCInput[0];
  receiverInputChannel2 = RCInput[1];
  receiverInputChannel3 = RCInput[2];
  receiverInputChannel4 = RCInput[3];
  // Serial.print("RCInput\t");
  // Serial.print(RCInput[0]);
  // Serial.print("\t");
  // Serial.print(RCInput[1]);
  // Serial.print("\t");
  // Serial.print(RCInput[2]);
  // Serial.print("\t");
  // Serial.println(RCInput[3]);
}

static void getGyroValue(void){
  if (GetUpdateStatus()){
    GetYPR(&ypr[0]);
    currentGyroYaw = (ypr[0] * 180/M_PI) - yawCalivalue;
    currentGyroPitch = (ypr[1] * 180/M_PI) - pitchCaliValue;
    currentGyroRoll = (ypr[2] * 180/M_PI) - rollCaliValue;
    currentGyroYaw = (lastGyroYaw * 0.8) + (currentGyroYaw * 0.2);            //Gyro pid input is deg/sec.
    currentGyroPitch = (lastGyroPitch * 0.8) + (currentGyroPitch * 0.2);      //Gyro pid input is deg/sec.
    currentGyroRoll = (lastGyroRoll * 0.8) + (currentGyroRoll * 0.2);         //Gyro pid input is deg/sec.
    // Serial.print("ypr\t");
    // Serial.print(currentGyroYaw);
    // Serial.print("\t");
    // Serial.print(currentGyroPitch);
    // Serial.print("\t");
    // Serial.println(currentGyroRoll);
    lastGyroYaw = currentGyroYaw;
    lastGyroPitch = currentGyroPitch;
    lastGyroRoll = currentGyroRoll;
    ResetUpdateStatus();
  }
}

static void calSetPoint(void){
  // channel 1 is roll
  if (receiverInputChannel1 > DEAD_BAND_MAX){
    rollSetPoint = (receiverInputChannel1 - DEAD_BAND_MAX)/CONVERT_FACTOR;
  } else if (receiverInputChannel1 < DEAD_BAND_MIN){
    rollSetPoint = (receiverInputChannel1 - DEAD_BAND_MIN)/CONVERT_FACTOR;
  } else {
    rollSetPoint = 0.0;
  }

  // channel 2 is pitch
  if (receiverInputChannel2 > DEAD_BAND_MAX){
    pitchSetPoint = (receiverInputChannel2 - DEAD_BAND_MAX)/CONVERT_FACTOR;
  } else if (receiverInputChannel2 < DEAD_BAND_MIN){
    pitchSetPoint = (receiverInputChannel2 - DEAD_BAND_MIN)/CONVERT_FACTOR;
  } else {
    pitchSetPoint = 0.0;
  }

  // channel 4 is yaw
  if (receiverInputChannel3 > INIT_THROTLE){
    if (receiverInputChannel4 > DEAD_BAND_MAX){
      yawSetPoint = (receiverInputChannel4 - DEAD_BAND_MAX)/CONVERT_FACTOR;
    } else if (receiverInputChannel4 < DEAD_BAND_MIN){
      yawSetPoint = (receiverInputChannel4 - DEAD_BAND_MIN)/CONVERT_FACTOR;
    } else {
      yawSetPoint = 0.0;
    }
  } else {
    yawSetPoint = 0.0;
  }
  // Serial.print("setPointypr\t");
  // Serial.print(yawSetPoint);
  // Serial.print("\t");
  // Serial.print(pitchSetPoint);
  // Serial.print("\t");
  // Serial.println(rollSetPoint);
}

static void calController(void){
  // roll
  currRollError = currentGyroRoll - rollSetPoint;
  rollSum += currRollError*iRoll;
  if (rollSum > MAX_ROLL){
    rollSum = MAX_ROLL;
  } else if (rollSum < -1*MAX_ROLL){
    rollSum = -1*MAX_ROLL;
  }
  currRollError = pRoll*currRollError + rollSum + dRoll*(currRollError - lastRollError);
  if (currRollError > MAX_ROLL){
    currRollError = MAX_ROLL;
  } else if (currRollError < -1*MAX_ROLL){
    currRollError = -1*MAX_ROLL;
  }
  lastRollError = currRollError;


  // pitch
  currPitchError = currentGyroPitch - pitchSetPoint;
  pitchSum += currPitchError*iPitch;
  if (pitchSum > MAX_PITCH){
    pitchSum = MAX_PITCH;
  } else if (pitchSum < -1*MAX_PITCH){
    pitchSum = -1*MAX_PITCH;
  }
  currPitchError = pPitch*currPitchError + pitchSum + dPitch*(currPitchError - lastPitchError);
  if (currPitchError > MAX_PITCH){
    currPitchError = MAX_PITCH;
  } else if (currPitchError < -1*MAX_PITCH){
    currPitchError = -1*MAX_PITCH;
  }
  lastPitchError = currPitchError;


  // yaw
  currYawError = currentGyroYaw - yawSetPoint;
  yawSum += currYawError*iYaw;
  if (yawSum > MAX_YAW){
    yawSum = MAX_YAW;
  } else if (yawSum < -1*MAX_YAW){
    yawSum = -1*MAX_YAW;
  }
  currYawError = pYaw*currYawError + yawSum + dYaw*(currYawError - lastYawError);
  if (currYawError > MAX_YAW){
    currYawError = MAX_YAW;
  } else if (currYawError < -1*MAX_YAW){
    currYawError = -1*MAX_YAW;
  }
  lastYawError = currYawError;
  // Serial.print("currErrorypr\t");
  // Serial.print(currYawError);
  // Serial.print("\t");
  // Serial.print(currPitchError);
  // Serial.print("\t");
  // Serial.println(currRollError);
}

static void ESCCommand(void){
  // get throttle value
  throttle = receiverInputChannel3;
  // set up limit of throttle
  if (throttle > MAX_THROTTLE){
    throttle = MAX_THROTTLE;
  }
  // calcualte esc value
  esc1 = throttle - currPitchError + currRollError - currYawError;
  esc2 = throttle + currPitchError + currRollError + currYawError;
  esc3 = throttle + currPitchError - currRollError - currYawError;
  esc4 = throttle - currPitchError - currRollError + currYawError;
  
  // compensate voltage seg
  if ((batteryVoltage < MAX_BATTERY_RANGE) && (batteryVoltage > MIN_BATTERY_RANGE)){
    esc1 += esc1*((MAX_BATTERY_RANGE - batteryVoltage)/(float)BATTERY_SCALE);
    esc2 += esc2*((MAX_BATTERY_RANGE - batteryVoltage)/(float)BATTERY_SCALE);
    esc3 += esc3*((MAX_BATTERY_RANGE - batteryVoltage)/(float)BATTERY_SCALE);
    esc4 += esc4*((MAX_BATTERY_RANGE - batteryVoltage)/(float)BATTERY_SCALE);
  }
  // compensate wind-up: anti-windup
  if (esc1 > ESC_MAX){
    esc1 = ESC_MAX;
  }
  if (esc2 > ESC_MAX){
    esc2 = ESC_MAX;
  }
  if (esc3 > ESC_MAX){
    esc3 = ESC_MAX;
  }
  if (esc4 > ESC_MAX){
    esc4 = ESC_MAX;
  }

  // compensate wind-up: anti-windup
  if (esc1 < ESC_MIN){
    esc1 = ESC_MIN;
  }
  if (esc2 < ESC_MIN){
    esc2 = ESC_MIN;
  }
  if (esc3 < ESC_MIN){
    esc3 = ESC_MIN;
  }
  if (esc4 < ESC_MIN){
    esc4 = ESC_MIN;
  }

  Serial.print("esc1234\t");
  Serial.print(esc1);
  Serial.print("\t");
  Serial.print(esc2);
  Serial.print("\t");
  Serial.print(esc3);
  Serial.print("\t");
  Serial.println(esc4);

  // pass control commmand to ESC
  if ((micros() - ZeroTimer) > ZEROTIMERLENGTH){
  ZeroTimer = micros();
  PORTD |= B11110000;                                        //Set port 4, 5, 6 and 7 high at once
  TimerChannel1 = esc1 + ZeroTimer;   //Calculate the time when digital port 4 is set low
  TimerChannel2 = esc2 + ZeroTimer;   //Calculate the time when digital port 5 is set low
  TimerChannel3 = esc3 + ZeroTimer;   //Calculate the time when digital port 6 is set low
  TimerChannel4 = esc4 + ZeroTimer;   //Calculate the time when digital port 7 is set low
  while(PORTD >= 16){                                        //Execute the loop until digital port 4 to 7 is low
    ESCLoopTimer = micros();                               //Check the current time
    if(TimerChannel1 <= ESCLoopTimer)PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low
    if(TimerChannel2 <= ESCLoopTimer)PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low
    if(TimerChannel3 <= ESCLoopTimer)PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low
    if(TimerChannel4 <= ESCLoopTimer)PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low
  }
}
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/


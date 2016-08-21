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
#define YAW_CALI_VECTOR_LENGTH 50
#define CALI_RANGE 0.2
#define ZEROTIMERLENGTH 4000

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behaviour of this service
*/
static void yawCali(void);
static bool yawCaliComplete(void);
static void yawCaliResult(void); 

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static float ypr[3];
static int RCInput[4];
static float yawCaliVector[YAW_CALI_VECTOR_LENGTH];
static double yawCalivalue = 0.00;
static double yawCaliMax;
static double yawCaliMin;
static int yawCounter = 0;
static UAVState_t CurrentState;
static unsigned long ZeroTimer;
unsigned long TimerChannel1, TimerChannel2, TimerChannel3, TimerChannel4, ESCLoopTimer;
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
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  // CurrentState = CaliUAV;
  CurrentState = RunUAV;
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
    case CaliUAV:
      if (ES_UPDATEYRP == ThisEvent.EventType){
        GetYPR(&ypr[0]);
        yawCali();
        if (yawCaliComplete()){
          yawCaliResult();
          CurrentState = RunUAV;
        }
      }
    break;

    case RunUAV:
      // if (ES_UPDATEYRP == ThisEvent.EventType){
      //   GetYPR(&ypr[0]);
      //   Serial.print("ypr\t");
      //   Serial.print((ypr[0] * 180/M_PI)-yawCalivalue);
      //   Serial.print("\t");
      //   Serial.print(ypr[1] * 180/M_PI);
      //   Serial.print("\t");
      //   Serial.println(ypr[2] * 180/M_PI);
      // }
    if (ES_UPDATERC == ThisEvent.EventType){
      if ((micros() - ZeroTimer) > ZEROTIMERLENGTH){
        GetRC(&RCInput[0]);
        Serial.print("RCInput\t");
        Serial.print(RCInput[0]);
        Serial.print("\t");
        Serial.print(RCInput[1]);
        Serial.print("\t");
        Serial.print(RCInput[2]);
        Serial.print("\t");
        Serial.println(RCInput[3]);  
        ZeroTimer = micros();
        PORTD |= B11110000;                                        //Set port 4, 5, 6 and 7 high at once
        TimerChannel1 = RCInput[2] + ZeroTimer;   //Calculate the time when digital port 4 is set low
        TimerChannel2 = RCInput[2] + ZeroTimer;   //Calculate the time when digital port 5 is set low
        TimerChannel3 = RCInput[2] + ZeroTimer;   //Calculate the time when digital port 6 is set low
        TimerChannel4 = RCInput[2] + ZeroTimer;   //Calculate the time when digital port 7 is set low
        while(PORTD >= 16){                                        //Execute the loop until digital port 4 to 7 is low
          ESCLoopTimer = micros();                               //Check the current time
          if(TimerChannel1 <= ESCLoopTimer)PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low
          if(TimerChannel2 <= ESCLoopTimer)PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low
          if(TimerChannel3 <= ESCLoopTimer)PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low
          if(TimerChannel4 <= ESCLoopTimer)PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low
        }
      }
    }
    break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void yawCali(void){
  if (yawCounter < YAW_CALI_VECTOR_LENGTH){
    yawCaliVector[yawCounter] = ypr[0]*(180/M_PI);
    yawCounter++;
  } else {
    for (int i = 0; i < YAW_CALI_VECTOR_LENGTH-1; i++){
      yawCaliVector[i] = yawCaliVector[i+1];
    }
    yawCaliVector[YAW_CALI_VECTOR_LENGTH-1] = ypr[0]*(180/M_PI);
  }
}

static bool yawCaliComplete(void){
  if (yawCounter > YAW_CALI_VECTOR_LENGTH-1){
    yawCaliMax = -160.00;
    yawCaliMin = 160.00;
    for (int i = 0; i < YAW_CALI_VECTOR_LENGTH; i++){
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
  for (int i = 0; i < YAW_CALI_VECTOR_LENGTH; i++){
    yawCalivalue += yawCaliVector[i];
  }
  yawCalivalue /= YAW_CALI_VECTOR_LENGTH;
}


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/


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
#include "TestHarnessService0.h"


/*----------------------------- Module Defines ----------------------------*/
// these times assume a 10.24mS/tick timing
#define ONE_SEC 98
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behaviour of this service
*/

static void InitLED(void);
static void BlinkLED(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event DeferralQueue[3+1];

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
bool InitTestHarnessService0 ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
	// initialize deferral queue for testing Deferal function
  ES_InitDeferralQueueWith( DeferralQueue, ARRAY_SIZE(DeferralQueue) );
	// initialize LED drive for testing/debug output
	InitLED();

  
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
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
bool PostTestHarnessService0( ES_Event ThisEvent )
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
ES_Event RunTestHarnessService0( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	static char DeferredChar = '1';
  
  switch (ThisEvent.EventType){
    case ES_INIT :
      ES_Timer_InitTimer(SERVICE0_TIMER, HALF_SEC);
      Serial.print(F("Service 00: "));
      Serial.print(F("ES_INIT received in Service"));
      Serial.println(MyPriority);
      break;
    case ES_TIMEOUT :  // re-start timer & announce
      ES_Timer_InitTimer(SERVICE0_TIMER, FIVE_SEC);
      Serial.print(F("ES_TIMEOUT received from Timer "));
      Serial.print(ThisEvent.EventParam);
      Serial.print(F(" in Service ")); 
      Serial.println(MyPriority);
			BlinkLED();
      break;
    case ES_NEW_KEY :  // announce
      Serial.print(F("ES_NEW_KEY received with ->"));
      Serial.write((char)ThisEvent.EventParam);
      Serial.println(F(" <- in Service 0")); 
              
      if( 'd' == ThisEvent.EventParam ){
          ThisEvent.EventParam = DeferredChar++; //
          if (ES_DeferEvent( DeferralQueue, ThisEvent )){
            Serial.println(F("ES_NEW_KEY deferred in Service 0\r"));
          }
      }
      if( 'r' == ThisEvent.EventParam ){
          ThisEvent.EventParam = 'Q'; // This one gets posted normally
          ES_PostToService( MyPriority, ThisEvent);
          // but we slide the deferred events under it so it(they) should come out first
          if ( true == ES_RecallEvents( MyPriority, DeferralQueue )){
            Serial.println(F("ES_NEW_KEY(s) recalled in Service 0"));
					DeferredChar = '1';
          }
      }
    default :
      break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static void InitLED(void)
{
}


static void BlinkLED(void)
{
	static uint8_t LEDvalue = 2;
	
	// Turn off all of the LEDs
	// Turn on the new LEDs
	
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/


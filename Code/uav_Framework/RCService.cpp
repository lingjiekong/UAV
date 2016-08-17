/****************************************************************************
 Module
   RCService.c

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
#include "RCService.h" 

/*----------------------------- Module Defines ----------------------------*/
// Pin define

// other define


/*---------------------------- Global Functions ---------------------------*/
void RCISR(void);

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behaviour of this service
*/
static void InitRCISR(void);   

/*---------------------------- Module Variables ---------------------------*/
// everybode needs a state machine variable, you may need others as well.
// type of state variable should match that of enum in header file

// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
static unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
static int receiver_input[5];
static bool RCISRFlag = false;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDetachModuleFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Lingjie Kong, 06/17/16, 10:00
****************************************************************************/
bool InitRCService ( uint8_t Priority )
{
  ES_Event ThisEvent;
  
  MyPriority = Priority;
  /********************************************
   in here you write your initialization codes
   *******************************************/
  InitRCISR();
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
     PostDetachModuleFSM

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Lingjie Kong, 06/17/16, 10:00
****************************************************************************/
bool PostRCService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunPostDetachModuleFSM

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   Lingjie Kong, 06/17/16, 15:23
****************************************************************************/
ES_Event RunRCService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  // if programming failed, don't try to do anything
  return ReturnEvent;
}

/***************************************************************************
 global functions
 ***************************************************************************/
 // Interrupt
ISR(PCINT0_vect){
  RCISRFlag = true;
}

// Interrupt Service Routine
void RCISR(void){
  if (true == RCISRFlag){
      current_time = micros();
    //Channel 1=========================================
    if(PINB & B00000001){                                        //Is input 8 high?
      if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
        last_channel_1 = 1;                                      //Remember current input state
        timer_1 = current_time;                                  //Set timer_1 to current_time
      }
    }
    else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
      last_channel_1 = 0;                                        //Remember current input state
      receiver_input[1] = current_time - timer_1;                //Channel 1 is current_time - timer_1
    }
    //Channel 2=========================================
    if(PINB & B00000010 ){                                       //Is input 9 high?
      if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
        last_channel_2 = 1;                                      //Remember current input state
        timer_2 = current_time;                                  //Set timer_2 to current_time
      }
    }
    else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
      last_channel_2 = 0;                                        //Remember current input state
      receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2
    }
    //Channel 3=========================================
    if(PINB & B00000100 ){                                       //Is input 10 high?
      if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
        last_channel_3 = 1;                                      //Remember current input state
        timer_3 = current_time;                                  //Set timer_3 to current_time
      }
    }
    else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
      last_channel_3 = 0;                                        //Remember current input state
      receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3

    }
    //Channel 4=========================================
    if(PINB & B00001000 ){                                       //Is input 11 high?
      if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
        last_channel_4 = 1;                                      //Remember current input state
        timer_4 = current_time;                                  //Set timer_4 to current_time
      }
    }
    else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
      last_channel_4 = 0;                                        //Remember current input state
      receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4
    }
    RCISRFlag = false;
  }
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void InitRCISR(void){ // tie the interrupt to the pin
  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/


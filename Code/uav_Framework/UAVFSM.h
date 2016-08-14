/****************************************************************************
 
  Header file for Test Harness Service0 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef UAVFSM_H
#define UAVFSM_H

#include "ES_Configure.h"
#include "./Framework/ES_Types.h"

// Public Function Prototypes

bool InitUAVFSM( uint8_t Priority );
bool PostUAVFSM( ES_Event ThisEvent );
ES_Event RunUAVFSM( ES_Event ThisEvent );


#endif /* ServUAVFSM_H */


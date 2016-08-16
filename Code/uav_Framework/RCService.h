/****************************************************************************
 
  Header file for Test Harness Service0 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef RCSERVICE_H
#define RCSERVICE_H

#include "ES_Configure.h"
#include "./Framework/ES_Types.h"

// Public Function Prototypes

bool InitRCService( uint8_t Priority );
bool PostRCService( ES_Event ThisEvent );
ES_Event RunRCService( ES_Event ThisEvent );

void RCISR(void);

#endif /* ServRCSERVICE_H */


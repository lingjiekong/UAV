/****************************************************************************
 
  Header file for Test Harness Service0 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef IMUService_H
#define IMUService_H

#include "ES_Configure.h"
#include "./Framework/ES_Types.h"


// Public Function Prototypes
bool InitIMUService( uint8_t Priority );
bool PostIMUService( ES_Event ThisEvent );
ES_Event RunIMUService( ES_Event ThisEvent );

// double GetYaw(void);
// double GetRoll(void);
// double GetPitch(void);
void IMUISR(void);

#endif /* SerIMUService_H */


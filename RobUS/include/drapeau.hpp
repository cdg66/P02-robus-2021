#ifndef _DRAPEAUX_H__
#define _DRAPEAUX_H__

#include "stdint.h"
// define

// public variables

// public function
void drapeaux_Init(void);
void drapeaux_Drop(uint8_t ServoID);
void drapeaux_Lock(uint8_t ServoID);

void drapeaux_DropAll(void);
void drapeaux_LockAll(void);
#endif

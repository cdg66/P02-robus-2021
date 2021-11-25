#ifndef _MINE_DETECT_H__
#define _MINE_DETECT_H__

#include "stdint.h"
// define
#define MAXNORMBUFFERSIZE 10
#define MINEDETECTIONVALUE 0.6

// public variables
struct _mine
{
    uint8_t processStatus = 0;
    bool mineDetected  = 0;
};
// public function
void mineDetection_Init(void);
void mineDetection_Enable(void);
void mineDetection_Disable(void);
#endif

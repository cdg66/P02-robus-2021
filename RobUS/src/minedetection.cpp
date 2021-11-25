#include "minedetection.hpp"
#include "LibRobus.h"
#include "SoftTimer.hpp"
#include "magsensor.hpp"
#include "manuel.hpp"
//public var
struct _mine mine;

//private var
float VectorArray[3];
float NormBuffer[MAXNORMBUFFERSIZE];
uint8_t OldestSampleIndex = 0;



void mineDetection_Callback(void)
{
    float Median;
    int i;
    mine.processStatus = 1;
    // lit un sample
    MagSensor_GetData(VectorArray,&NormBuffer[OldestSampleIndex]);
    // Serial.print(VectorArray[0], 3);
    // Serial.print(" ");
    // Serial.print(VectorArray[1], 3);
    // Serial.print(" ");
    // Serial.print(VectorArray[2], 3);
    // Serial.print(" ");
    // Serial.print("\n");
    // change l'index
    OldestSampleIndex++;
    if (OldestSampleIndex >= MAXNORMBUFFERSIZE)
    {
        OldestSampleIndex = 0;
    }

    // calcul la moyenne
    for (i = 0; i < MAXNORMBUFFERSIZE; i++)
    {
      Median = Median + NormBuffer[i];
    }
    Median = Median/MAXNORMBUFFERSIZE;
    Serial.print(Median, 6);
    Serial.print("\n");
    // mine detecte?
    if (Median < MINEDETECTIONVALUE )
    {
        mine.mineDetected = 0;
        mineStatus(mine.mineDetected);
        return;
    }
    mine.mineDetected = 1;
    mineStatus(mine.mineDetected);
    //Serial.print("mine presente ");
    //Serial.print("\n");

}

void mineDetection_Init(void)
{
  //MagSensor_Init();
  SOFT_TIMER_SetCallback(ID_MINE, &mineDetection_Callback);
  SOFT_TIMER_SetDelay(ID_MINE, 10);
  SOFT_TIMER_SetRepetition(ID_MINE, -1);
  //SOFT_TIMER_Enable(ID_MINE);
}

void mineDetection_Enable(void)
{
    SOFT_TIMER_Enable(ID_MINE);
}
void mineDetection_Disable(void)
{
    SOFT_TIMER_Disable(ID_MINE);
}



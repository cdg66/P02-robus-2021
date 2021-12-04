#include "drapeau.hpp"
#include "servo.hpp"
#include "stdint.h"
#include "LibRobus.h"

void drapeaux_Init(void)
{
    
}
void drapeaux_Drop(uint8_t ServoID)
{
    SERVO_SetPWM(ServoID, SERVO_OUVERT);
    delay(500);
    SERVO_SetPWM(ServoID, SERVO_FERME);
}
void drapeaux_Lock(uint8_t ServoID)
{
    SERVO_SetPWM(ServoID, SERVO_FERME);
}

void drapeaux_DropAll()
{
    int i;
for (i = 0; i < 16; i++)
{
    SERVO_SetPWM(i, SERVO_OUVERT);
}

}
void drapeaux_LockAll()
{
    int i;
    for (i = 0; i < 16; i++)
    {
      SERVO_SetPWM(i, SERVO_FERME);
    }

}
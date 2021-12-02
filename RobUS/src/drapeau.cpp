#include "drapeau.hpp"
#include "servo.hpp"
#include "stdint.h"

void drapeaux_Init(void)
{
    
}
void drapeaux_Drop(uint8_t ServoID)
{
    SERVO_SetPWM(ServoID, SERVOMAX);
}
void drapeaux_Lock(uint8_t ServoID)
{
    SERVO_SetPWM(ServoID, SERVOMAX/2);
}

void drapeaux_DropAll()
{
    int i;
for (i = 0; i < 16; i++)
{
    SERVO_SetPWM(i, SERVOMAX);
}

}
void drapeaux_LockAll()
{
    int i;
    for (i = 0; i < 16; i++)
    {
      SERVO_SetPWM(i, SERVOMAX/2);
    }

}
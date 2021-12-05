#include "mouvement.hpp"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "SoftTimer.hpp"
// value of encoder needed for avancer() and pid()
int32_t totalG, totalD;
float vitesseD = -0.35;
float vitesseG = -0.35;

/*------------------------------------------------- tourner ----------
|  Function tourner
|
|  Purpose:  Make RobUS do a turn on himself with only one weel active
|
|  Parameters:
|     type float 
|          angle(IN) angle of witch the robot need to face after his turn
|          give a negative value to turn left and a positive one to
|          turn right.
|
|  Constant : 
|     CLIC_DEGREE: Number of encoder pulse for 1 degrees of turn.
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void tourner(float angle)
{
  int32_t clic = CLIC_DEGREE * abs(angle);
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(angle<0)
  {
  MOTOR_SetSpeed(RIGHT, 0.5);

  while(ENCODER_Read(RIGHT)<clic)
  {}
  }
  else{
  MOTOR_SetSpeed(LEFT, 0.5);

  while(ENCODER_Read(LEFT)<clic)
  {}
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

/*------------------------------------------------- tourner ----------
|  Function tourner
|
|  Purpose:  Make RobUS do a turn on himself with only one weel active
|
|  Parameters:
|     type float 
|          angle(IN) angle of witch the robot need to face after his turn
|          give a negative value to turn left and a positive one to
|          turn right.
|
|  Constant : 
|     CLIC_DEGREE: Number of encoder pulse for 1 degrees of turn.
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void tournerSuiveur(float angle)
{
  int32_t clic = CLIC_DEGREE * abs(angle);
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(angle<0)
  {
  MOTOR_SetSpeed(RIGHT, 0.2);

  while(ENCODER_Read(RIGHT)<clic)
  {}
  }
  else{
  MOTOR_SetSpeed(LEFT, 0.2);

  while(ENCODER_Read(LEFT)<clic)
  {}
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

void tournerSelf(float angle,float speed)
{
  int32_t clic = CLIC_DEGREE * abs(angle);
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(angle<0)
  {
  MOTOR_SetSpeed(RIGHT, speed);
  MOTOR_SetSpeed(LEFT, -(speed));
  while(ENCODER_Read(RIGHT)<clic)
  {}
  }
  else{
  MOTOR_SetSpeed(RIGHT, -(speed));
  MOTOR_SetSpeed(LEFT, speed);

  while(ENCODER_Read(LEFT)<clic)
  {}
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

/*------------------------------------------------- avancer_distance ----------
|  Function avancer_distance
|
|  Purpose:  Make RobUS move forward to a fixed distance
|
|  Parameters:
|     type float 
|          distance(IN) distance in centimeter (cm) with the robot need
|          to move.
|
|  Constant :
|     CLIC_CM: number of pulse of the encoder for one cm forward
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void avancer_distance(float distance)
{

  int32_t clics = distance * CLIC_CM;
  delay(100);
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
  MOTOR_SetSpeed(RIGHT, vitesseD); 
  MOTOR_SetSpeed(LEFT, vitesseG);
  totalD=0;
  totalG=0;
  while(totalG < clics)
  {
  delay(100);
  pid();
  

  }
  pidReset();
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
 
}

void avancer_timer(float distance)
{

  int32_t clics = distance * CLIC_CM;
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
  MOTOR_SetSpeed(RIGHT, vitesseD); 
  MOTOR_SetSpeed(LEFT, vitesseG);
  
  totalD=0;
  totalG=0;
  SOFT_TIMER_Enable(ID_PID);
  while(totalG < clics)
  {
    SOFT_TIMER_Update();
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  SOFT_TIMER_Disable(ID_PID);
  pidReset();
}

/*------------------------------------------------- pid ---------------
|  Function pid
|
|  Purpose:  feedback loop for the motor speed
|
|  Parameters: nothing
|  Constant :
|       KP : Multiplicative constant of the proportional factor need to 
|            be set by the user.
|       KI : Multiplicative constant of the Integral factor need to 
|            be set by the user.
|  Variables :
| 
|  Dependency : LibRobUS
|       
|  Returns:    nothing
*-------------------------------------------------------------------*/
void pid()
{
  static int32_t erreurP, erreurI , pulseG, pulseD;
  float Correction;

  pulseD = ENCODER_ReadReset(RIGHT);
  //Serial.print(pulseD, DEC);
  //Serial.print("\n\r");
  pulseG = ENCODER_ReadReset(LEFT);
  //Serial.print(pulseG, DEC);
  //Serial.print("\n\r");


  totalG = totalG + pulseG; 
  //Serial.print(totalG, DEC);
  //Serial.print("\n\r");
  totalD = totalD + pulseD;
  //Serial.print(totalD, DEC);
  //Serial.print("\n\r");
  erreurP = pulseG - pulseD;
  //Serial.print(erreurP, DEC);
  //Serial.print("\n\r");
  //erreurI = erreurP * 100;
  //erreurI = totalG - totalD;
  Correction = (erreurP*KP) + (erreurI*KI);
  //Serial.print(Correction, DEC);
  //Serial.print("\n\r");
  vitesseD = vitesseD + Correction;
  MOTOR_SetSpeed(RIGHT,vitesseD);
  //ENCODER_Reset(LEFT);
  //ENCODER_Reset(RIGHT);

}
/*------------------------------------------------- pidReset ---------------
|  Function pid
|
|  Purpose:  reset pid values
|
|  Parameters: nothing
|  Constant : nothing
|  Variables :
| 
|  Dependency : LibRobUS
|       
|  Returns:    nothing
*-------------------------------------------------------------------*/
void pidReset()
{
  totalG = 0;
  totalD = 0;
  vitesseD = 0.30;
}

/*------------------------------------------------- uTurn ---------------
|  Function uTurn
|
|  Purpose:  make robUS do a full 180 degree.
|
|  Parameters: nothing
|  Constant :  nothing
|  Dependency : LibRobUS
|  Returns:    nothing
*-------------------------------------------------------------------*/
void uTurn()
{
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
  MOTOR_SetSpeed(LEFT, -0.5);
  MOTOR_SetSpeed(RIGHT, 0.5);

  while(ENCODER_Read(RIGHT)<3550){
    SOFT_TIMER_Update();
  }
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}
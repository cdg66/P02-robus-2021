#include <librobus.h>
#include <Arduino.h>
#include <math.h>
#define VERSIONID "Version capteurs"


#define CLIC_DEGREE 44
#define CLIC_CM 133.4
#define KP 0.0001
#define KI 0.00002


float valeurSonar;




// value of encoder needed for avancer()
int32_t totalG, totalD;

void tourner(float angle);
void avancer (float distance);
void uTurn();
void pid();
void getSonarRange();

void setup() {
  BoardInit();
  avancer(2000);
/*
u_turn();

avancer(200); */

/*   avancer(120);
  tourner(-43);
  avancer(50);
  tourner(47);
  avancer(100);
  avancer(60);
  tourner(45);
  avancer(63);
  tourner(-43);
  avancer(120);

  delay(500);
  uTurn();
  delay(500);

  avancer(110);
  avancer(100);
  avancer(100);
  avancer(100);
  avancer(100); */
}

void loop(){
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
void tourner(float angle)
{
  uint32_t clic = CLIC_DEGREE * abs(angle);
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
/*------------------------------------------------- avancer ----------
|  Function avancer
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
void avancer(float distance)
{
  float distanceMur = 15.0;
  

  
  
  int32_t clics = distance * CLIC_CM;
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);  
  MOTOR_SetSpeed(LEFT, 0.5);
  MOTOR_SetSpeed(RIGHT, 0.45);
  totalD=0;
  totalG=0;
  
  while((totalG < clics)&&(SONAR_GetRange(1) > distanceMur))
  {
    //while(SONAR_GetRange(1) > distanceMur)
    //{
    Serial.println(SONAR_GetRange(1));
    delay(100);
    pid();
    //}

   
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
 
}

void getSonarRange()
{
  valeurSonar = SONAR_GetRange(1);

  Serial.println(valeurSonar);
  
  delay(100);
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
  int32_t erreur1, pulseG, pulseD, erreurVitesse, correction;

  pulseD=ENCODER_Read(RIGHT);
  pulseG=ENCODER_Read(LEFT);
  
  totalG += pulseG; 
  totalD += pulseD;
  erreur1 = totalG - totalD;
  erreurVitesse = pulseG -pulseD;
  correction= KP*erreur1+erreurVitesse*KI;
  MOTOR_SetSpeed(RIGHT,(0.49+correction));
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
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

  while(ENCODER_Read(RIGHT)<3650){}
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}
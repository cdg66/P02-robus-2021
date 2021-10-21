#include <librobus.h>
#include <Arduino.h>
#include <math.h>
#define VERSIONID "Version PID"


#define CLIC_DEGREE 44
#define CLIC_CM 133.4
#define KP 0.0001
#define KI 0.00002

int32_t totalG, totalD, erreur1, pulseG, pulseD, erreurVitesse, correction;


void tourner(float angle);
void avancer (float distance);
void uTurn();
void pid();

void setup() {
  BoardInit();


}

void loop(){}

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

void avancer(float distance)
{
  int32_t clics = distance * CLIC_CM;
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);  
  MOTOR_SetSpeed(LEFT, 0.5);
  MOTOR_SetSpeed(RIGHT, 0.45);
  totalD=0;
  totalG=0;
  
  while(totalG < clics)
  {
    delay(100);
    pid();
   
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

void pid()
{
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
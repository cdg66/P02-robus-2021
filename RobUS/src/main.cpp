#include <librobus.h>
#include <Arduino.h>
#include <math.h>
#define VERSIONID "Version PID"


#define CLIC_DEGREE 44
#define CLIC_CM 133.67
#define kp 0.0001
#define ki 0.00002

int32_t total_G, total_D, erreur_1, pulse_G, pulse_D, erreur_vitesse, correction;


void tourner(float angle);
void avancer (float distance);
void u_turn();
void PID();

void setup() {
  BoardInit();
  
  avancer(210);
  tourner(-90);
  avancer(25);
  tourner(90);
  avancer(30);
  tourner(90);
  avancer(25);
  tourner(-50);
  avancer(50);
  tourner(-90);
  avancer(60);
  tourner(45);
  avancer(140);



  u_turn();


  avancer(140);
  tourner(-45);
  avancer(60);
  tourner(90);
  avancer(50);
  tourner(50);
  avancer(25);
  tourner(-90);
  avancer(30);
  tourner(-90);
  avancer(25);
  tourner(90);
  avancer(240);
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
  MOTOR_SetSpeed(RIGHT, 0.42);
  total_D=0;
  total_G=0;
  
  while(total_G < clics)
  {
    delay(100);
    PID();
   
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);

}

void PID ()
{
  pulse_D=ENCODER_Read(RIGHT);
  pulse_G=ENCODER_Read(LEFT);
  
  total_G += pulse_G; 
  total_D += pulse_D;
  erreur_1 = total_G - total_D;
  erreur_vitesse = pulse_G -pulse_D;
  correction= kp*erreur_1+erreur_vitesse*ki;
  MOTOR_SetSpeed(RIGHT,(0.5+correction));
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}

void u_turn()
{
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
  MOTOR_SetSpeed(LEFT, -0.5);
  MOTOR_SetSpeed(RIGHT, 0.5);

  while(ENCODER_Read(RIGHT)<3650){}
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}
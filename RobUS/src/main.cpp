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
void aller_bleu();
void aller_jaune();
void aller_rouge();
//fonction détecter couleur. elle renvoie 0 (bleu), 1 (rouge), 2 (jaune).

void setup() {
  BoardInit();

/* déposer la balle:

idée

étape #1: faire une fonction qui renvoie un chiffre, 0, 1 ou 2 
      après avoir détecté la couleur.

étape #2: selon ce chiffre, faire trois fonctions: aller_(bleu, jaune, ou rouge)


détecter la couleur
allumer DEL de la couleur détecter
jusqu'au ballon
prendre ballon
aller à la bonne couleur
lâcher ballon */

//fonction touver_couleur

if(trouver_couleur==0)
{
  aller_bleu();
}
else if(trouver_couleur==1)
{
  aller_rouge();
}
else
{
  aller_jaune();
}





}

void loop(){}

void aller_bleu()
{
  //allumer DEL bleu
  avancer(40);
  //prendre baller avec servos moteurs
  tourner(-90);
  avancer(70);
  tourner(90);
  avancer(225); //jusqu'à la case bleue
  //lâcher la balle

}

void aller_rouge()
{
  //allumer DEL rouge
  avancer(40);
  //prendre balle avec servos moteurs
  avancer(225); //jusqu'à la case rouge
  //lâcher balle

}

void aller_jaune()
{
  //allumer DEL jaune
   avancer(40);
  //prendre balle avec servos moteurs
  tourner(90);
  avancer(70);
  tourner(-90);
  avancer(225); //jusqu'à la case jaune
  //lâcher balle

}

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
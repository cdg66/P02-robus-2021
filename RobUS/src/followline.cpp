#include "followline.hpp"
#include <librobus.h>
//#include <Arduino.h>
#include <math.h>
#include "SoftTimer.hpp"
#include "mouvement.hpp"
#include "minedetection.hpp"
// private define
#define PIN_FOLLOW_RED 37
#define PIN_FOLLOW_YELLOW 38
#define PIN_FOLLOW_BLUE 39
#define PIN_FOLLOW_INTERSECT 40
// private variables
float SPEED_SUIVEUR = -0.4;
// private function prototype
uint8_t getFollowLineValue();

/*------------------------------------------------- followLineInit ------
|  Function followLineInit
|
|  Purpose:  
|
|  Parameters: None
|  Constant :
|       Nothing
|  Variables : Can change the value of the sensor pins in the #define 
| 
|  Dependency : Arduino ( digitalRead())
|  Returns:    nothing
*-------------------------------------------------------------------*/
void followLineInit() {
  pinMode(PIN_FOLLOW_RED, INPUT);
  pinMode(PIN_FOLLOW_YELLOW, INPUT);
  pinMode(PIN_FOLLOW_BLUE, INPUT);
}

/*------------------------------------------------- getFollowLineValue ------
|  Function getFollowLineValue
|
|  Purpose:  Get the value returned from the follow line sensor by taking the value of each sensor ( 3 sensors return 3 values => 1 , 2 , 4. then those are added together to form a number from 0 to 7 )
|
|  Parameters: None
|  Constant :
|       Nothing
|  Variables : Can change the value of the sensor pins in the #define 
| 
|  Dependency : Arduino ( digitalRead())
|  Returns:    value of sensors where
|               1 = blue
|               2 = yellow
|               3 = blue & yellow
|               4 = red
|               5 = blue & red
|               6 = yellow & red
|               7 = blue & yellow & red
| Warning the function asume that input pin are sequential  to read them
*-------------------------------------------------------------------*/
uint8_t getFollowLineValue() {
  int i;
  uint8_t valuePin, tempValue = 0;
  for (i = 0; i < 3; i++)
  {
    tempValue = digitalRead(PIN_FOLLOW_RED + i);
    valuePin = valuePin << 1;
    valuePin += tempValue;
  }
  return valuePin;
}

void followLineCallback(void)
{
  static int16_t CompteurCallback = 0; 
  uint8_t ValeurSuiveur;
  static float speedL = SPEED_SUIVEUR;
  static float speedR = SPEED_SUIVEUR;
  if (CompteurCallback == 0)
  {
    //Serial.print("entre init callback\n");
    MOTOR_SetSpeed(LEFT, speedL);
    MOTOR_SetSpeed(RIGHT, speedR);
    CompteurCallback++;
    return;
  }
  //Serial.print("entre  callback\n");
  CompteurCallback++;
  ValeurSuiveur = getFollowLineValue();
  //Serial.print(ValeurSuiveur, DEC );
  //Serial.print("\n");
  switch (ValeurSuiveur)
  {
    case 0: // in a pas de ligne on avance pendant x temps. Apres on arrete.
      speedL = SPEED_SUIVEUR;
      speedR = SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
      if (CompteurCallback >= 500)
      {
        speedL = 0;
        speedR = 0;
        MOTOR_SetSpeed(LEFT, speedL);
        MOTOR_SetSpeed(RIGHT, speedR);
      }
    break;
    case 1: // on est trop a gauche on tourne beaucoup a droite
      CompteurCallback = 1;
      speedL = SPEED_SUIVEUR + 0.2;
      speedR = speedR - SPEED_SUIVEUR;
      if (speedR > 0)
      {
        speedR = 0;
      }
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 2: // on avance 
      CompteurCallback = 1;
      speedL = SPEED_SUIVEUR;
      speedR = SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 3: // on est un peu a gauche on tourne un peu a droite
      CompteurCallback = 1;
      speedL = SPEED_SUIVEUR + 0.2;
      speedR = speedR - SPEED_SUIVEUR;
      if (speedR > 0)
      {
        speedR = 0;
      }
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 4: // on est trop a droite, on tourne beaucoup a gauche
      CompteurCallback = 1;
      speedR = SPEED_SUIVEUR + 0.1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL > 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 5: // erreur gauche et droit sont actif mais pas celui du centre 
      CompteurCallback = 1;
     /*  speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR); */
    break;
    case 6: // on est un peu a droite, on tourne un peu a gauche
      CompteurCallback = 1;
      speedR = SPEED_SUIVEUR + 0.1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL > 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 7: // erreur
      CompteurCallback =1;

    break;
    default: // erreur 
      CompteurCallback = 1;
      /*speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR); */
    break;
  }
}


void GetBackOnLineCallback(void)
{
  static int16_t CompteurCallback = 0;
  static int16_t CompteurValue = 0;
  
  static int fin = 0;
  uint8_t ValeurSuiveur;
  static float speedL = SPEED_SUIVEUR;
  static float speedR = SPEED_SUIVEUR;

  if (CompteurCallback == 0)
  {
    //Serial.print("entre init callback\n");
    MOTOR_SetSpeed(LEFT, speedL);
    MOTOR_SetSpeed(RIGHT, speedR);
    CompteurCallback++;
    return;
  }
  //Serial.print("entre  callback\n");
  CompteurCallback++;
  ValeurSuiveur = getFollowLineValue();
  if (ValeurSuiveur == 0)// est-ce qu'on a deja trouvee la ligne?
  {
//    if (ValeurSuiveur == 0) // est-ce qu'on est sur la ligne
//    {
//     MOTOR_SetSpeed(LEFT, speedR);
//      MOTOR_SetSpeed(RIGHT, speedR);
      CompteurValue = 0;
      return;
//    }
  }
  CompteurValue++;
  if (CompteurValue > 5)
  {
    do
    {
      //tournerSelf(5,0.2);
      
      tournerSuiveur(5);
    }while (getFollowLineValue() != 2 );
      //
    //SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &GoToCollorCallback);
    SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &followLineCallback);
    if(fin == 0)
    { 
      fin++;
      SOFT_TIMER_Enable(ID_INTERSECTION);
    }
    
    CompteurCallback = 0;
    
    
    //SOFT_TIMER_Disable(ID_QUILLE);
    }
}

void intersection(void)
{
  if(digitalRead(PIN_FOLLOW_INTERSECT) == 1)
  {
    SOFT_TIMER_Disable(ID_INTERSECTION);
    AX_BuzzerON(444, 1000);
    SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
    SPEED_SUIVEUR = 0.3;
    tourner(-90);
    SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);

    chercheCouleur = 1;

    /*
    for(int i = 0; i < 500; i++)
    {
      SOFT_TIMER_Update();
      delay(2);
    }
    delay(500);
    SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
    avancer_distance(52);
    SOFT_TIMER_Enable(ID_COULEUR);
    */
  }

}
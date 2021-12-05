#include <librobus.h>
#include <Arduino.h>
#include <math.h>
#include "magsensor.hpp"
// #include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "mouvement.hpp"
#include "SoftTimer.hpp"
#include "followline.hpp"
#include "distance.hpp"
#include "servo.hpp"
#include "minedetection.hpp"
#include "manuel.hpp"
#include "drapeau.hpp"
#define VERSIONID "Version je veux me tuer\n"
/* 
Avant de compiler ajouter les librairies:
- Adafruit PWM Servo Driver Library 
- TLV493D-A1B6

Pour les ajouter dans votre projet sur PIO Home 
-cliquez libraries sur le cote gauches
-Copier-coller une des libraries si dessus dans la barre de recherche 
-cliquez sur add to project
-selectionner ce projet
-cliquez sur Add
- repetez pour l'autre librairie.
*/

#define PIN_FOLLOW_RED 37
#define PIN_FOLLOW_YELLOW 38
#define PIN_FOLLOW_BLUE 39
#define PIN_FOLLOW_INTERSECT 40

//define pour les code couleur
#define BLEU 0
#define ROUGE 1
#define JAUNE 2

#define PIN_LED_RED  10
#define PIN_LED_YELLOW 11
#define PIN_LED_BLUE 43
#define PIN_LED_GREEN 41


#define micSon  A4 // entre analogique du 5khz
#define micAmb  A5 // entree analogique du son ambiant

uint16_t r, g, b, c;
uint8_t i;
uint8_t chercheCouleur = 0;
extern float SPEED_SUIVEUR;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_60X);


extern struct _mine mine;

float valeurSonar;



void modeManuel();
void modeAuto();

void setup() {
  BoardInit();
  Serial.write(VERSIONID);

  manuel_init();
  manuelStart();
  


  
  MagSensor_Init();
  SERVO_Init();
  drapeaux_LockAll();
  followLineInit();
  mineDetection_Init();
  mineDetection_Enable();




  // init pid
  SOFT_TIMER_SetCallback(ID_PID, &pid);
  SOFT_TIMER_SetDelay(ID_PID, 100);
  SOFT_TIMER_SetRepetition(ID_PID, -1);

  //SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &GoToCollorCallback);
  SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &followLineCallback);
  SOFT_TIMER_SetDelay(ID_SUIVEURDELIGNE, 2);
  SOFT_TIMER_SetRepetition(ID_SUIVEURDELIGNE, -1);
  //SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
  



  pinMode(PIN_LED_RED,OUTPUT);
  pinMode(PIN_LED_YELLOW,OUTPUT);
  pinMode(PIN_LED_BLUE,OUTPUT);
  pinMode(PIN_LED_GREEN,OUTPUT);

  // pour test 
  pinMode(13,OUTPUT);
}


void loop()
{
  SOFT_TIMER_Update();
  if (mine.mineDetected >= 1)
  {
    digitalWrite(13,HIGH);
  }
  else
  {
    digitalWrite(13,LOW);
  }

}


void modeManuel()
{
  SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT, 0);

  manuelStart();
  mineDetection_Enable();
}

void modeAuto()
{
  manuelStop();

  SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
}


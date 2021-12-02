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



void readMicrophone();

void setup() {
  BoardInit();
  Serial.write(VERSIONID);

  MOTOR_SetSpeed(0, 1);

  manuel_init();
  manuelStart();
  
  Serial.available();


  
  //MagSensor_Init();
  //SERVO_Init();
  //followLineInit();
  //mineDetection_Init();
  //mineDetection_Enable();





  // init pid
  SOFT_TIMER_SetCallback(ID_PID, &pid);
  SOFT_TIMER_SetDelay(ID_PID, 100);
  SOFT_TIMER_SetRepetition(ID_PID, -1);

  //SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &GoToCollorCallback);
  SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &followLineCallback);
  SOFT_TIMER_SetDelay(ID_SUIVEURDELIGNE, 2);
  SOFT_TIMER_SetRepetition(ID_SUIVEURDELIGNE, -1);
  //SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
  
  SOFT_TIMER_SetCallback(ID_MICRO, &readMicrophone);
  SOFT_TIMER_SetDelay(ID_MICRO, 20);
  SOFT_TIMER_SetRepetition(ID_MICRO, -1);
  //SOFT_TIMER_Enable(ID_MICRO);



  pinMode(PIN_LED_RED,OUTPUT);
  pinMode(PIN_LED_YELLOW,OUTPUT);
  pinMode(PIN_LED_BLUE,OUTPUT);
  pinMode(PIN_LED_GREEN,OUTPUT);

  // pour test 
  pinMode(13,OUTPUT);
  //tourner(-90);
  //avancer_distance(320);
  //MagSensor_Init();

  // test 
  //avancer_distance(10);
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


void readMicrophone( ) 
{ /* function readMicrophone : allume les led quand le sifflet est détecté*/
  static int mic_son_val,mic_amb_val;
  mic_son_val = analogRead(micSon);
  mic_amb_val = analogRead(micAmb);
  int difference_son = mic_son_val - mic_amb_val ; 

  //Serial.print(F("mic 5k ")); Serial.println(mic_son_val); // affiche les valeurs analog pour le debugguage
  //Serial.print(F("mic amb ")); Serial.println(mic_amb_val);
  Serial.print(F("mic diff ")); Serial.println(difference_son);
  if (difference_son >= 60) 
  {
    //SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
    //Serial.println("mic detected"); 
    //Ouverture du voltage des LEDS
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH);
    SOFT_TIMER_Enable(ID_QUILLE);
    //compteurCallback++;
    SOFT_TIMER_Disable(ID_MICRO);
    //SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
  }
  
}


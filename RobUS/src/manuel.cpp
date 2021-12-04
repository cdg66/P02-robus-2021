#include "manuel.hpp"
#include "LibRobus.h"
#include "SoftTimer.hpp"
#include "drapeau.hpp"
#include "main.cpp"

String msg;
float speedG, speedD;
int drapeau, numDrapeau;
const unsigned int MAX_INPUT = 50;
char character;


void processBluetooth(String msg);
void sendMessage(String message);

//Fonction pour initialiser le bluetooth
void manuel_init()
{
  numDrapeau = 4;
  Serial3.begin(115200);
  msg = "";

   SOFT_TIMER_SetCallback(ID_MANUEL, &lireBluetooth);
  SOFT_TIMER_SetDelay(ID_MANUEL, 20);
  SOFT_TIMER_SetRepetition(ID_MANUEL, -1);
}

void manuelStart()
{
  isModeMan = true;
  SOFT_TIMER_Enable(ID_MANUEL);
  sendMessage(numDrapeau + "");
}

void manuelStop()
{
  isModeMan = false;
  SOFT_TIMER_Disable(ID_MANUEL);
}

//Fonction qui envoie le bon message à l'application
//pour update le status.
//Paramètre true si le robot détecte une mine, false sinon
void mineStatus(bool status)
{
  if(isModeMan)
  {
    if(status)
      sendMessage("mine");
    else
      sendMessage("pas mine");
  }
  else
  {
    if(status)
    {
      SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
      MOTOR_SetSpeed(LEFT, 0);
      MOTOR_SetSpeed(RIGHT, 0);
      delay(50);

      dropDrapeau(numDrapeau);
      delay(50);
      SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
    }
      
  }
}

//Fonction pour le callback appeler au 20ms pour un bon fonctionnement
void lireBluetooth()
{
  while(Serial3.available())
  {
    character = (char)Serial3.read();

    if(character == '#')
    {
      //Serial.print(msg);
      //Serial.print("\n");
      processBluetooth(msg);
      msg = "";
      return;
    }
    
     msg.concat(character);  
  }
}


//Fonction privé qui sert à envoyer le message voulu à l'application mobile
void sendMessage(String message)
{
  message += "\n";
  for(int i = 0; i < message.length(); i++)
  {
    Serial3.write(message.charAt(i));
  }

}

//Fonction privé qui analyse le message reçu de l'application et prend les actions nécessaire
void processBluetooth(String msg)
{
  if(msg.equals("auto"))
    modeAuto();
  else if(msg.equals("man"))
    modeManuel();
  else{    
  speedG = msg.substring(0, msg.indexOf(',')).toFloat();
  speedD = msg.substring(msg.indexOf(',')+1, msg.lastIndexOf(',')).toFloat();
  drapeau = msg.substring(msg.lastIndexOf(',')+1, msg.length()).toFloat();
  }
  
  //Serial.print(speedG);
  //Serial.print("\n");
  MOTOR_SetSpeed(LEFT, speedG);
  MOTOR_SetSpeed(RIGHT, speedD);
  
  if(drapeau >= 0)
    dropDrapeau(drapeau);
  
}

void dropDrapeau(int drapeau)
{
  drapeaux_Drop(4-drapeau);


    AX_BuzzerON(1976, 67.5);
    delay(75);
    AX_BuzzerOFF();

    AX_BuzzerON(2637, 472.5);
    delay(525);
    AX_BuzzerOFF();
    numDrapeau = drapeau - 1;
}




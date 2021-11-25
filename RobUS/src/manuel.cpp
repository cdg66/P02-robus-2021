#include "manuel.hpp"
#include "LibRobus.h"

String msg;
float speedG, speedD;
int drapeau;
const unsigned int MAX_INPUT = 50;
char character;


void processBluetooth(String msg);
void sendMessage(String message);



//Fonction qui envoie le bon message à l'application
//pour update le status.
//Paramètre true si le robot détecte une mine, false sinon
void mineStatus(bool status)
{
  if(status)
    sendMessage("mine");
  else
    sendMessage("pas mine");
}

//Fonction pour le callback appeler au 20ms pour un bon fonctionnement
void lireBluetooth()
{
  while(Serial3.available())
  {
    character = (char)Serial3.read();

    if(character == '#')
    {
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
  

  MOTOR_SetSpeed(LEFT, speedG);
  MOTOR_SetSpeed(RIGHT, speedD);
  
  if(drapeau > 0){
    dropDrapeau(drapeau);
    AX_BuzzerON(444, 500);
  }
  
}

void modeAuto()
{

}

void modeManuel()
{

}

void dropDrapeau(int drapeau)
{

}


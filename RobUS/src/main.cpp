#include <librobus.h>
#define VERSIONID "Version main"
void setup() {
  // put your setup code here, to run once:
  BoardInit();
  Serial.write(VERSIONID);

  avancer(2400);
  tourner(-90);
  avancer(550);
  tourner(90);
  avancer(550);
  tourner(90);
  avancer(550);
  tourner(-45);
  avancer(750);
  tourner(-90);
  avancer(690);
  tourner(30);
  avancer(1120);



  tourner(180);


  avancer(1120);
  tourner(-30);
  avancer(690);
  tourner(90);
  avancer(750);
  tourner(45);
  avancer(550);
  tourner(-90);
  avancer(550);
  tourner(-90);
  avancer(550);
  tourner(90);
  avancer(2400;
}

void loop() {
  // put your main code here, to run repeatedly:
  //Ceci est un test
  
}
#include <librobus.h>
#define VERSIONID "Version main"
void setup() {
  // put your setup code here, to run once:
  BoardInit();
  Serial.write(VERSIONID);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
#ifndef _MOUVEMENT_H__
#define _MOUVEMENT_H__

#include <librobus.h>

// define
#define CLIC_DEGREE 44     //44
#define CLIC_CM 133.4                    
#define KP 0.002   // teste avec le robot A OK
#define KI 0.0000000000000
// public variables

// public function
void tourner(float angle);
void tournerSuiveur(float angle);
void tournerSelf(float angle,float speed);

void avancer_distance (float distance);
void avancer_timer (float distance);
void uTurn();
void pid();
void pidReset();
#endif
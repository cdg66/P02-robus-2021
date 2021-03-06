#ifndef _SERVO_H__
#define _SERVO_H__


// define pour les servo
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FERME 300
#define SERVO_OUVERT 380
#define TOURELLE_CENTRE 300
#define TOURELLE_GAUCHE 1
#define TOURELLE_DROITE 600
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define ID_TOURELLE 15
// public variables

// public function
void SERVO_Init(void);
void SERVO_setServoPulse(uint8_t ServoID, double pulse);
void SERVO_SetPWM(uint8_t ServoID, uint16_t pulselen);
#endif
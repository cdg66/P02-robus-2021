#include <librobus.h>
#include <Arduino.h>
#include <math.h>
#include <Tlv493d.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <SoftwareSerial.h>
#define VERSIONID "Version main"
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


#define CLIC_DEGREE 44.4444444444444     //44
#define CLIC_CM 133.4                    
#define KP 0.0001    // teste avec le robot A OK
#define KI 0.00002

// define pour les servo
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
// define pour les callbacks
#define ID_PID 1
#define ID_SUIVEURDELIGNE 2
#define ID_QUILLE 3
#define ID_MICRO 4
#define ID_COULEUR 5
#define ID_6 6
#define ID_7 7
#define ID_8 8
#define ID_9 9
#define ID_10 10
// define pour les pins du arduino
#define PIN_FOLLOW_RED 37
#define PIN_FOLLOW_YELLOW 38
#define PIN_FOLLOW_BLUE 39


#define PIN_LED_RED  10
#define PIN_LED_YELLOW 11
#define PIN_LED_BLUE 43
#define PIN_LED_GREEN 41


#define SPEED_SUIVEUR 0.3

#define micSon  A4 // entre analogique du 5khz
#define micAmb  A5 // entree analogique du son ambiant

// objet pour le Mag sensor
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
// objet pour le driver de servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
float valeurSonar;


// value of encoder needed for avancer() and pid()
int32_t totalG, totalD;
float vitesseD = 0.50;
// prototype de fonctions

// mouvements
void tourner(float angle);
void tournerSuiveur(float angle);
void avancer_distance (float distance);
void avancer_timer (float distance);
void uTurn();
void pid();
void pidReset();
//fonction Sonar
float getSonarRange(int idSensor);
// fonctions infrarouges
float getIrRange(int idSensor);
// fonction senseur magnetique
void MagSensor_GetData(float VectorArray[3]);
void MagSensor_Init(void);
// fonction pour driver servo
void SERVO_Init(Adafruit_PWMServoDriver *pwm);
void SERVO_setServoPulse(Adafruit_PWMServoDriver *pwm, uint8_t n, double pulse);
void SERVO_SetPWM(Adafruit_PWMServoDriver *pwm, uint8_t ServoID, uint16_t pulselen);
void aller_bleu();
void aller_jaune();
void aller_rouge();
void trouver_aller_couleur();
//fonction détecter couleur. elle renvoie 0 (bleu), 1 (rouge), 2 (jaune).

// fonctions pour suiveur de ligne
void followLineInit();
uint8_t getFollowLineValue();
void followLineCallback(void);
void GetBackOnLineCallback(void);
void GoToCollorCallback(void);

void renverser_quille();

void readMicrophone();

void setup() {
  BoardInit();
  Serial.write(VERSIONID);

  //SoftwareSerial BTSerial(16,17);
  //Serial2.begin(115200);
  //BTSerial.begin(115200);

  //while(true)
  //{
  //  delay(1000);
  //  Serial.println(Serial2.read());
    //Serial2.write('a');
    //Serial.print('y');
  //}
  /* BluetoothInit();
  while(true){
  String str = BLUETOOTH_read();
  delay(200);
  Serial.print(str);
  } */

  MagSensor_Init();
  SERVO_Init(&pwm);
  followLineInit();
  // init pid
  SOFT_TIMER_SetCallback(ID_PID, &pid);
  SOFT_TIMER_SetDelay(ID_PID, 100);
  SOFT_TIMER_SetRepetition(ID_PID, -1);

  SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &followLineCallback);
  SOFT_TIMER_SetDelay(ID_SUIVEURDELIGNE, 2);
  SOFT_TIMER_SetRepetition(ID_SUIVEURDELIGNE, -1);
  SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
  
  SOFT_TIMER_SetCallback(ID_MICRO, &readMicrophone);
  SOFT_TIMER_SetDelay(ID_MICRO, 20);
  SOFT_TIMER_SetRepetition(ID_MICRO, -1);
  SOFT_TIMER_Enable(ID_MICRO);

  SOFT_TIMER_SetCallback(ID_QUILLE, &renverser_quille);
  SOFT_TIMER_SetDelay(ID_QUILLE, 50);
  SOFT_TIMER_SetRepetition(ID_QUILLE, -1);
  //SOFT_TIMER_Enable(ID_QUILLE);
  pinMode(PIN_LED_RED,OUTPUT);
  pinMode(PIN_LED_YELLOW,OUTPUT);
  pinMode(PIN_LED_BLUE,OUTPUT);
  pinMode(PIN_LED_GREEN,OUTPUT);
}


void loop()
{
   //avancer_distance(200);
  SOFT_TIMER_Update();
}
void aller_bleu()
{
  //allumer DEL bleu
  avancer_distance(40);
  //prendre baller avec servos moteurs
  tourner(-90);
  avancer_distance(70);
  tourner(90);
  avancer_distance(225); //jusqu'à la case bleue
  //lâcher la balle

}

void aller_rouge()
{
  //allumer DEL rouge
  avancer_distance(40);
  //prendre balle avec servos moteurs
  avancer_distance(225); //jusqu'à la case rouge
  //lâcher balle

}

void aller_jaune()
{
  //allumer DEL jaune
   avancer_distance(40);
  //prendre balle avec servos moteurs
  tourner(90);
  avancer_distance(70);
  tourner(-90);
  avancer_distance(225); //jusqu'à la case jaune
  //lâcher balle

}
/*------------------------------------------------- tourner ----------
|  Function tourner
|
|  Purpose:  Make RobUS do a turn on himself with only one weel active
|
|  Parameters:
|     type float 
|          angle(IN) angle of witch the robot need to face after his turn
|          give a negative value to turn left and a positive one to
|          turn right.
|
|  Constant : 
|     CLIC_DEGREE: Number of encoder pulse for 1 degrees of turn.
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void tourner(float angle)
{
  int32_t clic = CLIC_DEGREE * abs(angle);
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

/*------------------------------------------------- tourner ----------
|  Function tourner
|
|  Purpose:  Make RobUS do a turn on himself with only one weel active
|
|  Parameters:
|     type float 
|          angle(IN) angle of witch the robot need to face after his turn
|          give a negative value to turn left and a positive one to
|          turn right.
|
|  Constant : 
|     CLIC_DEGREE: Number of encoder pulse for 1 degrees of turn.
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void tournerSuiveur(float angle)
{
  int32_t clic = CLIC_DEGREE * abs(angle);
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(angle<0)
  {
  MOTOR_SetSpeed(RIGHT, 0.2);

  while(ENCODER_Read(RIGHT)<clic)
  {}
  }
  else{
  MOTOR_SetSpeed(LEFT, 0.2);

  while(ENCODER_Read(LEFT)<clic)
  {}
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}

/*------------------------------------------------- avancer_distance ----------
|  Function avancer_distance
|
|  Purpose:  Make RobUS move forward to a fixed distance
|
|  Parameters:
|     type float 
|          distance(IN) distance in centimeter (cm) with the robot need
|          to move.
|
|  Constant :
|     CLIC_CM: number of pulse of the encoder for one cm forward
|  Dependency : LibRobUS
|  Returns:  nothing
*-------------------------------------------------------------------*/
void avancer_distance(float distance)
{

  int32_t clics = distance * CLIC_CM;
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
  MOTOR_SetSpeed(RIGHT, vitesseD); 
  MOTOR_SetSpeed(LEFT, 0.5);
  
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

void avancer_timer(float distance)
{

  int32_t clics = distance * CLIC_CM;
  ENCODER_ReadReset(LEFT);
  ENCODER_ReadReset(RIGHT);
  MOTOR_SetSpeed(RIGHT, vitesseD); 
  MOTOR_SetSpeed(LEFT, 0.5);
  
  totalD=0;
  totalG=0;
  SOFT_TIMER_Enable(ID_PID);
  while(totalG < clics)
  {
    SOFT_TIMER_Update();
  }
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  SOFT_TIMER_Disable(ID_PID);
  pidReset();
}

void getSonarRange()
{
  valeurSonar = SONAR_GetRange(1);

  Serial.println(valeurSonar);
  
  //delay(100);
}


//void testArretForce(){

//}
/*------------------------------------------------- pid ---------------
|  Function pid
|
|  Purpose:  feedback loop for the motor speed
|
|  Parameters: nothing
|  Constant :
|       KP : Multiplicative constant of the proportional factor need to 
|            be set by the user.
|       KI : Multiplicative constant of the Integral factor need to 
|            be set by the user.
|  Variables :
| 
|  Dependency : LibRobUS
|       
|  Returns:    nothing
*-------------------------------------------------------------------*/
void pid()
{
  static int32_t erreurP, erreurI , pulseG, pulseD, Correction;

  pulseD=ENCODER_ReadReset(RIGHT);
  pulseG=ENCODER_ReadReset(LEFT);
  
  totalG += pulseG; 
  totalD += pulseD;
  erreurP = pulseG - pulseD;
  erreurI = totalG -totalD;
  Correction = (erreurP*KP) + (erreurI*KI);
  vitesseD = vitesseD + Correction;
  MOTOR_SetSpeed(RIGHT,vitesseD);
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);

}
/*------------------------------------------------- pidReset ---------------
|  Function pid
|
|  Purpose:  reset pid values
|
|  Parameters: nothing
|  Constant : nothing
|  Variables :
| 
|  Dependency : LibRobUS
|       
|  Returns:    nothing
*-------------------------------------------------------------------*/
void pidReset()
{
  totalG = 0;
  totalD = 0;
  vitesseD = 0.50;
}
/*------------------------------------------------- uTurn ---------------
|  Function uTurn
|
|  Purpose:  make robUS do a full 180 degree.
|
|  Parameters: nothing
|  Constant :  nothing
|  Dependency : LibRobUS
|  Returns:    nothing
*-------------------------------------------------------------------*/
void uTurn()
{
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
  MOTOR_SetSpeed(LEFT, -0.5);
  MOTOR_SetSpeed(RIGHT, 0.5);

  while(ENCODER_Read(RIGHT)<3650){
    SOFT_TIMER_Update();
  }
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}
/*------------------------------------------------- MagSensor_Init ---
|  Function MagSensor_Init
|
|  Purpose:  Initialise le capteur magnetique
|
|  Parameters: nothing
|  Constant :  nothing
|  Dependency : Tlv493d.h
|  Returns:    nothing
*-------------------------------------------------------------------*/
void MagSensor_Init(void)
{
  Tlv493dMagnetic3DSensor.begin();
}
/*------------------------------------------------- MagSensor_GetData ---
|  Function MagSensor_Init
|
|  Purpose:  lis le capteur et retourne la valeur xyz du champ mag en miliTesla
|
|  Parameters: VectorArray pointer to an array of 3 float each one 
|              repectively correspond to the X,Y and Z of the magnetic feild
|  Constant :  nothing
|  Dependency : Tlv493d.h
|  Returns:    nothing
*-------------------------------------------------------------------*/
void MagSensor_GetData(float VectorArray[3])
{
  Tlv493dMagnetic3DSensor.updateData();
  VectorArray[1] = Tlv493dMagnetic3DSensor.getX();
  VectorArray[2] = Tlv493dMagnetic3DSensor.getY();
  VectorArray[3] = Tlv493dMagnetic3DSensor.getZ();
}
/*------------------------------------------------- SERVO_Init ------
|  Function SERVO_Init
|
|  Purpose:  Initialise le PCA9685 servo driver
|
|  Parameters: objet pwm s'utilise SERVO_Init(&pwm);
|  Constant :  nothing
|  Dependency : Adafruit_PWMServoDriver.h
|  Returns:    nothing
*-------------------------------------------------------------------*/
void SERVO_Init(Adafruit_PWMServoDriver *pwm)
{
  pwm->begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm->begin();
  pwm->setOscillatorFrequency(26300000);
  pwm->setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

}
// Utilise a tes propre risque non teste, fait partit des example de la librairie
// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void SERVO_setServoPulse(Adafruit_PWMServoDriver *pwm, uint8_t n, double pulse) 
{
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm->setPWM(n, 0, pulse);
}
/*------------------------------------------------- SERVO_Init ------
|  Function SERVO_Init
|
|  Purpose:  set le PWM pour un channel (0 a 15) le pulse pour faire 
|            bouger le servo
|
|  Parameters: objet pwm (&pwm);
|              ServoID le channel sur lequel le pwm va s'appliquer (0 a 15) plus haut que ca la fonction ne fera rien
|              pulselen set avec SERVOMIN et SERVOMAX et entre les deux
|  Constant :  nothing
|  Dependency : Adafruit_PWMServoDriver.h
|  Returns:    nothing
*-------------------------------------------------------------------*/
void SERVO_SetPWM(Adafruit_PWMServoDriver *pwm, uint8_t ServoID, uint16_t pulselen) // pulselen < 4096
{
  if (ServoID > 15) 
  {
    return;
  }
  if (pulselen > 4096)
  {
    pulselen = 4096;
  }
  pwm->setPWM(ServoID,pulselen , 0);
}
/*------------------------------------------------- getSonarRange ---
|  Function getSonarRange
|
|  Purpose:  Get the range of the specified sonar sensor
|
|  Parameters: ID of sonar sensor ( int ) where id => { 0 , 1 }
|  Constant :
|       Nothing
|  Variables :
| 
|  Dependency : LibRobUS 
|       
|  Returns:    Distance in cm ( float )
*-------------------------------------------------------------------*/
float getSonarRange(int idSensor) {
  if((idSensor <= 1) && (idSensor >= 0)) {
    return SONAR_GetRange(idSensor);
  }
  return 0;
}
/*------------------------------------------------- getIrRange ------
|  Function getIrRange
|
|  Purpose:  Get the range of the specified ir sensor, then calculate its value in cm
|
|  Parameters: ID of sensor ( int ) where id => { 0 , 1 , 2 , 3 }
|  Constant :
|       Nothing
|  Variables :
| 
|  Dependency : LibRobUS (https://swanrobotics.com/projects/gp2d12_project/) <== reasoning 
|                        (https://wiki.wpi.edu/images/images/1/13/Linearizing_Sharp_ranger_data.pdf)
|  Returns:    Distance in cm ( float ) 
|              Warning! It is the user responsbility
|              to verify the data integrity returned by the function.
|              Between 8 and 30 cm the data is accurate. 
|              Under the minimun
|              distance the value returned will be higer that actual.
|              Over 30 cm the data may be negative (ex.:-2335)
|              If the wrong idSensor is inputed the output will be 0. 
|              
*-------------------------------------------------------------------*/
float getIrRange(int idSensor) {
  if((idSensor <=3) && (idSensor >= 0)) {
    return ((6787.0 / (ROBUS_ReadIR(idSensor) - 3.0)) - 4.0);
  }
  return 0;
}

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
      if (speedR < 0)
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
      if (speedR < 0)
      {
        speedR = 0;
      }
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 4: // on est trop a droite, on tourne beaucoup a gauche
      CompteurCallback = 1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL < 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 5: // erreur gauche et droit sont actif mais pas celui du centre 
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 6: // on est un peu a droite, on tourne un peu a gauche
      CompteurCallback = 1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL < 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 7: // erreur
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR); 

    break;
    default: // erreur 
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
  }
}


void GetBackOnLineCallback(void)
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
  if (ValeurSuiveur == 0)// est-ce qu'on a deja trouvee la ligne?
  {
//    if (ValeurSuiveur == 0) // est-ce qu'on est sur la ligne
//    {
//     MOTOR_SetSpeed(LEFT, speedR);
//      MOTOR_SetSpeed(RIGHT, speedR);
      return;
//    }
  }
  do
  {
    tournerSuiveur(5);
  }while (getFollowLineValue() != 2);
  delay(100);
  SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &GoToCollorCallback);
  //SOFT_TIMER_Disable(ID_QUILLE);
}

void GoToCollorCallback(void)
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
      if (CompteurCallback >= 1000)
      {
        speedL = 0;
        speedR = 0;
        MOTOR_SetSpeed(LEFT, speedL);
        MOTOR_SetSpeed(RIGHT, speedR);
      }
    break;
    case 1: // on est trop a gauche on tourne beaucoup a droite
      CompteurCallback = 1;
      speedL =  SPEED_SUIVEUR + 0.2;
      speedR = speedR - SPEED_SUIVEUR;
      if (speedR < 0)
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
      if (speedR < 0)
      {
        speedR = 0;
      }
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 4: // on est trop a droite, on tourne beaucoup a gauche
      CompteurCallback = 1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL < 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 5: // erreur gauche et droit sont actif mais pas celui du centre 
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
    case 6: // on est un peu a droite, on tourne un peu a gauche
      tourner(-95);
      /* CompteurCallback = 1;
      speedL = speedL - SPEED_SUIVEUR;
      if (speedL < 0)
      {
        speedL = 0;
      }
      //speedR = speedR - SPEED_SUIVEUR;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR); */
    break;
    case 7: // on est a la feulle de couleur
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR); 
      SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
      
    break;
    default: // erreur 
      CompteurCallback = 1;
      speedL = 0;
      speedR = 0;
      MOTOR_SetSpeed(LEFT, speedL);
      MOTOR_SetSpeed(RIGHT, speedR);
    break;
  }
}
void renverser_quille()
{
  static float dis;
  //delay(1000);
  //Serial.println(getSonarRange(0));
  //avancer(10);
  dis = getSonarRange(0);
  if(dis>30)
  {
    return;
  }
  digitalWrite(PIN_LED_GREEN,HIGH);
  SOFT_TIMER_Disable(ID_SUIVEURDELIGNE);
  SOFT_TIMER_Disable(ID_QUILLE);
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
  tourner(75);
  //Serial.print(dis, 6);
  avancer_timer(dis + 5);
  uTurn();
  //uTurn();
  SOFT_TIMER_SetCallback(ID_SUIVEURDELIGNE, &GetBackOnLineCallback);
  SOFT_TIMER_Enable(ID_SUIVEURDELIGNE);
}


void readMicrophone( ) 
{ /* function readMicrophone : allume les led quand le sifflet est détecté*/
  static int mic_son_val,mic_amb_val;
  static int compteurCallback = 0;
  mic_son_val = analogRead(micSon);
  mic_amb_val = analogRead(micAmb);
  int difference_son = mic_son_val - mic_amb_val ; 

  //Serial.print(F("mic 5k ")); Serial.println(mic_son_val); // affiche les valeurs analog pour le debugguage
  //Serial.print(F("mic amb ")); Serial.println(mic_amb_val);
  Serial.print(F("mic diff ")); Serial.println(difference_son);
  if (difference_son >= 60) 
  {
    //Serial.println("mic detected"); 
    //Ouverture du voltage des LEDS
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH);
    SOFT_TIMER_Enable(ID_QUILLE);
    //compteurCallback++;
    SOFT_TIMER_Disable(ID_MICRO);
  }
  
}
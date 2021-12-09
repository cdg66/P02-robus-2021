#include <Adafruit_PWMServoDriver.h>
#include "servo.hpp"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
void SERVO_Init(void)
{
  pwm.begin();
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
  pwm.begin();
  pwm.setOscillatorFrequency(26300000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

}
// Utilise a tes propre risque non teste, fait partit des example de la librairie
// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void SERVO_setServoPulse(uint8_t ServoID, double pulse) 
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
  pwm.setPWM(ServoID, 0, pulse);
}
/*------------------------------------------------- SERVO_SetPWM ------
|  Function SERVO_SetPWM 
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
void SERVO_SetPWM(uint8_t ServoID, uint16_t pulselen) // pulselen < 4096
{
  if (ServoID > 15) 
  {
    return;
  }
  if (pulselen > 4096)
  {
    pulselen = 4096;
  }
  pwm.setPWM(ServoID,pulselen , 0);
}

void tourelleGauche()
{
  
}

void tourelleDroite()
{

}
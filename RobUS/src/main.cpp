#include <librobus.h>
#include <Arduino.h>
#define VERSIONID "Version main"
typedef  int32_t PID_Constant_Type;
typedef struct _PID
{
    float Kp;
    float Ki;
    float Kd;
    int32_t PError;
    int32_t LastPError;
    int32_t IError;
    int32_t DError;
    int32_t Goal;
    uint32_t TimeBaseMS; // delta of time(in ms) witch the PID_Copute function is called periodically

} PID_Handler;
void PID_Init(PID_Handler *PID, float ConstantP, float ConstantI, float ConstantD, PID_Constant_Type TimeBaseMs);
void PID_SetGoal(PID_Handler *PID, PID_Constant_Type Goal);
float PID_Compute(PID_Handler *PID, PID_Constant_Type InputData );
PID_Handler PID_Right;
void setup() {
  // put your setup code here, to run once:
  BoardInit();
  Serial.write(VERSIONID);
}

void loop() {
  // put your main code here, to run repeatedly:
   float SpeedAdjust = 0.0f;
  float SpeedR = 0.3f;
  int32_t LEnco;
  int32_t REnco;
  int i;
  BoardInit();

  PID_Init( &PID_Right, 0.000009f ,0.0000003f, 0.0f, 100);
  //PID_SetGoal(&PID_Right, 10000);
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
  MOTOR_SetSpeed(RIGHT, 0.3f);
  MOTOR_SetSpeed(LEFT, 0.3f);
  
  delay(100);
  LEnco = ENCODER_Read(LEFT);
  //Serial.print("L:");
  //Serial.print(LEnco,DEC);
  //Serial.print("\n\r");
  //Serial.print("R:");
  //Serial.print(REnco,DEC);
  //Serial.print("\n\r");  
  REnco = ENCODER_Read(RIGHT);
  PID_SetGoal(&PID_Right, LEnco );
  PID_Compute(&PID_Right, REnco);
  for (i = 0; i<70000; i++)
  {
    delay(100);
    LEnco = ENCODER_Read(LEFT);
    //Serial.print(LEnco);
    //Serial.print('\n');
    REnco = ENCODER_Read(RIGHT);
    //Serial.print(REnco);
    //Serial.print('\n');
    PID_SetGoal(&PID_Right, LEnco );
    SpeedAdjust = PID_Compute(&PID_Right, REnco);
    //Serial.print(SpeedR);
    //Serial.print('\n');
    SpeedR = SpeedR + SpeedAdjust;
    MOTOR_SetSpeed(RIGHT, SpeedR);
    
  }
  
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
}
void PID_Init(PID_Handler *PID, float ConstantP, float ConstantI, float ConstantD, PID_Constant_Type TimeBaseMs)
{
    PID->Kp = ConstantP;
    PID->Ki = ConstantI;
    PID->Kd = ConstantD;
    PID->LastPError = 0;
};
void PID_SetGoal(PID_Handler *PID, PID_Constant_Type Goal)
{
    PID->Goal = Goal;
};
float PID_Compute(PID_Handler *PID, PID_Constant_Type InputData )
{
    float Output;
    PID->PError = PID->Goal - InputData; //P
    PID->IError = PID->PError * PID->TimeBaseMS; //I
    PID->DError = (PID->PError - PID->LastPError)/ PID->TimeBaseMS; // D
    //somation
    Output = PID->Kp*PID->PError +  PID->Ki*PID->IError +  PID->Kd*PID->DError;
    //Serial.print(Output,10);
    //Serial.print("\n\r");
    PID->LastPError = PID->PError;

    return Output;
};
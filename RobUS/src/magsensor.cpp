#include <Tlv493d.h>
// objet pour le Mag sensor
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();

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
|              (out) Norm output the norm of the vector usful to detect if there is a magnetic feild present
|  Constant :  nothing
|  Dependency : Tlv493d.h
|  Returns:    nothing
*-------------------------------------------------------------------*/
void MagSensor_GetData(float VectorArray[3], float *Norm)
{
  float tempVector[3];
  float sum;
  int i;
  Tlv493dMagnetic3DSensor.updateData();
  VectorArray[0] = Tlv493dMagnetic3DSensor.getX();
  VectorArray[1] = Tlv493dMagnetic3DSensor.getY();
  VectorArray[2] = Tlv493dMagnetic3DSensor.getZ();

  for (i = 0; i<3; i++)
  {
    tempVector[i] = (VectorArray[i]*VectorArray[i]);
    //vector[i] = exp()
  }
  sum = tempVector[0]+tempVector[1]+tempVector[2];
  *Norm = sqrt((double)sum);
  //Serial.print("Norm: ");
  //Serial.print(norm, 6);
  //Serial.print("\n ");
  //sum = 0;
  //delay(1000);
}
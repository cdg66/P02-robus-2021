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
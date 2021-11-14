#include "distance.hpp"
#include "LibRobus.h"
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
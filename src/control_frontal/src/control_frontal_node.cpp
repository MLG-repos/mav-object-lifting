/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 * Curso EIR 2019                                                      * 
 *                                                                     *
 * Ejemplo de control de frontal del vehículo Ar drone simulado        *
 * usando GAZEBO                                                       *     
 ***********************************************************************/

#include "control_frontal.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_frontal");
  ControlFrontal OBJECT;
  
  ros::spin();
  return 0;
}

/***********************************
* File Name          : -
* Author             : Lorenzo Saccares
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
*********************************************************************************/

#include "HumanBodyParameter.h"

typedef struct JointTorque{

double	Torque_Hip;
double	Torque_Knee;			
double	Torque_Ankle;

} JointTorque;

/* Exported functions ------------------------------------------------------- */

JointTorque JointTorquefunction(HumanBodyParametr HB, float RollIMU, int Footsensor_AnkleTorque, double ATI_Torque);
